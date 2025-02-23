package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.RobotView;
import com.swrobotics.robot.subsystems.outtake.CoralOuttakeSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public final class SuperstructureSubsystem extends SubsystemBase {
    private static final NTBoolean CALIBRATE_PIVOT = new NTBoolean("Superstructure/Pivot/Encoder/Calibrate", false);
    private static final NTBoolean SYNC_PIVOT = new NTBoolean("Superstructure/Pivot/Encoder/Sync", false);

    public enum State {
        RECEIVE_CORAL_FROM_INDEXER(Constants.kElevatorHeightBottom, Constants.kOuttakePivotInAngle),
        SCORE_L1(Constants.kElevatorHeightL1, Constants.kOuttakePivotScoreL1Angle),
        SCORE_L2(Constants.kElevatorHeightL2, Constants.kOuttakePivotScoreL2Angle),
        SCORE_L3(Constants.kElevatorHeightL3, Constants.kOuttakePivotScoreL3Angle),
        SCORE_L4(Constants.kElevatorHeightL4, Constants.kOuttakePivotScoreL4Angle);

        public static State forScoring(int level) {
            return switch (level) {
                case 1 -> SCORE_L1;
                case 2 -> SCORE_L2;
                case 3 -> SCORE_L3;
                case 4 -> SCORE_L4;
                default -> throw new IndexOutOfBoundsException(level);
            };
        }

        private final Supplier<Double> elevatorHeightGetter;
        private final Supplier<Double> pivotAngleGetter;

        State(Supplier<Double> elevatorHeightGetter, Supplier<Double> pivotAngleGetter) {
            this.elevatorHeightGetter = elevatorHeightGetter;
            this.pivotAngleGetter = pivotAngleGetter;
        }

        public double getElevatorHeight() {
            return elevatorHeightGetter.get();
        }

        public double getPivotAngle() {
            return Units.degreesToRotations(pivotAngleGetter.get());
        }
    }

    private final ElevatorIO elevatorIO;
    private final ElevatorIO.Inputs elevatorInputs;
    private final OuttakePivotIO pivotIO;
    private final OuttakePivotIO.Inputs pivotInputs;

    private final CoralOuttakeSubsystem coralOuttakeSubsystem;

    private final Timer pivotSyncTimer;
    private State targetState;

    public SuperstructureSubsystem(CoralOuttakeSubsystem coralOuttakeSubsystem) {
        if (RobotBase.isReal()) {
            elevatorIO = new ElevatorIOReal();
            pivotIO = new OuttakePivotIOReal();
        } else {
            elevatorIO = new ElevatorIOSim();
            pivotIO = new OuttakePivotIOSim();
        }
        elevatorInputs = new ElevatorIO.Inputs();
        pivotInputs = new OuttakePivotIO.Inputs();

        this.coralOuttakeSubsystem = coralOuttakeSubsystem;

        pivotSyncTimer = new Timer();
        pivotSyncTimer.start();
        targetState = State.RECEIVE_CORAL_FROM_INDEXER;
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }

    public Command commandSetStateOnce(State targetState) {
        return Commands.runOnce(() -> setTargetState(targetState), this);
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        Logger.processInputs("Outtake Pivot", pivotInputs);

        if (pivotSyncTimer.advanceIfElapsed(1)) {
            pivotIO.syncWithEncoder();
        }

        if (CALIBRATE_PIVOT.get()) {
            CALIBRATE_PIVOT.set(false);
            pivotIO.calibrateEncoder();
        }

        if (SYNC_PIVOT.get()) {
            SYNC_PIVOT.set(false);
            pivotIO.syncWithEncoder();
        }

        RobotView.setSuperstructureState(elevatorInputs.currentHeightPct, pivotInputs.currentAngleRot);

        double elevatorCurrent = elevatorInputs.currentHeightPct;
        double pivotCurrent = pivotInputs.currentAngleRot;
        double elevatorTarget = targetState.getElevatorHeight();
        double pivotTarget = targetState.getPivotAngle();

        double elevatorTol = Constants.kElevatorCollisionTolerance.get();
        double elevatorCollisionHeight = Constants.kElevatorMaxHeightWithArmInBelowBar.get();
        double pivotTol = Units.degreesToRotations(Constants.kOuttakePivotCollisionTolerance.get());
        double pivotCollisionAngle = Units.degreesToRotations(Constants.kOuttakePivotMaxAngleNearBar.get());

        // When pivot is too far in:
        //   if pivot and target are below bar, no change
        //   if pivot above bar, target below bar: elevator not move
        //   if target above bar, pivot below bar: target is max collision height
        //   if both above bar: elevator not move

        // If pivot target is too far in:
        //   if elevator current and target below bar, go to target
        //   if elevator above, target below: get out of way (case should be impossible)
        //   if elevator below, target above: get out of way
        //   if elevator above, target above: get out of way (case should be impossible)

        if (pivotTarget > pivotCollisionAngle - pivotTol) {
            if (elevatorCurrent > elevatorCollisionHeight - elevatorTol || elevatorTarget > elevatorCollisionHeight - elevatorTol) {
                pivotTarget = pivotCollisionAngle - pivotTol;
            }
        }

        if (pivotCurrent > pivotCollisionAngle) {
            if (elevatorCurrent > elevatorCollisionHeight) {
                elevatorTarget = elevatorCurrent; // Don't move until the pivot gets out
            } else if (elevatorTarget > elevatorCollisionHeight - elevatorTol) {
                elevatorTarget = elevatorCollisionHeight - elevatorTol;
            }
        }

        RobotView.setTargetSuperstructureState(elevatorTarget, pivotTarget);

        boolean hasCoral = coralOuttakeSubsystem.hasPiece();

        if (elevatorTarget == 0.0 && Math.abs(elevatorInputs.currentHeightPct) < Constants.kElevatorTolerance.get()) {
            // Conserve battery power when we can
            elevatorIO.setNeutral();
        } else {
            elevatorIO.setTargetHeight(elevatorTarget);
        }
        pivotIO.setTargetAngle(pivotTarget, hasCoral);
    }

    public boolean isInTolerance() {
        boolean elevator = Math.abs(elevatorInputs.currentHeightPct - targetState.getElevatorHeight())
                < Constants.kElevatorTolerance.get();

        boolean pivot = Math.abs(pivotInputs.currentAngleRot - targetState.getPivotAngle())
                < Units.degreesToRotations(Constants.kOuttakePivotTolerance.get());

        Logger.recordOutput("Auto/Elevator In Tolerance", elevator);
        Logger.recordOutput("Auto/Elevator Error", Math.abs(elevatorInputs.currentHeightPct - targetState.getElevatorHeight()));
        Logger.recordOutput("Auto/Pivot In Tolerance", pivot);
        Logger.recordOutput("Auto/Pivot Error", Units.rotationsToDegrees(Math.abs(pivotInputs.currentAngleRot - targetState.getPivotAngle())));

        return elevator && pivot;
    }
}
