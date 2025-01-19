package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.RobotView;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public final class SuperstructureSubsystem extends SubsystemBase {
    private static final NTBoolean CALIBRATE_PIVOT = new NTBoolean("Coral Outtake/Pivot/Encoder/Calibrate", false);

    public enum State {
        RECEIVE_CORAL_FROM_INDEXER(() -> 0.0, Constants.kOuttakePivotInAngle),
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

    private State targetState;

    public SuperstructureSubsystem() {
        if (RobotBase.isReal()) {
            elevatorIO = new ElevatorIOReal();
            pivotIO = new OuttakePivotIOReal();
        } else {
            elevatorIO = new ElevatorIOSim();
            pivotIO = new OuttakePivotIOSim();
        }
        elevatorInputs = new ElevatorIO.Inputs();
        pivotInputs = new OuttakePivotIO.Inputs();

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

        if (CALIBRATE_PIVOT.get()) {
            CALIBRATE_PIVOT.set(false);
            pivotIO.calibrateEncoder();
        }

        RobotView.setSuperstructureState(elevatorInputs.currentHeightMeters, pivotInputs.currentAngleRot);

        double elevatorCurrent = elevatorInputs.currentHeightMeters;
        double pivotCurrent = pivotInputs.currentAngleRot;
        double elevatorTarget = targetState.getElevatorHeight();
        double pivotTarget = targetState.getPivotAngle();

        double collisionBottom = Constants.kElevatorMaxHeightWithArmInBelowBar.get();
        double collisionTop = Constants.kElevatorMinHeightWithArmInAboveBar.get();
        double collisionPivot = Units.degreesToRotations(Constants.kOuttakePivotMaxAngleNearBar.get());
        double elevatorTolerance = Constants.kElevatorCollisionTolerance.get();
        double pivotTolerance = Constants.kOuttakePivotCollisionTolerance.get();

        // Prevent collision with elevator crossbar
        boolean sameSideOfBar = (elevatorCurrent < collisionBottom && elevatorTarget < collisionBottom)
                || (elevatorCurrent > collisionTop && elevatorTarget > collisionTop);
        if (!sameSideOfBar) {
            // Pivot must get out of the way of the bar
            // Subtract tolerance so it goes a little farther out than necessary
            pivotTarget = Math.min(pivotTarget, collisionPivot - Units.degreesToRotations(pivotTolerance));

            boolean pivotIsOutOfWay = pivotCurrent <= collisionPivot;
            if (!pivotIsOutOfWay) {
                // Wait for pivot to move
                if (elevatorCurrent < collisionBottom)
                    elevatorTarget = collisionBottom - elevatorTolerance;
                else if (elevatorCurrent > collisionTop)
                    elevatorTarget = collisionTop + elevatorTolerance;
                else
                    elevatorTarget = elevatorCurrent;
            }
        }

        if (elevatorTarget == 0.0 && Math.abs(elevatorInputs.currentHeightMeters) < Constants.kElevatorTolerance.get()) {
            // Conserve battery power when we can
            elevatorIO.setNeutral();
        } else {
            elevatorIO.setTargetHeight(elevatorTarget);
        }
        pivotIO.setTargetAngle(pivotTarget);
    }

    public boolean isInTolerance() {
        boolean elevator = Math.abs(elevatorInputs.currentHeightMeters - targetState.getElevatorHeight())
                < Constants.kElevatorTolerance.get();

        boolean pivot = Math.abs(pivotInputs.currentAngleRot - targetState.getPivotAngle())
                < Constants.kOuttakePivotTolerance.get();

        return elevator && pivot;
    }
}
