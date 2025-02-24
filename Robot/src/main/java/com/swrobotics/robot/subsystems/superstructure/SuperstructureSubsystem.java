package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.RobotView;
import com.swrobotics.robot.subsystems.outtake.CoralOuttakeSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
        SCORE_L4(Constants.kElevatorHeightL4, Constants.kOuttakePivotScoreL4Angle),
        PREP_CLIMB(Constants.kElevatorHeightClimbPrep, Constants.kOuttakePivotClimbAngle),
        CLIMB(Constants.kElevatorHeightClimbEnd, Constants.kOuttakePivotClimbAngle);

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

    private TrapezoidProfile elevatorProfile;
    private TrapezoidProfile pivotProfile;
    private TrapezoidProfile.State elevatorSetpoint;
    private TrapezoidProfile.State pivotSetpoint;

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

        updateElevatorProfile();
        updatePivotProfile();
        Constants.kElevatorMaxVelocity.onChange(this::updateElevatorProfile);
        Constants.kElevatorMaxAccel.onChange(this::updateElevatorProfile);
        Constants.kOuttakePivotMaxVelocity.onChange(this::updatePivotProfile);
        Constants.kOuttakePivotMaxAccel.onChange(this::updatePivotProfile);

        elevatorSetpoint = pivotSetpoint = null;
    }

    private void updateElevatorProfile() {
        elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Constants.kElevatorMaxVelocity.get() / Constants.kElevatorMaxHeightRotations,
                Constants.kElevatorMaxAccel.get() / Constants.kElevatorMaxHeightRotations
        ));
    }

    private void updatePivotProfile() {
        pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Constants.kOuttakePivotMaxVelocity.get(),
                Constants.kOuttakePivotMaxAccel.get()
        ));
    }

    public void setTargetState(State targetState) {
        if (targetState != this.targetState) {
            // Re-sync setpoints with actual state
            elevatorSetpoint = null;
            pivotSetpoint = null;
        }
        this.targetState = targetState;
    }

    public State getTargetState() {
        return targetState;
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

        // Set elevator climb mode
        elevatorIO.setClimbMode(targetState == State.CLIMB);

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

        double elevatorMaxDev = Constants.kElevatorDeviationTolerance.get();
        double elevatorAvoid = Constants.kElevatorCollisionTolerance.get();
        double elevatorCollisionFrame = Constants.kElevatorFrameCollisionHeight.get();
        double elevatorCollisionStage2 = Constants.kElevatorStage2CollisionHeight.get();
        double pivotMaxDev = Units.degreesToRotations(Constants.kOuttakePivotDeviationTolerance.get());
        double pivotAvoid = Units.degreesToRotations(Constants.kOuttakePivotCollisionTolerance.get());
        double pivotCollisionFrame = Units.degreesToRotations(Constants.kOuttakePivotFrameCollisionAngle.get());
        double pivotCollisionStage2 = Units.degreesToRotations(Constants.kOuttakePivotStage2CollisionAngle.get());

        // Reset setpoints to sensor values
        if (elevatorSetpoint == null || Math.abs(elevatorSetpoint.position - elevatorInputs.currentHeightPct) > elevatorMaxDev) {
            elevatorSetpoint = new TrapezoidProfile.State(elevatorInputs.currentHeightPct, elevatorInputs.currentVelocityPctPerSec);
        }
        if (pivotSetpoint == null || Math.abs(pivotSetpoint.position - pivotInputs.currentAngleRot) > pivotMaxDev) {
            pivotSetpoint = new TrapezoidProfile.State(pivotInputs.currentAngleRot, pivotInputs.currentVelocityRotPerSec);
        }

        // Cases:
        // - current and target below -> both to target
        // - current below, target above -> pivot out, delay before moving elevator up
        // - current above, target below -> pivot at collision + avoidance, if pivot is out elevator to target
        // - current and target above -> if pivot is out both to target, keep pivot out

        // Collision with frame crossbar
        if (elevatorCurrent < elevatorCollisionFrame) {
            if (elevatorTarget > elevatorCollisionFrame) {
                pivotTarget = Math.min(pivotTarget, pivotCollisionFrame - pivotAvoid);

                elevatorProfile.calculate(
                        0,
                        elevatorSetpoint,
                        new TrapezoidProfile.State(elevatorTarget, 0)
                );
                pivotProfile.calculate(
                        0,
                        pivotSetpoint,
                        new TrapezoidProfile.State(pivotTarget, 0)
                );

                if (pivotCurrent > pivotCollisionFrame) {
                    double pivotMoveTime = pivotProfile.timeLeftUntil(pivotCollisionFrame - pivotAvoid);
                    double elevatorMoveTime = elevatorProfile.timeLeftUntil(elevatorCollisionFrame - elevatorAvoid);
                    if (elevatorMoveTime < pivotMoveTime) {
                        // Wait for pivot to move some
                        elevatorTarget = elevatorCurrent;
                    }
                }
            }
        } else {
            pivotTarget = Math.min(pivotTarget, pivotCollisionFrame - pivotAvoid);

            if (pivotCurrent > pivotCollisionFrame) {
                // Hold elevator still until pivot gets out of collision area
                elevatorTarget = elevatorCurrent;
            }
        }

        // Collision with stage 2 crossbar
        if (elevatorCurrent < elevatorCollisionStage2) {
            if (elevatorTarget > elevatorCollisionStage2) {
                pivotTarget = Math.min(pivotTarget, pivotCollisionStage2 - pivotAvoid);

                elevatorProfile.calculate(
                        0,
                        elevatorSetpoint,
                        new TrapezoidProfile.State(elevatorTarget, 0)
                );
                pivotProfile.calculate(
                        0,
                        pivotSetpoint,
                        new TrapezoidProfile.State(pivotTarget, 0)
                );

                if (pivotCurrent > pivotCollisionStage2) {
                    double pivotMoveTime = pivotProfile.timeLeftUntil(pivotCollisionStage2 - pivotAvoid);
                    double elevatorMoveTime = elevatorProfile.timeLeftUntil(elevatorCollisionStage2 - elevatorAvoid);
                    if (elevatorMoveTime < pivotMoveTime) {
                        // Elevator is already moving so we don't want to wait
                        // in place, so continue up until the collision point.
                        // Shouldn't cause slowdown because pivot should be out
                        // by the time the elevator gets near stage 2 collision
                        elevatorTarget = elevatorCollisionStage2 - elevatorAvoid;
                    }
                }
            }
        } else {
            pivotTarget = Math.min(pivotTarget, pivotCollisionStage2 - pivotAvoid);

            if (pivotCurrent > pivotCollisionStage2) {
                // Hold elevator still until pivot gets out of collision area
                elevatorTarget = elevatorCurrent;
            }
        }

        RobotView.setTargetSuperstructureState(elevatorTarget, pivotTarget);

        // Update motion profiles
        elevatorSetpoint = elevatorProfile.calculate(Constants.kPeriodicTime, elevatorSetpoint, new TrapezoidProfile.State(elevatorTarget, 0));
        pivotSetpoint = pivotProfile.calculate(Constants.kPeriodicTime, pivotSetpoint, new TrapezoidProfile.State(pivotTarget, 0));

        RobotView.setSuperstructureSetpoint(elevatorSetpoint.position, pivotSetpoint.position);

        boolean hasCoral = coralOuttakeSubsystem.hasPiece();
        elevatorIO.setTarget(elevatorSetpoint.position, elevatorSetpoint.velocity);
        pivotIO.setTarget(pivotSetpoint.position, pivotSetpoint.velocity, hasCoral);
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
