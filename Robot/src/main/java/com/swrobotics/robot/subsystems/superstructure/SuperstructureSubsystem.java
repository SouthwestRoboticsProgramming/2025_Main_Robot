package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.RobotView;
import com.swrobotics.robot.subsystems.outtake.OuttakeSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
        BOTTOM(() -> 0.0, Constants.kOuttakePivotInAngle),
        RECEIVE_CORAL_FROM_INDEXER(Constants.kElevatorIndexerHeight, Constants.kOuttakePivotInAngle),
        SCORE_L1(Constants.kElevatorHeightL1, Constants.kOuttakePivotScoreL1Angle),
        SCORE_L2(Constants.kElevatorHeightL2, Constants.kOuttakePivotScoreL2Angle),
        SCORE_L3(Constants.kElevatorHeightL3, Constants.kOuttakePivotScoreL3Angle),
        SCORE_L4(Constants.kElevatorHeightL4, Constants.kOuttakePivotScoreL4Angle),
        PREP_CLIMB(Constants.kElevatorHeightClimbPrep, Constants.kOuttakePivotClimbAngle),
        CLIMB(Constants.kElevatorHeightClimbEnd, Constants.kOuttakePivotClimbAngle),
        PICKUP_LOW_ALGAE(Constants.kElevatorHeightLowAlgae, Constants.kOuttakePivotLowAlgaeAngle),
        PICKUP_HIGH_ALGAE(Constants.kElevatorHeightHighAlgae, Constants.kOuttakePivotHighAlgaeAngle),
        SCORE_NET(Constants.kElevatorHeightNet, Constants.kOuttakePivotNetAngle);

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
        private double pivotAdjustDeg = 0.0;
        private double elevatorAdjust = 0.0;
        private double pivotAimDeg = 0.0;
        private double elevatorAim = 0.0;

        State(Supplier<Double> elevatorHeightGetter, Supplier<Double> pivotAngleGetter) {
            this.elevatorHeightGetter = elevatorHeightGetter;
            this.pivotAngleGetter = pivotAngleGetter;
        }

        public double getElevatorHeight() {
            return MathUtil.clamp(elevatorHeightGetter.get() + elevatorAdjust + elevatorAim, 0.0, 1.0);
        }

        public double getPivotAngle() {
            return Units.degreesToRotations(pivotAngleGetter.get() + pivotAdjustDeg + pivotAimDeg);
        }

        public void setPivotAdjust(double pivotAdjustDeg) {
            this.pivotAdjustDeg = pivotAdjustDeg;
        }

        public void setElevatorAim(double elevatorAim) {
            this.elevatorAim = elevatorAim;
        }

        public void setPivotAim(double pivotAimDeg) {
            this.pivotAimDeg = pivotAimDeg;
        }

        public void setElevatorAdjust(double elevatorAdjust) {
            this.elevatorAdjust = elevatorAdjust;
        }
    }

    private final ElevatorIO elevatorIO;
    private final ElevatorIO.Inputs elevatorInputs;
    private final OuttakePivotIO pivotIO;
    private final OuttakePivotIO.Inputs pivotInputs;

    private final OuttakeSubsystem outtakeSubsystem;

    private final Timer pivotSyncTimer;
    private State targetState;
    private double pivotAdjustDeg;
    private double pivotAim;
    private double elevatorAdjust;
    private double elevatorAim;

    private TrapezoidProfile elevatorProfile;
    private TrapezoidProfile pivotProfile;
    private TrapezoidProfile.State elevatorSetpoint;
    private TrapezoidProfile.State pivotSetpoint;

    private double prevElevatorSetpointVelocity;

    public SuperstructureSubsystem(OuttakeSubsystem outtakeSubsystem) {
        if (RobotBase.isReal()) {
            elevatorIO = new ElevatorIOReal();
            pivotIO = new OuttakePivotIOReal();
        } else {
            elevatorIO = new ElevatorIOSim();
            pivotIO = new OuttakePivotIOSim();
        }
        elevatorInputs = new ElevatorIO.Inputs();
        pivotInputs = new OuttakePivotIO.Inputs();

        this.outtakeSubsystem = outtakeSubsystem;

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

    public void setPivotAdjust(double pivotAdjustDeg) {
        this.pivotAdjustDeg = pivotAdjustDeg;
    }

    public void setPivotAim(double pivotAim) {
        this.pivotAim = pivotAim;
    }

    public void setElevatorAdjust(double elevatorAdjust) {
        this.elevatorAdjust = elevatorAdjust;
    }

    public void setElevatorAim(double elevatorAim) {
        this.elevatorAim = elevatorAim;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }

    public Command commandSetStateOnce(State targetState) {
        return Commands.runOnce(() -> setTargetState(targetState), this);
    }

    private void handleFeedbackSensors() {
        double elevatorCurrent = elevatorInputs.currentHeightPct;
        double pivotCurrent = pivotInputs.currentAngleRot;

//        boolean up = Math.abs(pivotCurrent - 0.25) < (5.0 / 360.0);
        if (pivotSyncTimer.advanceIfElapsed(1) && pivotInputs.absoluteAngleOK || SYNC_PIVOT.get()) {
            SYNC_PIVOT.set(false);

            pivotCurrent = pivotInputs.absoluteAngleRot;
            pivotIO.setCurrentPosition(pivotCurrent);

            // Reset setpoint to sensor position
            pivotSetpoint = null;
        }

        // Reset setpoints to sensor values
        double elevatorMaxDev = Constants.kElevatorDeviationTolerance.get();
        double pivotMaxDev = Units.degreesToRotations(Constants.kOuttakePivotDeviationTolerance.get());
        if (elevatorSetpoint == null/* || Math.abs(elevatorSetpoint.position - elevatorCurrent) > elevatorMaxDev*/) {
            elevatorSetpoint = new TrapezoidProfile.State(elevatorCurrent, elevatorInputs.currentVelocityPctPerSec);
        }
        if (pivotSetpoint == null || Math.abs(pivotSetpoint.position - pivotCurrent) > pivotMaxDev) {
            pivotSetpoint = new TrapezoidProfile.State(pivotCurrent, pivotInputs.currentVelocityRotPerSec);
        }

        RobotView.setSuperstructureState(elevatorCurrent, pivotCurrent);
    }

    private void updateTrajectory() {
        double elevatorAvoid = Constants.kElevatorCollisionTolerance.get();
        double elevatorCollisionFrame = Constants.kElevatorFrameCollisionHeight.get();
        double elevatorCollisionStage2 = Constants.kElevatorStage2CollisionHeight.get();
        double pivotAvoid = Units.degreesToRotations(Constants.kOuttakePivotCollisionTolerance.get());
        double pivotCollisionFrame = Units.degreesToRotations(Constants.kOuttakePivotFrameCollisionAngle.get());
        double pivotCollisionStage2 = Units.degreesToRotations(Constants.kOuttakePivotStage2CollisionAngle.get());

        double elevatorTarget = targetState.getElevatorHeight();
        double pivotTarget = targetState.getPivotAngle();

        // Cases:
        // - current and target below -> both to target
        // - current below, target above -> pivot out, delay before moving elevator up
        // - current above, target below -> pivot at collision + avoidance, if pivot is out elevator to target
        // - current and target above -> if pivot is out both to target, keep pivot out

        if (elevatorInputs.currentHeightPct < 0.75 || elevatorTarget < 0.75) {

            // Collision with frame crossbar
            if (elevatorSetpoint.position < elevatorCollisionFrame) {
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

                    if (pivotSetpoint.position > pivotCollisionFrame) {
                        double pivotMoveTime = pivotProfile.timeLeftUntil(pivotCollisionFrame);
                        double elevatorMoveTime = elevatorProfile.timeLeftUntil(elevatorCollisionFrame);
                        if (elevatorMoveTime < pivotMoveTime) {
                            // Wait for pivot to move some
                            elevatorTarget = elevatorSetpoint.position;
                        }
                    }
                }
            } else {
                pivotTarget = Math.min(pivotTarget, pivotCollisionFrame - pivotAvoid);

                if (pivotSetpoint.position > pivotCollisionFrame) {
                    // Hold elevator still until pivot gets out of collision area
                    elevatorTarget = elevatorSetpoint.position;
                }
            }

            // Collision with stage 2 crossbar
            if (elevatorSetpoint.position < elevatorCollisionStage2) {
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

                    if (pivotSetpoint.position > pivotCollisionStage2) {
                        double pivotMoveTime = pivotProfile.timeLeftUntil(pivotCollisionStage2);
                        double elevatorMoveTime = elevatorProfile.timeLeftUntil(elevatorCollisionStage2);
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

                if (pivotSetpoint.position > pivotCollisionStage2) {
                    // Hold elevator still until pivot gets out of collision area
                    elevatorTarget = elevatorSetpoint.position;
                }
            }

            // Stupid simple collision check for upside down pivot
            // FIXME: Probably has broken edge cases
            if (pivotTarget < 0 && (elevatorSetpoint.position < 0.38 || elevatorTarget < 0.38))
                pivotTarget = 0;
            if ((pivotSetpoint.position < 0 || pivotTarget < 0) && elevatorTarget < 0.38)
                elevatorTarget = 0.38;
        }

        RobotView.setTargetSuperstructureState(elevatorTarget, pivotTarget);

        // Update motion profiles
        if (DriverStation.isEnabled()) {
            elevatorSetpoint = elevatorProfile.calculate(Constants.kPeriodicTime, elevatorSetpoint, new TrapezoidProfile.State(elevatorTarget, 0));
            pivotSetpoint = pivotProfile.calculate(Constants.kPeriodicTime, pivotSetpoint, new TrapezoidProfile.State(pivotTarget, 0));
        }

        RobotView.setSuperstructureSetpoint(elevatorSetpoint.position, pivotSetpoint.position);
    }

    @Override
    public void periodic() {
        // Set adjusts
        targetState.setElevatorAdjust(elevatorAdjust);
        targetState.setPivotAdjust(pivotAdjustDeg);

        targetState.setElevatorAim(elevatorAim);
        targetState.setPivotAim(pivotAim);

        elevatorIO.updateInputs(elevatorInputs);
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        Logger.processInputs("Outtake Pivot", pivotInputs);

        if (CALIBRATE_PIVOT.get()) {
            CALIBRATE_PIVOT.set(false);
            pivotIO.calibrateEncoder();
        }

        handleFeedbackSensors();
        updateTrajectory();

        if (targetState == State.CLIMB) {
            double hold = -Constants.kElevatorClimbHoldVolts.get();
            double pull = -Constants.kElevatorClimbPullVolts.get();
            double height = Constants.kElevatorHeightClimbEnd.get();

            double volts = hold;
            if (elevatorInputs.currentHeightPct > height) {
                volts += pull;
            }

            elevatorIO.setVoltage(volts);
        } else {
            double threshold = Constants.kElevatorNeutralThreshold.get();
            if (targetState.getElevatorHeight() < threshold && elevatorInputs.currentHeightPct < threshold) {
                elevatorIO.setVoltage(0.0);
            } else {
                double accel = (elevatorSetpoint.velocity - prevElevatorSetpointVelocity) / Constants.kPeriodicTime;
                elevatorIO.setTarget(elevatorSetpoint.position, elevatorSetpoint.velocity, accel);
            }
        }

        boolean hasCoral = outtakeSubsystem.hasPiece();
        pivotIO.setTarget(pivotSetpoint.position, pivotSetpoint.velocity, hasCoral);

        outtakeSubsystem.setReverseScore(pivotInputs.currentAngleRot < 0);

        prevElevatorSetpointVelocity = elevatorSetpoint.velocity;
    }

    // For auto
    public double calculateIndexerToL4TravelTime() {
        TrapezoidProfile.State elevatorInitialState = new TrapezoidProfile.State(State.RECEIVE_CORAL_FROM_INDEXER.getElevatorHeight(), 0);
        TrapezoidProfile.State elevatorFinalState = new TrapezoidProfile.State(State.SCORE_L4.getElevatorHeight(), 0);
        elevatorProfile.calculate(0, elevatorInitialState, elevatorFinalState);

        TrapezoidProfile.State pivotInitialState = new TrapezoidProfile.State(State.RECEIVE_CORAL_FROM_INDEXER.getPivotAngle(), 0);
        TrapezoidProfile.State pivotFinalState = new TrapezoidProfile.State(State.SCORE_L4.getPivotAngle(), 0);
        pivotProfile.calculate(0, pivotInitialState, pivotFinalState);

        double fullElevatorMovementTime = elevatorProfile.totalTime();
        double timeUntilElevatorCollision = elevatorProfile.timeLeftUntil(Constants.kElevatorFrameCollisionHeight.get());
        double timeUntilPivotOutOfCollision = pivotProfile.timeLeftUntil(Units.degreesToRotations(Constants.kOuttakePivotFrameCollisionAngle.get()));

        double waitTime = Math.max(0, timeUntilPivotOutOfCollision - timeUntilElevatorCollision);

        System.out.println("Theoretical travel time is " + (waitTime + fullElevatorMovementTime));
        return waitTime + fullElevatorMovementTime;
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

    public double getCurrentElevatorHeight() {
        return elevatorInputs.currentHeightPct;
    }
}
