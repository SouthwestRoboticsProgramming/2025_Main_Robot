package com.swrobotics.robot.control;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.RobotPhysics;
import com.swrobotics.robot.commands.CharacterizeWheelsCommand;
import com.swrobotics.robot.commands.DriveCommands;
import com.swrobotics.robot.commands.RumblePatternCommands;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;

import com.swrobotics.robot.subsystems.algae.AlgaeIntakeSubsystem;
import com.swrobotics.robot.subsystems.outtake.OuttakeSubsystem;
import com.swrobotics.robot.subsystems.superstructure.SuperstructureSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class ControlBoard extends SubsystemBase {
    /*
     * Control mapping:
     *
     * Driver:
     * Left stick: drive translation
     * Right stick X: drive rotation
     * 
     * Start: Reset gyro
     * Back: Reset gyro
     * 
     * B: Snap to coral station
     * A: Snap to reef point
     * 
     * Left trigger: Intake algae
     * Right trigger: Score algae
     * 
     * Operator:
     * A: L2
     * B: L3
     * Y: L4
     * X: L1
     * 
     * Up: Prep climb
     * Down: Climb
     * 
     * Right trigger: Score coral
     * Left bumper: Reverse coral
     * 
     * Inverted: 3
     * Not inverted: 2
     */

    private static final NTEntry<Boolean> CHARACTERISE_WHEEL_RADIUS = new NTBoolean("Drive/Characterize Wheel Radius", false);

    private final RobotContainer robot;
    public final XboxController driver;
    public final XboxController operator;

    private final DriveAccelFilter driveControlFilter;
    private final DriveAccelFilter2d driveTippingFilter;

    private int retriggerL4;

    public ControlBoard(RobotContainer robot) {
        this.robot = robot;

        // Passing deadband here means we don't have to deadband anywhere else
        driver = new XboxController(Constants.kDriverControllerPort, Constants.kDeadband);
        operator = new XboxController(Constants.kOperatorControllerPort, Constants.kDeadband);

        driveControlFilter = new DriveAccelFilter(Constants.kDriveControlMaxAccel);
        driveTippingFilter = new DriveAccelFilter2d(
                (dir) -> RobotPhysics.getMaxAccelerationWithoutTipping(
                        robot.drive.fieldToRobotRelative(dir),
                        robot.superstructure.getCurrentElevatorHeight()
                ) - Constants.kDriveTippingAccelTolerance
        );

        configureControls();
    }

    private void configureControls() {
        // Gyro reset buttons
        driver.start.onReleased(() -> robot.drive.resetRotation(new Rotation2d()));
        driver.back.onReleased(() -> robot.drive.resetRotation(new Rotation2d())); // Two buttons to reset gyro so the driver can't get confused

        robot.drive.setDefaultCommand(DriveCommands.driveFieldRelative(
                robot.drive,
                this::getDriveTranslation,
                this::getDriveRotation
        ));

        new Trigger(CHARACTERISE_WHEEL_RADIUS::get).whileTrue(new CharacterizeWheelsCommand(robot.drive));

        // Endgame Notice (controller rumble)
        new Trigger(
                () ->
                        DriverStation.isTeleopEnabled()
                                && DriverStation.getMatchTime() > 0
                                && DriverStation.getMatchTime() <= Constants.kEndgameAlertTime)
                .onTrue(RumblePatternCommands.endgameAlert(driver, 0.75)
                        .alongWith(RumblePatternCommands.endgameAlert(operator, 0.75)));

        new Trigger(
                 () ->
                        DriverStation.isTeleopEnabled()
                                && DriverStation.getMatchTime() > 0
                                && DriverStation.getMatchTime() <= Constants.kEndgameAlert2Time)
                .onTrue(RumblePatternCommands.endgameAlertFinalCountdown(driver, 0.75));

        driver.b.trigger()
                .whileTrue(DriveCommands.driveFieldRelativeSnapToAngle(
                        robot.drive,
                        robot.lights,
                        this::getDriveTranslation,
                        () -> FieldPositions.getClosestCoralStationAngle(robot.drive.getEstimatedPose())
                ));

        driver.a.trigger()
                .whileTrue(DriveCommands.snapToPose(
                        robot.drive,
                        robot.lights,
                        () -> FieldPositions.getClosestSnapTarget(robot.drive.getEstimatedPose())
                ));

        Trigger intakeAlgaeFloor = driver.leftTrigger.triggerOutside(0.25);
        Trigger scoreAlgaeFloor = driver.rightTrigger.triggerOutside(0.25);
        Trigger algaeIntakeOut = intakeAlgaeFloor.or(scoreAlgaeFloor);

        robot.algaeIntake.setDefaultCommand(
                robot.algaeIntake.commandSetState(AlgaeIntakeSubsystem.State.STOW));
        intakeAlgaeFloor
                .whileTrue(robot.algaeIntake.commandSetState(AlgaeIntakeSubsystem.State.INTAKE));
        scoreAlgaeFloor
                .whileTrue(robot.algaeIntake.commandSetState(AlgaeIntakeSubsystem.State.OUTTAKE));

//        robot.superstructure.setDefaultCommand(
//                robot.superstructure.commandSetState(SuperstructureSubsystem.State.RECEIVE_CORAL_FROM_INDEXER));
        robot.superstructure.setDefaultCommand(Commands.run(() -> {
            if (algaeIntakeOut.getAsBoolean() || robot.outtake.hasPiece()) {
                robot.superstructure.setTargetState(SuperstructureSubsystem.State.BOTTOM);
            } else {
                robot.superstructure.setTargetState(SuperstructureSubsystem.State.RECEIVE_CORAL_FROM_INDEXER);
            }
        }, robot.superstructure));
        operator.x.trigger()
                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L1));
        operator.a.trigger()
                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L2));
        operator.b.trigger()
                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L3));
//        operator.y.trigger()
//                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L4));

        Trigger elevatorL4 = operator.y.trigger()
                .and(() -> retriggerL4 == 0);
        
        double[] startTimestamp = {0};
        elevatorL4.whileTrue(
                new FunctionalCommand(
                        () -> startTimestamp[0] = Timer.getTimestamp(),
                        () -> robot.superstructure.setTargetState(SuperstructureSubsystem.State.SCORE_L4),
                        (cancelled) -> {
                            if (!cancelled) {
                                double endTimestamp = Timer.getTimestamp();
                                System.out.println("Travel time: " + (endTimestamp - startTimestamp[0]));
                            }
                        },
                        robot.superstructure::isInTolerance,
                        robot.superstructure
                ).andThen(robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L4))
        );

        // operator.dpad.up.trigger()
        //         .toggleOnTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.PREP_CLIMB));
        // operator.dpad.down.trigger().and(() -> robot.superstructure.getTargetState() == SuperstructureSubsystem.State.PREP_CLIMB)
        //         .onTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.CLIMB));

        Trigger removeLowAlgae = operator.dpad.down.trigger();
        Trigger removeHighAlgae = operator.dpad.up.trigger();
        Trigger removeAlgae = removeLowAlgae.or(removeHighAlgae);

        removeLowAlgae
                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.PICKUP_LOW_ALGAE));
        removeHighAlgae
                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.PICKUP_HIGH_ALGAE));


        robot.outtake.setDefaultCommand(Commands.run(() -> {
            if (removeAlgae.getAsBoolean()) {
                robot.outtake.setTargetState(OuttakeSubsystem.State.INTAKE_ALGAE);
            } else {
                robot.outtake.setTargetState(OuttakeSubsystem.State.INTAKE_CORAL);
            }
        }, robot.outtake));
        new Trigger(() -> operator.rightTrigger.isOutside(Constants.kTriggerThreshold))
                .whileTrue(Commands.either(
                        robot.outtake.commandSetState(OuttakeSubsystem.State.SCORE_L4),
                        robot.outtake.commandSetState(OuttakeSubsystem.State.SCORE_NOT_L4),
                        elevatorL4
                ));
//                .whileTrue(robot.outtake.commandSetState(OuttakeSubsystem.State.SCORE));
        operator.leftBumper.trigger()
               .onTrue(robot.outtake.commandSetState(OuttakeSubsystem.State.REVERSE)
                       .withTimeout(0.15));

        double step = 0.0005;
        driver.dpad.up.trigger()
                .onTrue(Commands.runOnce(() -> Constants.kElevatorIndexerHeight.set(Constants.kElevatorIndexerHeight.get() + step)));
        driver.dpad.down.trigger()
                .onTrue(Commands.runOnce(() -> Constants.kElevatorIndexerHeight.set(Constants.kElevatorIndexerHeight.get() - step)));

        // Everything past here is for testing and should eventually be removed

        // Test LEDs
//        driver.a.onPressed(LightCommands.blink(robot.lights, Color.kCyan));
//        driver.a.onHeld(LightCommands.blink(robot.lights, Color.kYellow));

//        driver.b.onPressed(RumblePatternCommands.endgameAlert(driver, 0.75));

//        driver.x.onPressed(Commands.defer(() -> AutoBuilderExt.pathfindToPose(
//                PathEnvironments.kFieldWithAutoGamePieces,
//                FieldView.pathfindingGoal.getPose(),
//                new PathConstraints(
//                        Constants.kAutoMaxDriveSpeed,
//                        Constants.kAutoMaxDriveAccel,
//                        Units.rotationsToRadians(Constants.kAutoMaxTurnSpeed),
//                        Units.rotationsToRadians(Constants.kAutoMaxTurnAccel)
//                )
//        ), Set.of(robot.drive)));
//        driver.y.onPressed(() -> FieldView.pathfindingGoal.setPose(robot.drive.getEstimatedPose()));

//        operator.start.trigger()
//                .whileTrue(DriveCommands.feedforwardCharacterization(robot.drive));
//        operator.start.trigger()
//                .whileTrue(Autonomous.goSideways5Meters(robot));
//        operator.start.trigger()
//                .whileTrue(Commands.run(() -> robot.drive.setControl(new SwerveRequest.RobotCentric()
//                                .withVelocityX(3)
//                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)),
//                        robot.drive));
//        operator.back.trigger()
//                .whileTrue(Commands.run(() -> robot.drive.setControl(new SwerveRequest.RobotCentric()
//                        .withVelocityX(1)
//                        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)),
//                        robot.drive));
    }

    private Translation2d getDesiredDriveTranslation() {
        double maxSpeed = Constants.kDriveMaxAchievableSpeed;

        Translation2d leftStick = driver.getLeftStick();

        // Apply an exponential curve to the driver's input. This allows the
        // driver to have slower, more precise movement in the center of the
        // stick, while still having high speed movement towards the edges.
        double rawMag = leftStick.getNorm();
        double powerMag = MathUtil.powerWithSign(rawMag, Constants.kDriveControlDrivePower);

        // Prevent division by zero, which would result in a target velocity of
        // (NaN, NaN), which motor controllers do not like
        if (rawMag == 0 || powerMag == 0)
            return new Translation2d(0, 0);

        double targetSpeed = powerMag * maxSpeed;
        double filteredSpeed = driveControlFilter.calculate(targetSpeed);
        return new Translation2d(-leftStick.getY(), -leftStick.getX())
                .div(rawMag) // Normalize translation
                .times(filteredSpeed) // Apply new speed
                .rotateBy(FieldInfo.getAllianceForwardAngle()); // Account for driver's perspective
    }

    /**
     * @return translation input for the drive base, in meters/sec
     */
    private Translation2d getDriveTranslation() {
        return driveTippingFilter.calculate(getDesiredDriveTranslation());
    }

    /**
     * @return radians per second input for the drive base
     */
    private double getDriveRotation() {
        double input = MathUtil.powerWithSign(-driver.rightStickX.get(), Constants.kDriveControlTurnPower);
        return Units.rotationsToRadians(input * Constants.kDriveControlMaxTurnSpeed);
    }

    private double getPivotAdjustDeg() {
        return -operator.leftStickX.get() * Constants.kPivotAdjustMax.get();
    }

    private double getElevatorAdjust() {
        return -operator.rightStickY.get() * Constants.kElevatorAdjustMax.get();
    }

    @Override
    public void periodic() {
        if (DriverStation.isTeleopEnabled()) {
                robot.superstructure.setElevatorAdjust(getElevatorAdjust());
                robot.superstructure.setPivotAdjust(getPivotAdjustDeg());

                if (retriggerL4 > 0)
                    retriggerL4--;
        } else {
            retriggerL4 = 3;
        }
    }
}
