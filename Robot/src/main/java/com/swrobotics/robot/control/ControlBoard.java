package com.swrobotics.robot.control;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.commands.CharacterizeWheelsCommand;
import com.swrobotics.robot.commands.DriveCommands;
import com.swrobotics.robot.commands.RumblePatternCommands;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;

import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.superstructure.SuperstructureSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Collections;

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
     * 
     * Operator:
     * A: L2
     * B: L3
     * Y: L4
     * X: L1
     * 
     * Right trigger: Score coral
     */

    private static final NTEntry<Boolean> CHARACTERISE_WHEEL_RADIUS = new NTBoolean("Drive/Characterize Wheel Radius", false);

    private final RobotContainer robot;
    public final XboxController driver;
    public final XboxController operator;

    private final DriveAccelFilter driveFilter;

    public ControlBoard(RobotContainer robot) {
        this.robot = robot;

        // Passing deadband here means we don't have to deadband anywhere else
        driver = new XboxController(Constants.kDriverControllerPort, Constants.kDeadband);
        operator = new XboxController(Constants.kOperatorControllerPort, Constants.kDeadband);

        driveFilter = new DriveAccelFilter(Constants.kDriveControlMaxAccel);

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

        driver.b.trigger()
                .whileTrue(DriveCommands.driveFieldRelativeSnapToAngle(
                        robot.drive,
                        this::getDriveTranslation,
                        () -> FieldPositions.getClosestCoralStationAngle(robot.drive.getEstimatedPose())
                ));

        driver.a.trigger()
                .whileTrue(DriveCommands.snapToPose(
                        robot.drive,
                        () -> FieldPositions.getClosestSnapTarget(robot.drive.getEstimatedPose())
                ));

        robot.superstructure.setDefaultCommand(
                robot.superstructure.commandSetState(SuperstructureSubsystem.State.RECEIVE_CORAL_FROM_INDEXER));
        operator.x.trigger()
                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L1));
        operator.a.trigger()
                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L2));
        operator.b.trigger()
                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L3));
        operator.y.trigger()
                .whileTrue(robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L4));

        // Everything past here is for testing and should eventually be removed

        // Test LEDs
//        driver.a.onPressed(LightCommands.blink(robot.lights, Color.kCyan));
//        driver.a.onHeld(LightCommands.blink(robot.lights, Color.kYellow));

//        driver.b.onPressed(RumblePatternCommands.endgameAlert(driver, 0.75));

//        driver.x.onPressed(Commands.defer(robot.pathfindingTest::getFollowCommand, Collections.emptySet()));
//        driver.y.onPressed(() -> FieldView.pathfindingGoal.setPose(robot.drive.getEstimatedPose()));
    }

    /**
     * @return translation input for the drive base, in meters/sec
     */
    private Translation2d getDriveTranslation() {
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
        double filteredSpeed = driveFilter.calculate(targetSpeed);

        return new Translation2d(-leftStick.getY(), -leftStick.getX())
                .div(rawMag) // Normalize translation
                .times(filteredSpeed) // Apply new speed
                .rotateBy(FieldInfo.getAllianceForwardAngle()); // Account for driver's perspective
    }

    /**
     * @return radians per second input for the drive base
     */
    private double getDriveRotation() {
        double input = MathUtil.powerWithSign(-driver.rightStickX.get(), Constants.kDriveControlTurnPower);
        return Units.rotationsToRadians(input * Constants.kDriveControlMaxTurnSpeed);
    }
}
