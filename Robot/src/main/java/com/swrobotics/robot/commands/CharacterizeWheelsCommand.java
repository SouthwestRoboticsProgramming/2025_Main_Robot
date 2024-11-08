package com.swrobotics.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Command to measure the effective wheel radius on the field carpet.
 */
public final class CharacterizeWheelsCommand extends Command {
    private static final double ROTATION_SPEED = Math.PI; // rad/s

    private final SwerveDriveSubsystem drive;
    private SwerveModulePosition[] startingPositions;
    private double lastGyroRad;
    private double gyroAccumulatorRad;

    public CharacterizeWheelsCommand(SwerveDriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        startingPositions = drive.getModulePositions();
        gyroAccumulatorRad = 0;
        lastGyroRad = drive.getRawGyroRotation().getRadians();
    }

    @Override
    public void execute() {
        drive.setControl(new SwerveRequest.RobotCentric()
            .withRotationalRate(ROTATION_SPEED));

        double currentGyroRad = drive.getRawGyroRotation().getRadians();
        gyroAccumulatorRad += MathUtil.angleModulus(currentGyroRad - lastGyroRad);
        lastGyroRad = currentGyroRad;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveModulePosition[] endingPositions = drive.getModulePositions();
        double averageWheelDisplacement = 0;

        for (int i = 0; i < 4; i++) {
            averageWheelDisplacement += Math.abs(endingPositions[i].distanceMeters - startingPositions[i].distanceMeters);
        }
        averageWheelDisplacement /= 4;

        double correctionScalar = (gyroAccumulatorRad * Constants.kDriveRadius) / averageWheelDisplacement;
        double correctedRadius = Constants.kModuleConstantsFactory.WheelRadius * correctionScalar;

        // Report result as warning so it shows up in the driver station
        DriverStation.reportWarning("Corrected wheel radius: " + correctedRadius, false);

        // Also log to AdvantageKit in case we miss the message
        Logger.recordOutput("Drive/Corrected wheel radius", correctedRadius);
    }
}
