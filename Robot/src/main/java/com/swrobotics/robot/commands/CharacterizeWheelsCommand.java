package com.swrobotics.robot.commands;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        startingPositions = drive.getCurrentModulePositions();
        gyroAccumulatorRad = 0;
        lastGyroRad = drive.getRawGyroRotation().getRadians();
    }

    @Override
    public void execute() {
        drive.turn(new SwerveDriveSubsystem.TurnRequest(
                SwerveDriveSubsystem.Priority.AUTO,
                new Rotation2d(ROTATION_SPEED)
        ));

        double currentGyroRad = drive.getRawGyroRotation().getRadians();
        gyroAccumulatorRad += MathUtil.angleModulus(currentGyroRad - lastGyroRad);
        lastGyroRad = currentGyroRad;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveModulePosition[] endingPositions = drive.getCurrentModulePositions();
        double averageWheelDisplacement = 0;

        for (int i = 0; i < 4; i++) {
            averageWheelDisplacement += Math.abs(endingPositions[i].distanceMeters - startingPositions[i].distanceMeters);
        }
        averageWheelDisplacement /= 4;

        double correctionScalar = (gyroAccumulatorRad * Constants.kDriveRadius) / averageWheelDisplacement;
        double correctedRadius = Constants.kSwerveConstantsFactory.WheelRadius * correctionScalar;

        // Report the result in various places
        DriverStation.reportWarning("Corrected wheel radius: " + correctedRadius, false);
        Logger.recordOutput("Drive/Corrected wheel radius", correctedRadius);
        SmartDashboard.putNumber("Characterized wheel radius", correctedRadius);
    }
}
