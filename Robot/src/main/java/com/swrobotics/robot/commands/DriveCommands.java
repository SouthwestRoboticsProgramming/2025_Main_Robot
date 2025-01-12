package com.swrobotics.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

public final class DriveCommands {
    public static Command driveFieldRelative(
            SwerveDriveSubsystem drive,
            Supplier<Translation2d> translationSupplier, // m/s
            Supplier<Double> rotationSupplier // rad/s
    ) {
        return Commands.run(() -> {
            Translation2d tx = translationSupplier.get();
            double rot = rotationSupplier.get();

            drive.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(tx.getX())
                    .withVelocityY(tx.getY())
                    .withRotationalRate(rot));
        }, drive);
    }

    public static Command driveFieldRelativeSnapToAngle(
            SwerveDriveSubsystem drive,
            Supplier<Translation2d> translationSupplier,
            Supplier<Rotation2d> targetRotationSupplier) {
        PIDController turnPid = new PIDController(0, 0, 0);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.sequence(
                Commands.runOnce(() -> {
                    turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());
                    turnPid.reset();
                }),
                Commands.run(() -> {
                    Translation2d tx = translationSupplier.get();

                    Rotation2d currentRot = drive.getEstimatedPose().getRotation();
                    Rotation2d targetRot = targetRotationSupplier.get();
                    double rotOutput = turnPid.calculate(
                            MathUtil.wrap(currentRot.getRadians(), -Math.PI, Math.PI),
                            MathUtil.wrap(targetRot.getRadians(), -Math.PI, Math.PI)
                    );

                    double maxTurnSpeed = Units.rotationsToRadians(Constants.kSnapMaxTurnSpeed.get());
                    rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);

                    drive.setControl(new SwerveRequest.FieldCentric()
                            .withVelocityX(tx.getX())
                            .withVelocityY(tx.getY())
                            .withRotationalRate(rotOutput));
                }, drive)
        );
    }

    public static Command snapToPose(SwerveDriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
        PIDController driveXPid = new PIDController(0, 0, 0);
        PIDController driveYPid = new PIDController(0, 0, 0);
        PIDController turnPid = new PIDController(0, 0, 0);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.sequence(
                Commands.runOnce(() -> {
                    // Update PIDs in case we tuned them since last time
                    driveXPid.setPID(Constants.kSnapDriveKp.get(), 0, Constants.kSnapDriveKd.get());
                    driveYPid.setPID(Constants.kSnapDriveKp.get(), 0, Constants.kSnapDriveKd.get());
                    turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());

                    driveXPid.reset();
                    driveYPid.reset();
                    turnPid.reset();
                }),
                Commands.run(() -> {
                    Pose2d currentPose = drive.getEstimatedPose();
                    Pose2d targetPose = targetPoseSupplier.get();

                    double xOutput = driveXPid.calculate(currentPose.getX(), targetPose.getX());
                    double yOutput = driveYPid.calculate(currentPose.getY(), targetPose.getY());
                    double rotOutput = turnPid.calculate(
                            MathUtil.wrap(currentPose.getRotation().getRadians(), -Math.PI, Math.PI),
                            MathUtil.wrap(targetPose.getRotation().getRadians(), -Math.PI, Math.PI)
                    );

                    double maxDriveSpeed = Constants.kSnapMaxSpeed.get();
                    double driveSpeed = Math.hypot(xOutput, yOutput);
                    if (driveSpeed > maxDriveSpeed) {
                        double scale = maxDriveSpeed / driveSpeed;
                        xOutput *= scale;
                        yOutput *= scale;
                    }

                    double maxTurnSpeed = Constants.kSnapMaxTurnSpeed.get();
                    rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);

                    drive.setControl(new SwerveRequest.FieldCentric()
                            .withVelocityX(xOutput)
                            .withVelocityY(yOutput)
                            .withRotationalRate(rotOutput));
                }, drive)
        );
    }
}
