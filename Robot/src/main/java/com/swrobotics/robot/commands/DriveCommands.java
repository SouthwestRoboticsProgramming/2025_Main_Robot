package com.swrobotics.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.lib.utils.PolynomialRegression;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.lights.LightsSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public final class DriveCommands {
    public static Command driveRobotRelative(
            SwerveDriveSubsystem drive,
            Supplier<Translation2d> translationSupplier, // m/s
            Supplier<Double> rotationSupplier // rad/s
    ) {
        return Commands.run(() -> {
            Translation2d tx = translationSupplier.get();
            double rot = rotationSupplier.get();

            drive.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(tx.getX())
                    .withVelocityY(tx.getY())
                    .withRotationalRate(rot));
        }, drive);
    }

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
            LightsSubsystem lights,
            Supplier<Translation2d> translationSupplier,
            Supplier<Rotation2d> targetRotationSupplier) {
        PIDController turnPid = new PIDController(0, 0, 0);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.startRun(() -> {
            turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());
            turnPid.reset();
        }, () -> {
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
                .raceWith(LightCommands.showSnappingToAngle(lights));
    }

    public static Command driveFieldRelativeSnapToAngleAndY(
            SwerveDriveSubsystem drive,
            LightsSubsystem lights,
            Supplier<Double> driveXSupplier,
            Supplier<Rotation2d> targetAngleSupplier,
            Supplier<Double> targetYSupplier) {
        PIDController driveYPid = new PIDController(0, 0, 0);
        PIDController turnPid = new PIDController(0, 0, 0);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.startRun(() -> {
                    // Update PIDs in case we tuned them since last time
                    driveYPid.setPID(Constants.kSnapDriveKp.get(), 0, Constants.kSnapDriveKd.get());
                    turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());

                    driveYPid.reset();
                    turnPid.reset();
                }, () -> {
                    Pose2d currentPose = drive.getEstimatedPose();
                    Rotation2d targetAngle = targetAngleSupplier.get();
                    double targetY = targetYSupplier.get();

                    double yError = Math.abs(currentPose.getY() - targetY);
                    double thetaError = MathUtil.absDiffRad(currentPose.getRotation().getRadians(), targetAngle.getRadians());

                    double yOutput = 0;
                    if (yError > Constants.kSnapXYDeadzone.get()) {
                        yOutput = driveYPid.calculate(currentPose.getY(), targetY);
                    }
                    double rotOutput = 0;
                    if (thetaError > Math.toRadians(Constants.kSnapThetaDeadzone.get())) {
                        rotOutput = turnPid.calculate(
                                MathUtil.wrap(currentPose.getRotation().getRadians(), -Math.PI, Math.PI),
                                MathUtil.wrap(targetAngle.getRadians(), -Math.PI, Math.PI)
                        );
                    }

                    double maxDriveSpeed = Constants.kSnapMaxSpeed.get();
                    if (Math.abs(yOutput) > maxDriveSpeed) {
                        yOutput = Math.copySign(maxDriveSpeed, yOutput);
                    }

                    double maxTurnSpeed = Units.rotationsToRadians(Constants.kSnapMaxTurnSpeed.get());
                    rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);

                    drive.setControl(new SwerveRequest.FieldCentric()
                            .withVelocityX(driveXSupplier.get())
                            .withVelocityY(yOutput)
                            .withRotationalRate(rotOutput));
                }, drive)
                .raceWith(LightCommands.showSnappingToAngleAndY(lights));
    }

    public static Command snapToPose(SwerveDriveSubsystem drive, LightsSubsystem lights, Supplier<Pose2d> targetPoseSupplier) {
        PIDController driveXPid = new PIDController(0, 0, 0);
        PIDController driveYPid = new PIDController(0, 0, 0);
        PIDController turnPid = new PIDController(0, 0, 0);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.startRun(() -> {
            // Update PIDs in case we tuned them since last time
            driveXPid.setPID(Constants.kSnapDriveKp.get(), 0, Constants.kSnapDriveKd.get());
            driveYPid.setPID(Constants.kSnapDriveKp.get(), 0, Constants.kSnapDriveKd.get());
            turnPid.setPID(Constants.kSnapTurnKp.get(), 0, Constants.kSnapTurnKd.get());

            driveXPid.reset();
            driveYPid.reset();
            turnPid.reset();
        }, () -> {
            Pose2d currentPose = drive.getEstimatedPose();
            Pose2d targetPose = targetPoseSupplier.get();

            double xyError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double thetaError = MathUtil.absDiffRad(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

            double xOutput = 0, yOutput = 0;
            if (xyError > Constants.kSnapXYDeadzone.get()) {
                xOutput = driveXPid.calculate(currentPose.getX(), targetPose.getX());
                yOutput = driveYPid.calculate(currentPose.getY(), targetPose.getY());
            }
            double rotOutput = 0;
            if (thetaError > Math.toRadians(Constants.kSnapThetaDeadzone.get())) {
                rotOutput = turnPid.calculate(
                        MathUtil.wrap(currentPose.getRotation().getRadians(), -Math.PI, Math.PI),
                        MathUtil.wrap(targetPose.getRotation().getRadians(), -Math.PI, Math.PI)
                );
            }

            double maxDriveSpeed = Constants.kSnapMaxSpeed.get();
            double driveSpeed = Math.hypot(xOutput, yOutput);
            if (driveSpeed > maxDriveSpeed) {
                double scale = maxDriveSpeed / driveSpeed;
                xOutput *= scale;
                yOutput *= scale;
            }

            double maxTurnSpeed = Units.rotationsToRadians(Constants.kSnapMaxTurnSpeed.get());
            rotOutput = MathUtil.clamp(rotOutput, -maxTurnSpeed, maxTurnSpeed);

            drive.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(xOutput)
                    .withVelocityY(yOutput)
                    .withRotationalRate(rotOutput));
        }, drive)
                .raceWith(LightCommands.showSnappingToPose(lights));
    }

    public static Command snapToPoseUntilInTolerance(
            SwerveDriveSubsystem drive,
            LightsSubsystem lights,
            Supplier<Pose2d> poseSupplier,
            Supplier<Double> toleranceSupplier) {
        return snapToPose(drive, lights, poseSupplier)
                .until(() -> drive.isCloseTo(
                        poseSupplier.get().getTranslation(),
                        toleranceSupplier.get()
                ));
    }

    public static Command feedforwardCharacterization(SwerveDriveSubsystem drive) {
        List<Double> velocitySamples = new ArrayList<>();
        List<Double> voltageSamples = new ArrayList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient forward
                Commands.run(() -> drive.setControl(new SwerveRequest.SysIdSwerveTranslation()
                        .withVolts(0)), drive)
                        .withTimeout(1),

                Commands.runOnce(timer::restart),

                Commands.run(() -> {
                    double voltage = timer.get() * 0.1;

                    drive.setControl(new SwerveRequest.SysIdSwerveTranslation()
                            .withVolts(voltage));

                    // Convert units to rotor rotations, which is what CTRE wants
                    double metersPerSec = drive.getFFCharacterizationVelocity();
                    double wheelRadPerSec = metersPerSec / Constants.kModuleConstantsFactory.WheelRadius;
                    double wheelRotPerSec = Units.radiansToRotations(wheelRadPerSec);
                    double rotorRotPerSec = wheelRotPerSec * Constants.kModuleConstantsFactory.DriveMotorGearRatio;

                    velocitySamples.add(rotorRotPerSec);
                    voltageSamples.add(voltage);
                }, drive).finallyDo(() -> {
                    if (velocitySamples.size() < 2 || voltageSamples.size() < 2)
                        return;

                    double[] velocityArray = new double[velocitySamples.size()];
                    for (int i = 0; i < velocityArray.length; i++) {
                        velocityArray[i] = velocitySamples.get(i);
                    }

                    double[] voltageArray = new double[voltageSamples.size()];
                    for (int i = 0; i < voltageArray.length; i++) {
                        voltageArray[i] = voltageSamples.get(i);
                    }

                    PolynomialRegression regression = new PolynomialRegression(velocityArray, voltageArray, 1);
                    double kS = regression.beta(0); // y-intercept
                    double kV = regression.beta(1); // slope
                    double R2 = regression.R2();

                    System.out.println("FF Characterization Results:");
                    System.out.printf("\tR2=%.5f%n", R2);
                    System.out.printf("\tkS=%.5f%n", kS);
                    System.out.printf("\tkV=%.5f%n", kV);

                    Logger.recordOutput("Drive/FF Characterization/kS", kS);
                    Logger.recordOutput("Drive/FF Characterization/kV", kV);
                    Logger.recordOutput("Drive/FF Characterization/R2", R2);
                })
        );
    }
}
