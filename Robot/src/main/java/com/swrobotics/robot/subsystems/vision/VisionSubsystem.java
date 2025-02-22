package com.swrobotics.robot.subsystems.vision;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import com.swrobotics.robot.subsystems.vision.limelight.LimelightCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public final class VisionSubsystem extends SubsystemBase {
    private final SwerveDriveSubsystem drive;
    private final List<LimelightCamera> cameras;

    private boolean ignoreUpdates;

    public VisionSubsystem(SwerveDriveSubsystem drive) {
        this.drive = drive;

        cameras = List.of(
                new LimelightCamera(
                        "limelight-ftleft",
                        Constants.kLimelightFrontLeftLocation,
                        Constants.kLimelightConfig),
                new LimelightCamera(
                        "limelight-ftright",
                        Constants.kLimelightFrontRightLocation,
                        Constants.kLimelightConfig),
                new LimelightCamera(
                        "limelight-back",
                        Constants.kLimelightBackLocation,
                        Constants.kLimelightConfig)
                // Add more cameras here...
        );

        ignoreUpdates = false;
        setDefaultCommand(Commands.run(() -> ignoreUpdates = false, this));
    }

    public Command commandIgnoreUpdates() {
        return Commands.run(() -> ignoreUpdates = true, this);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = drive.getEstimatedPose();
        ChassisSpeeds currentSpeeds = drive.getRobotRelativeSpeeds();

        double yaw = currentPose.getRotation().getDegrees();
        double yawRate = Math.toDegrees(currentSpeeds.omegaRadiansPerSecond);
        for (LimelightCamera camera : cameras) {
            camera.updateRobotState(yaw, yawRate);
        }

        // MegaTag 1 is unreliable while moving, so use MegaTag 2 then.
        // However, still use MegaTag 1 when slow/stopped so the gyro can be
        // corrected by vision measurements.
        boolean useMegaTag2 = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                > Constants.kVisionMT2SpeedThreshold;

        List<LimelightCamera.Update> updates = new ArrayList<>();
        for (LimelightCamera camera : cameras) {
            camera.getNewUpdates(updates, useMegaTag2);
        }

        if (ignoreUpdates)
            return;

        Pose2d[] poses = new Pose2d[updates.size()];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = updates.get(i).pose();
        }
        FieldView.visionEstimates.setPoses(poses);

        for (LimelightCamera.Update update : updates) {
            drive.addVisionMeasurement(update.pose(), update.timestamp(), update.stdDevs());
        }
    }
}
