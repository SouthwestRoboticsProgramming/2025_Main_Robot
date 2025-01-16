package com.swrobotics.robot.subsystems.vision;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import com.swrobotics.robot.subsystems.vision.limelight.LimelightCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public final class VisionSubsystem extends SubsystemBase {
    private final SwerveDriveSubsystem drive;
    private final List<LimelightCamera> cameras;

    public VisionSubsystem(SwerveDriveSubsystem drive) {
        this.drive = drive;

        cameras = List.of(
                new LimelightCamera(
                        "limelight",
                        Constants.kLimelightLocation,
                        Constants.kLimelightMT1MaxDistance)
                // Add more cameras here...
        );
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
        // TODO: See how this behaves on 2025 field
        boolean useMegaTag2 = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                > Constants.kVisionMT2SpeedThreshold;

        List<LimelightCamera.Update> updates = new ArrayList<>();
        for (LimelightCamera camera : cameras) {
            camera.getNewUpdates(updates, useMegaTag2);
        }

        for (LimelightCamera.Update update : updates) {
            drive.addVisionMeasurement(update.pose(), update.timestamp(), update.stdDevs());
        }
    }
}
