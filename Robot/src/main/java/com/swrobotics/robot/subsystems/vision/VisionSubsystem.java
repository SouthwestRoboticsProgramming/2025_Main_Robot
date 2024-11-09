package com.swrobotics.robot.subsystems.vision;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import com.swrobotics.robot.subsystems.vision.limelight.LimelightIO;
import com.swrobotics.robot.subsystems.vision.limelight.NTLimelightIO;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class VisionSubsystem extends SubsystemBase {
    private static record PoseEstimate(Pose2d pose, double timestamp, int tagCount, double avgTagDist) {}

    private final SwerveDriveSubsystem drive;

    private final LimelightIO limelightIO;
    private final LimelightIO.Inputs limelightInputs;

    private double prevUpdateTimestamp;

    public VisionSubsystem(SwerveDriveSubsystem drive) {
        this.drive = drive;

        limelightIO = new NTLimelightIO("limelight");
        limelightInputs = new LimelightIO.Inputs();

        prevUpdateTimestamp = Double.NaN;
    }

    private boolean shouldUseMegaTag2(ChassisSpeeds currentSpeeds) {
        // MegaTag 1 is unreliable while moving, so use MegaTag 2 then.
        // However, still use MegaTag 1 when slow/stopped so the gyro can be
        // corrected by vision measurements.
        return Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                > Constants.kVisionMT2SpeedThreshold;
    }

    private void processEstimate(boolean useMegaTag2) {
        PoseEstimate est = decodeEstimate(useMegaTag2
                ? limelightInputs.megaTag2
                : limelightInputs.megaTag1);

        // Only process each frame once
        if (est == null || est.timestamp == prevUpdateTimestamp)
            return;
        prevUpdateTimestamp = est.timestamp;

        // If too far away, use MegaTag 2 estimate instead. MegaTag 1 estimate
        // is too unstable at far distances
        if (!useMegaTag2 && est.avgTagDist > Constants.kVisionMT1MaxDistance) {
            processEstimate(true);
            return;
        }

        // Standard deviation of vision estimates appears to be proportional to
        // the square of the distance to the tag. Also, estimates are more
        // stable with more tags.
        // TODO: How accurate is the divide by tag count? Do we even want it?
        double baseStdDev = MathUtil.square(est.avgTagDist) / est.tagCount;

        // Calculate standard deviations by linear regressions on distance^2
        double xyStdDev, thetaStdDev;
        if (useMegaTag2) {
            xyStdDev = baseStdDev * Constants.kVisionXYStdDevCoeffMT2;
            // Don't trust MT2 theta at all, it's just gyro theta but with latency
            thetaStdDev = 99999999999999.0;
        } else {
            xyStdDev = baseStdDev * Constants.kVisionXYStdDevCoeffMT1;
            thetaStdDev = baseStdDev * Constants.kVisionThetaStdDevCoeffMT1;
        }

        drive.addVisionMeasurement(
                est.pose,
                est.timestamp,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
        );
        FieldView.visionEstimates.setPose(est.pose);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = drive.getEstimatedPose();
        ChassisSpeeds currentSpeeds = drive.getRobotRelativeSpeeds();

        double yaw = currentPose.getRotation().getDegrees();
        double yawRate = Math.toDegrees(currentSpeeds.omegaRadiansPerSecond);
        limelightIO.updateRobotState(yaw, yawRate);

        limelightIO.updateInputs(limelightInputs);
        Logger.processInputs("Limelight", limelightInputs);

        processEstimate(shouldUseMegaTag2(currentSpeeds));
    }

    private PoseEstimate decodeEstimate(LimelightIO.EstimateInputs inputs) {
        long timestamp = inputs.timestamp;
        double[] data = inputs.data;

        // Incomplete data from Limelight
        if (data.length < 10)
            return null;

        Pose2d pose = new Pose2d(
                new Translation2d(data[0], data[1]),
                Rotation2d.fromDegrees(data[5]));
        double latency = data[6];
        int tagCount = (int) data[7];
        double avgTagDist = data[9];

        // No tags seen or Limelight returns origin for some reason
        if (tagCount <= 0 || (pose.getX() == 0 && pose.getY() == 0))
            return null;

        double correctedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        return new PoseEstimate(pose, correctedTimestamp, tagCount, avgTagDist);
    }
}
