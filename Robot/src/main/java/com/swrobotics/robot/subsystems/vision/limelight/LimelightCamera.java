package com.swrobotics.robot.subsystems.vision.limelight;

import com.swrobotics.lib.utils.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public final class LimelightCamera {
    // forward, right, up in meters; pitch, yaw, roll in degrees CCW
    public static record MountingLocation(
            double forward, double right, double up,
            double roll, double pitch, double yaw) {}

    public static record Config(
            double mt1MaxDistance,
            double xyStdDevCoeffMT1,
            double thetaStdDevCoeffMT1,
            double xyStdDevCoeffMT2) {}

    public static record Update(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    private static record PoseEstimate(Pose2d pose, double timestamp, int tagCount, double avgTagDist) {
        public void log(String prefix) {
            Logger.recordOutput(prefix + "/Pose", pose);
            Logger.recordOutput(prefix + "/Timestamp", timestamp);
            Logger.recordOutput(prefix + "/Tag Count", tagCount);
            Logger.recordOutput(prefix + "/Average Tag Dist", avgTagDist);
        }
    }

    private final String name;
    private final Config config;

    private final LimelightIO io;
    private final LimelightIO.Inputs inputs;

    private double prevUpdateTimestamp;

    public LimelightCamera(String name, MountingLocation location, Config config) {
        this.name = name;
        this.config = config;

        io = new NTLimelightIO(name, location);
        inputs = new LimelightIO.Inputs();

        prevUpdateTimestamp = Double.NaN;
    }

    public void updateRobotState(double yaw, double yawRate) {
        io.updateRobotState(yaw, yawRate);
    }

    public void getNewUpdates(List<Update> updatesOut, boolean useMegaTag2) {
        io.updateInputs(inputs);
        Logger.processInputs("Limelight/" + name, inputs);

        PoseEstimate mt1 = decodeEstimate(inputs.megaTag1);
        PoseEstimate mt2 = decodeEstimate(inputs.megaTag2);
        if (mt1 != null)
            mt1.log("Limelight/" + name + "/MegaTag 1");
        if (mt2 != null)
            mt2.log("Limelight/" + name + "/MegaTag 2");

        processEstimate(updatesOut, mt1, mt2, useMegaTag2);
    }

    private void processEstimate(List<Update> updatesOut, PoseEstimate mt1, PoseEstimate mt2, boolean useMegaTag2) {
        PoseEstimate est = useMegaTag2 ? mt2 : mt1;

        // Only process each frame once
        if (est == null || est.timestamp == prevUpdateTimestamp)
            return;

        // If too far away, use MegaTag 2 estimate instead. MegaTag 1 estimate
        // is too unstable at far distances
        if (!useMegaTag2 && est.avgTagDist > config.mt1MaxDistance && DriverStation.isEnabled()) {
            processEstimate(updatesOut, mt1, mt2, true);
            return;
        }

        if (DriverStation.isDisabled())
            useMegaTag2 = false;

        prevUpdateTimestamp = est.timestamp;

        // Standard deviation of vision estimates appears to be proportional to
        // the square of the distance to the tag. Also, estimates are more
        // stable with more tags.
        // TODO: How accurate is the divide by tag count? Do we even want it?
        double baseStdDev = MathUtil.square(est.avgTagDist) / est.tagCount;

        // Calculate standard deviations by linear regressions on distance^2
        double xyStdDev, thetaStdDev;
        if (useMegaTag2) {
            xyStdDev = baseStdDev * config.xyStdDevCoeffMT2;
            // Don't trust MT2 theta at all, it's just gyro theta but with latency
            thetaStdDev = 99999999999999.0;
        } else {
            xyStdDev = baseStdDev * config.xyStdDevCoeffMT1;
            thetaStdDev = baseStdDev * config.thetaStdDevCoeffMT1;
        }

        updatesOut.add(new Update(
                est.pose,
                est.timestamp,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
        ));
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
