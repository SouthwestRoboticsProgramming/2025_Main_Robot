package com.swrobotics.robot.subsystems.vision.limelight;

import com.swrobotics.lib.utils.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public final class LimelightCamera {
    // forward, right, up in meters; pitch, yaw, roll in degrees CCW
    public record MountingLocation(
            double forward, double right, double up,
            double roll, double pitch, double yaw) {}

    public record Update(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    private record PoseEstimate(Pose2d pose, double timestamp, double avgTagDist) {}

    private final String name;
    private final double mt1MaxDistance;

    private final LimelightIO io;
    private final LimelightIO.Inputs inputs;

    private double prevUpdateTimestamp;

    public LimelightCamera(String name, MountingLocation location, double mt1MaxDistance) {
        this.name = name;
        this.mt1MaxDistance = mt1MaxDistance;

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

        processEstimate(updatesOut, useMegaTag2);
    }

    private void processEstimate(List<Update> updatesOut, boolean useMegaTag2) {
        PoseEstimate est = decodeEstimate(useMegaTag2 ? inputs.megaTag2 : inputs.megaTag1);

        // Only process each frame once
        if (est == null || est.timestamp == prevUpdateTimestamp)
            return;

        // If too far away, use MegaTag 2 estimate instead. MegaTag 1 estimate
        // is too unstable at far distances
        if (!useMegaTag2 && est.avgTagDist > mt1MaxDistance) {
            processEstimate(updatesOut, true);
            return;
        }

        prevUpdateTimestamp = est.timestamp;

        if (inputs.stdDevsData.length < 12)
            return; // Incomplete data

        int stdDevIdx = useMegaTag2 ? 6 : 0;
        double xStdDev = inputs.stdDevsData[stdDevIdx];
        double yStdDev = inputs.stdDevsData[stdDevIdx + 1];
        double thetaStdDev = inputs.stdDevsData[stdDevIdx + 5];

        if (useMegaTag2) {
            // Don't trust MT2 theta at all, it's just gyro theta but with latency
            thetaStdDev = 999999999.0;
        }

        updatesOut.add(new Update(
                est.pose,
                est.timestamp,
                VecBuilder.fill(xStdDev, yStdDev, thetaStdDev)
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

        return new PoseEstimate(pose, correctedTimestamp, avgTagDist);
    }
}
