package com.swrobotics.robot.subsystems.vision.tagtracker;

import com.swrobotics.lib.utils.DoubleOutput;
import com.swrobotics.lib.utils.Transformation3d;
import com.swrobotics.robot.subsystems.vision.AprilTagEnvironment;
import com.swrobotics.robot.subsystems.vision.RawAprilTagSource;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import java.io.ByteArrayInputStream;
import java.io.DataInput;
import java.io.DataInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * AprilTag vision source from TagTracker.
 */
public final class TagTrackerSource extends RawAprilTagSource {
    private static final String TABLE = "TagTracker";

    /**
     * Publishes the AprilTag environment for TagTracker to use. This must be
     * done before TagTracker is able to make any estimates!
     *
     * @param environment environment to publish
     */
    public static void publishTagEnvironment(AprilTagEnvironment environment) {
        Map<Integer, Pose3d> poses = environment.getPoseMap();

        DoubleOutput data = new DoubleOutput(poses.size() * 8 + 1);

        data.add(environment.getTagSize());
        for (Map.Entry<Integer, Pose3d> entry : poses.entrySet()) {
            data.add(entry.getKey());

            Pose3d pose = entry.getValue();
            Translation3d tx = pose.getTranslation();
            Quaternion q = pose.getRotation().getQuaternion();

            data.add(tx.getX());
            data.add(tx.getY());
            data.add(tx.getZ());
            data.add(q.getW());
            data.add(q.getX());
            data.add(q.getY());
            data.add(q.getZ());
        }

        NetworkTableInstance.getDefault()
                .getTable(TABLE)
                .getEntry("Environment")
                .setDoubleArray(data.toArray());
    }

    private final String name;
    private final TagTrackerCameraIO io;
    private final TagTrackerCameraIO.Inputs inputs;

    public TagTrackerSource(
            String name,
            AprilTagEnvironment environment,
            Transformation3d cameraToRobotTransform,
            FilterParameters filterParams,
            TagTrackerCaptureProperties captureProps) {
        super(environment, filterParams, cameraToRobotTransform);
        this.name = name;

        NetworkTable table = NetworkTableInstance.getDefault()
                .getTable(TABLE + "/Cameras/" + name);
        io = new NTCameraIO(table);
        inputs = new TagTrackerCameraIO.Inputs();

        io.setCaptureProperties(captureProps);
    }

    private PoseEstimate decodePoseEstimate(DataInput in) throws IOException {
        double error = in.readFloat();

        double tx = in.readDouble();
        double ty = in.readDouble();
        double tz = in.readDouble();
        Translation3d translation = new Translation3d(tx, ty, tz);

        double qw = in.readDouble();
        double qx = in.readDouble();
        double qy = in.readDouble();
        double qz = in.readDouble();
        Rotation3d rotation = new Rotation3d(new Quaternion(qw, qx, qy, qz));

        Pose3d cameraPose = new Pose3d(translation, rotation);
        return new PoseEstimate(cameraPose, error);
    }

    private EstimateInput decodeEstimate(long ntTimestamp, byte[] data) throws IOException {
        double timestamp = ntTimestamp / 1_000_000.0;
        DataInputStream in = new DataInputStream(new ByteArrayInputStream(data));

        boolean hasEstimate = in.readBoolean();
        if (!hasEstimate)
            return null;

        PoseEstimate poseA = decodePoseEstimate(in);
        PoseEstimate poseB = null;
        int tagCount = 1;
        if (in.readBoolean()) {
            poseB = decodePoseEstimate(in);
            tagCount = in.readUnsignedByte();
        }

        int[] visibleTagIds = new int[tagCount];
        Translation2d[][] visibleTagCorners = new Translation2d[tagCount][4];
        for (int i = 0; i < tagCount; i++) {
            visibleTagIds[i] = in.readUnsignedByte();

            Translation2d[] cornerSet = visibleTagCorners[i];
            for (int j = 0; j < 4; j++) {
                double x = in.readUnsignedShort();
                double y = in.readUnsignedShort();
                cornerSet[j] = new Translation2d(x, y);
            }
        }

        double timeOffset = in.readDouble();
        return new EstimateInput(
                timestamp - timeOffset,
                poseA,
                poseB,
                visibleTagIds,
                visibleTagCorners
        );
    }

    @Override
    public List<EstimateInput> getNewEstimates() {
        io.updateInputs(inputs);
        Logger.processInputs("TagTracker/" + name, inputs);

        List<EstimateInput> estimates = new ArrayList<>();

        for (int i = 0; i < inputs.timestamps.length; i++) {
            long timestamp = inputs.timestamps[i];
            byte[] data = inputs.framePackedData[i];

            try {
                EstimateInput estimate = decodeEstimate(timestamp, data);
                if (estimate != null)
                    estimates.add(estimate);
            } catch (IndexOutOfBoundsException e) {
                DriverStation.reportWarning("TagTracker sent incomplete data", false);
            } catch (IOException impossible) {
                // Won't happen, ByteArrayInputStream never throws
            }
        }

        return estimates;
    }
}
