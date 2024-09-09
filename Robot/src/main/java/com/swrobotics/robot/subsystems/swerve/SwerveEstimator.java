package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.lib.utils.Transformation3d;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.vision.AprilTagEnvironment;
import com.swrobotics.robot.subsystems.vision.RawAprilTagSource;
import com.swrobotics.robot.subsystems.vision.VisionSource;
import com.swrobotics.robot.subsystems.vision.VisionUpdate;
import com.swrobotics.robot.subsystems.vision.tagtracker.TagTrackerSource;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Uses vision estimates and odometry measurements to estimate the robot's pose
 * on the field.
 */
public final class SwerveEstimator {
    private final List<VisionSource> visionSources;

    private Pose2d basePose, latestPose;
    private final TreeMap<Double, PoseUpdate> updates = new TreeMap<>();
    private final Matrix<N3, N1> q, qWithMoreTrustAngle;

    private boolean ignoreVision;

    public SwerveEstimator() {
        // Load AprilTag environment from JSON file
        AprilTagEnvironment environment;
        try {
            environment = AprilTagEnvironment.load(Constants.kAprilTagJson);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load AprilTag environment!", e);
        }
        TagTrackerSource.publishTagEnvironment(environment);

        // Show AprilTag poses on the field view
        List<Pose2d> tagPoses = new ArrayList<>();
        for (Pose3d tagPose3d : environment.getAllPoses()) {
            tagPoses.add(tagPose3d.toPose2d());
        }
        FieldView.aprilTagPoses.setPoses(tagPoses);

        double halfFrameL = 0.77 / 2;
        double halfFrameW = 0.695 / 2;

        // TODO: Come up with cleaner way of specifying camera position
        Transformation3d frontTransform = (camPose) -> camPose
                // Compensate for mounting angle
                .transformBy(new Transform3d(new Translation3d(), new Rotation3d(Math.PI, 0, 0)))
                .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, Math.toRadians(90 - 67), 0)))
                // Compensate for mounting position
                .transformBy(new Transform3d(new Translation3d(halfFrameL - 0.046, -halfFrameW + 0.12, 0.19).unaryMinus(), new Rotation3d()));

        Transformation3d zoomTransform = (camPose) -> camPose
                // Compensate for mounting angle
                .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, Math.toRadians(16.88), 0)))
                // Compensate for mounting position
                .transformBy(new Transform3d(new Translation3d(halfFrameL - 0.16, halfFrameW - 0.133, 0.39).unaryMinus(), new Rotation3d()));

        visionSources = new ArrayList<>();
        visionSources.add(new TagTrackerSource(
                "front",
                environment,
                frontTransform,
                // Only trust front camera when close, its far measurements are
                // very noisy and inaccurate
                new RawAprilTagSource.FilterParameters(Constants.kTagTrackerFilterParams)
                        .setMaxTrustDistance(5),
                Constants.kTagTrackerCaptureProps
        ));
        visionSources.add(new TagTrackerSource(
                "zoom",
                environment,
                zoomTransform,
                Constants.kTagTrackerFilterParams,
                Constants.kTagTrackerCaptureProps
        ));

        latestPose = new Pose2d();
        basePose = new Pose2d();

        q = new Matrix<>(Nat.N3(), Nat.N1());
        for (int i = 0; i < 3; i++) {
            q.set(i, 0, MathUtil.square(Constants.kVisionStateStdDevs[i]));
        }

        qWithMoreTrustAngle = new Matrix<>(Nat.N3(), Nat.N1());
        for (int i = 0; i < 3; i++) {
            qWithMoreTrustAngle.set(i, 0, MathUtil.square(i == 2
                    ? Constants.kVisionInitialAngleStdDev
                    : Constants.kVisionStateStdDevs[i]));
        }

        ignoreVision = false;
    }

    public void setIgnoreVision(boolean ignoreVision) {
        this.ignoreVision = ignoreVision;
    }

    /**
     * @return the current pose estimate
     */
    public Pose2d getEstimatedPose() {
        return latestPose;
    }

    /**
     * Sets the pose estimate to a known pose and discards all previous vision
     * updates.
     *
     * @param newPose new known pose to set
     */
    public void resetPose(Pose2d newPose) {
        basePose = newPose;
        updates.clear();
        update(false);
    }

    private double enabledTimestamp = Double.NaN;

    public void update(Twist2d driveTwist) {
        // Trust vision angle estimates for 5 seconds at start of teleop
        boolean trustTurnMore = false;
        if (DriverStation.isTeleopEnabled() && !DriverStation.isFMSAttached()) {
            if (Double.isNaN(enabledTimestamp)) {
                enabledTimestamp = Timer.getFPGATimestamp();
            }

            if (Timer.getFPGATimestamp() - enabledTimestamp < Constants.kVisionInitialTrustTime) {
                trustTurnMore = true;
            }
        } else {
            enabledTimestamp = Double.NaN;
        }

        List<VisionUpdate> visionData = new ArrayList<>();
        for (VisionSource input : visionSources) {
            visionData.addAll(input.getNewUpdates());
        }
        updates.put(Timer.getFPGATimestamp(), new PoseUpdate(driveTwist, new ArrayList<>()));

        if (ignoreVision) {
            update(false);
            return;
        }

        for (VisionUpdate visionUpdate : visionData) {
            double timestamp = visionUpdate.timestamp;

            if (updates.containsKey(timestamp)) {
                List<VisionUpdate> oldUpdates = updates.get(timestamp).visionUpdates;
                oldUpdates.add(visionUpdate);
                oldUpdates.sort(this::compareStdDevs);
            } else {
                Map.Entry<Double, PoseUpdate> prevUpdate = updates.floorEntry(timestamp);
                Map.Entry<Double, PoseUpdate> nextUpdate = updates.ceilingEntry(timestamp);

                if (prevUpdate == null || nextUpdate == null)
                    continue;

                Twist2d prevToVisionTwist = MathUtil.multiplyTwist(
                        nextUpdate.getValue().twist,
                        MathUtil.percent(timestamp, prevUpdate.getKey(), nextUpdate.getKey()));
                Twist2d visionToNextTwist = MathUtil.multiplyTwist(
                        nextUpdate.getValue().twist,
                        (nextUpdate.getKey() - timestamp) / (nextUpdate.getKey() - prevUpdate.getKey()));

                List<VisionUpdate> newVisionUpdates = new ArrayList<>();
                newVisionUpdates.add(visionUpdate);

                // Insert new update entry for this vision update
                updates.put(timestamp, new PoseUpdate(prevToVisionTwist, newVisionUpdates));

                // Overwrite nextUpdate with twist after this vision update
                updates.put(nextUpdate.getKey(), new PoseUpdate(visionToNextTwist, nextUpdate.getValue().visionUpdates));
            }
        }

        update(trustTurnMore);
    }

    private void update(boolean trustTurnMore) {
        Matrix<N3, N1> selectedQ = trustTurnMore ? qWithMoreTrustAngle : q;

        while (updates.size() > 1 && updates.firstKey() < Timer.getFPGATimestamp() - Constants.kVisionHistoryTime) {
            Map.Entry<Double, PoseUpdate> update = updates.pollFirstEntry();
            basePose = update.getValue().apply(basePose, selectedQ);
        }

        latestPose = basePose;
        for (Map.Entry<Double, PoseUpdate> entry : updates.entrySet()) {
            latestPose = entry.getValue().apply(latestPose, selectedQ);
        }

        // Show current estimate and vision updates on field view
        FieldView.robotPose.setPose(latestPose);
        List<Pose2d> visionPoses = new ArrayList<>();
        for (PoseUpdate update : updates.values()) {
            for (VisionUpdate visionUpdate : update.visionUpdates) {
                visionPoses.add(visionUpdate.estPose);
            }
        }
        FieldView.visionEstimates.setPoses(visionPoses);

        Logger.recordOutput("Drive/Estimated Pose", latestPose);
        Logger.recordOutput("Drive/Vision Estimates", visionPoses.toArray(new Pose2d[0]));
    }

    private int compareStdDevs(VisionUpdate u1, VisionUpdate u2) {
        return -Double.compare(
                u1.stdDevs.get(0, 0) + u1.stdDevs.get(1, 0),
                u2.stdDevs.get(0, 0) + u2.stdDevs.get(1, 0)
        );
    }

    private static final class PoseUpdate {
        public final Twist2d twist;
        public final List<VisionUpdate> visionUpdates;

        public PoseUpdate(Twist2d twist, List<VisionUpdate> visionUpdates) {
            this.twist = twist;
            this.visionUpdates = visionUpdates;
        }

        public Pose2d apply(Pose2d prevPose, Matrix<N3, N1> q) {
            Pose2d pose = prevPose.exp(twist);

            for (VisionUpdate visionUpdate : visionUpdates) {
                Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
                double[] r = new double[3];
                for (int i = 0; i < 3; i++)
                    r[i] = MathUtil.square(visionUpdate.stdDevs.get(i, 0));

                for (int row = 0; row < 3; row++) {
                    if (q.get(row, 0) == 0.0) {
                        visionK.set(row, row, 0.0);
                    } else {
                        double qRow0 = q.get(row, 0);
                        visionK.set(row, row, qRow0 / (qRow0 + Math.sqrt(qRow0 * r[row])));
                    }
                }

                Twist2d visionTwist = pose.log(visionUpdate.estPose);
                Matrix<N3, N1> twistMatrix = visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));

                pose = pose.exp(new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));
            }

            return pose;
        }
    }
}
