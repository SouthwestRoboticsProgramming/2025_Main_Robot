package com.swrobotics.robot.subsystems.vision.limelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;

public final class NTLimelightIO implements LimelightIO {
    private static final String MEGATAG_1_NAME = "botpose_wpiblue";
    private static final String MEGATAG_2_NAME = "botpose_orb_wpiblue";
    private static final String STD_DEVS_NAME = "stddevs";
    private static final String CORNERS_NAME = "tcornxy";
    private static final String ORIENTATION_NAME = "robot_orientation_set";
    private static final String LOCATION_NAME = "camerapose_robotspace_set";

    private final DoubleArraySubscriber mt1EstimateSub;
    private final DoubleArraySubscriber mt2EstimateSub;
    private final DoubleArraySubscriber stdDevsSub;
    private final DoubleArraySubscriber cornersSub;

    private final DoubleArrayPublisher robotOrientationPub;

    public NTLimelightIO(String limelightName, LimelightCamera.MountingLocation location) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);

        mt1EstimateSub = table.getDoubleArrayTopic(MEGATAG_1_NAME).subscribe(new double[0]);
        mt2EstimateSub = table.getDoubleArrayTopic(MEGATAG_2_NAME).subscribe(new double[0]);
        stdDevsSub = table.getDoubleArrayTopic(STD_DEVS_NAME).subscribe(new double[0]);
        cornersSub = table.getDoubleArrayTopic(CORNERS_NAME).subscribe(new double[0]);

        robotOrientationPub = table.getDoubleArrayTopic(ORIENTATION_NAME).publish();

        DoubleArrayPublisher mountingLocationPub = table.getDoubleArrayTopic(LOCATION_NAME).publish();
        mountingLocationPub.set(new double[] {
                location.forward(), location.right(), location.up(),
                location.roll(), location.pitch(), location.yaw()
        });
    }

    private void updateEstimateInput(EstimateInputs inputs, DoubleArraySubscriber sub) {
        // Limelight 3G AprilTag processing runs at about 30 FPS, which is less
        // than the periodic rate of 50 Hz, so this shouldn't miss any frames
        TimestampedDoubleArray estimate = sub.getAtomic();
        inputs.timestamp = estimate.timestamp;
        inputs.data = estimate.value;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        updateEstimateInput(inputs.megaTag1, mt1EstimateSub);
        updateEstimateInput(inputs.megaTag2, mt2EstimateSub);
        inputs.stdDevsData = stdDevsSub.get();

        double[] cornersData = cornersSub.get();
        inputs.corners = new Translation2d[cornersData.length / 2];
        for (int i = 0; i < inputs.corners.length; i++) {
            inputs.corners[i] = new Translation2d(
                    cornersData[i * 2],
                    cornersData[i * 2 + 1]
            );
        }
    }

    @Override
    public void updateRobotState(double yawAngle, double yawRate) {
        robotOrientationPub.set(new double[] {
                yawAngle, yawRate, 0, 0, 0, 0
        });
    }
}
