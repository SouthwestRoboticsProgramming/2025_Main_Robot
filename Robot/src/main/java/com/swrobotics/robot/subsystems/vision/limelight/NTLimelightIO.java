package com.swrobotics.robot.subsystems.vision.limelight;

import edu.wpi.first.networktables.*;

public final class NTLimelightIO implements LimelightIO {
    private static final String MEGATAG_1_NAME = "botpose_wpiblue";
    private static final String MEGATAG_2_NAME = "botpose_orb_wpiblue";
    private static final String ORIENTATION_NAME = "robot_orientation_set";

    private final DoubleArraySubscriber mt1EstimateSub;
    private final DoubleArraySubscriber mt2EstimateSub;

    private final DoubleArrayPublisher robotOrientationPub;

    public NTLimelightIO(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);

        mt1EstimateSub = table.getDoubleArrayTopic(MEGATAG_1_NAME).subscribe(new double[0]);
        mt2EstimateSub = table.getDoubleArrayTopic(MEGATAG_2_NAME).subscribe(new double[0]);

        robotOrientationPub = table.getDoubleArrayTopic(ORIENTATION_NAME).publish();
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
    }

    @Override
    public void updateRobotState(double yawAngle, double yawRate) {
        robotOrientationPub.set(new double[] {
                yawAngle, yawRate, 0, 0, 0, 0
        });
    }
}
