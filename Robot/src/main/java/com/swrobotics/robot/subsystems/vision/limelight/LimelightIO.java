package com.swrobotics.robot.subsystems.vision.limelight;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LimelightIO {
    final class EstimateInputs implements LoggableInputs {
        public long timestamp;
        public double[] data;

        @Override
        public void toLog(LogTable table) {
            table.put("timestamp", timestamp);
            table.put("data", data);
        }

        @Override
        public void fromLog(LogTable table) {
            timestamp = table.get("timestamp", timestamp);
            data = table.get("data", data);
        }
    }

    final class Inputs implements LoggableInputs {
        public final EstimateInputs megaTag1 = new EstimateInputs();
        public final EstimateInputs megaTag2 = new EstimateInputs();

        @Override
        public void toLog(LogTable table) {
            megaTag1.toLog(table.getSubtable("megaTag1"));
            megaTag2.toLog(table.getSubtable("megaTag2"));
        }

        @Override
        public void fromLog(LogTable table) {
            megaTag1.fromLog(table.getSubtable("megaTag1"));
            megaTag2.fromLog(table.getSubtable("megaTag2"));
        }
    }

    void updateInputs(Inputs inputs);

    // Both in degrees CCW
    void updateRobotState(double yawAngle, double yawRate);
}
