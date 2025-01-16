package com.swrobotics.robot.subsystems.vision.limelight;

import edu.wpi.first.math.geometry.Translation2d;
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
        public double[] stdDevsData = new double[0];

        // Not used by robot code, but likely will be useful for log viewing
        // with AdvantageScope's Points tab. Ideally we would log the camera's
        // video stream, but that requires too much bandwidth.
        public Translation2d[] corners;

        @Override
        public void toLog(LogTable table) {
            megaTag1.toLog(table.getSubtable("megaTag1"));
            megaTag2.toLog(table.getSubtable("megaTag2"));
            table.put("stdDevs", stdDevsData);
            table.put("corners", corners);
        }

        @Override
        public void fromLog(LogTable table) {
            megaTag1.fromLog(table.getSubtable("megaTag1"));
            megaTag2.fromLog(table.getSubtable("megaTag2"));
            stdDevsData = table.get("stdDevs", stdDevsData);
            corners = table.get("corners", corners);
        }
    }

    void updateInputs(Inputs inputs);

    // Both in degrees CCW
    void updateRobotState(double yawAngle, double yawRate);
}
