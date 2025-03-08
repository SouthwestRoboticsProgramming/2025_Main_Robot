package com.swrobotics.robot.subsystems.outtake;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface OuttakeIO {
    class Inputs extends AutoLoggedInputs {
        public double voltage;

        public boolean hasPiece;
        public double positionAtPieceDetect;

        public int successfulDaqs;
        public int failedDaqs;
    }

    void updateInputs(Inputs inputs);

    void setVoltage(double voltage);

    void setHoldPosition(double position);

    void setBeamBreakIgnored(boolean ignored);
}
