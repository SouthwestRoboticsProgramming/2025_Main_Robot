package com.swrobotics.robot.subsystems.outtake;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface AlgaeOuttakeIO {
    class Inputs extends AutoLoggedInputs {
        public double voltage;

        public boolean hasPiece;

        public int successfulDaqs;
        public int failedDaqs;
    }

    void updateInputs(Inputs inputs);

    void setVoltage(double voltage);
}
