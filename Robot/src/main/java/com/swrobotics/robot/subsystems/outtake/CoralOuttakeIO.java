package com.swrobotics.robot.subsystems.outtake;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface CoralOuttakeIO {
    class Inputs extends AutoLoggedInputs {
        public double voltage;
        public boolean hasPiece;
    }

    void updateInputs(Inputs inputs);

    void setVoltage(double voltage);
}
