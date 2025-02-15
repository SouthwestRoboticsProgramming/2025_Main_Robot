package com.swrobotics.robot.subsystems.outtake;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface CoralOuttakeIO {
    class Inputs extends AutoLoggedInputs {
        public double voltage;

        public boolean hasPiece;
        public double positionAtPieceDetect;
    }

    void updateInputs(Inputs inputs);

    void setVoltage(double voltage);

    void setHoldPosition(double position);
}
