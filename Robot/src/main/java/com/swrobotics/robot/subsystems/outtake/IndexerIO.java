package com.swrobotics.robot.subsystems.outtake;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface IndexerIO {
    class Inputs extends AutoLoggedInputs {
    }

    void updateInputs(Inputs inputs);

    void setVoltage(double voltage);
}
