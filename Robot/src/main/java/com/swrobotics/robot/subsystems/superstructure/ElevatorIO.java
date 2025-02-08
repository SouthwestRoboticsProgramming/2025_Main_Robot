package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface ElevatorIO {
    final class Inputs extends AutoLoggedInputs {
        public double currentHeightPct;
    }

    void updateInputs(Inputs inputs);

    void setTargetHeight(double heightPct);

    void setNeutral();
}
