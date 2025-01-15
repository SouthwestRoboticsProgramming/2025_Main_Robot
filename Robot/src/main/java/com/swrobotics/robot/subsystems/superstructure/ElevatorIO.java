package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface ElevatorIO {
    final class Inputs extends AutoLoggedInputs {
        public double currentHeight;
    }

    void updateInputs(Inputs inputs);

    // percent of full height
    void setTargetHeight(double heightPct);

    void setNeutral();
}
