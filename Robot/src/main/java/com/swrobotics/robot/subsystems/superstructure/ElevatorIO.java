package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface ElevatorIO {
    final class Inputs extends AutoLoggedInputs {
        public double currentHeightPct;
        public double currentVelocityPctPerSec;
    }

    void updateInputs(Inputs inputs);

    void setTarget(double heightPct, double ffVelocityPctPerSec);

    // Positive up, negative down
    void setVoltage(double volts);
}
