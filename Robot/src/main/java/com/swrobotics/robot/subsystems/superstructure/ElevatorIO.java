package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface ElevatorIO {
    final class Inputs extends AutoLoggedInputs {
        public double currentHeightPct;
        public double currentVelocityPctPerSec;
    }

    void updateInputs(Inputs inputs);

    void setTarget(double heightPct, double ffVelocityPctPerSec, double ffAccelPctPerSecSq);

    // Positive up, negative down
    void setVoltage(double volts);
}
