package com.swrobotics.robot.subsystems.tunnel;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface TunnelIO {
    final class Inputs extends AutoLoggedInputs {
        public boolean hasCoral;
    }

    void updateInputs(Inputs inputs);

    void setVoltage(double volts);
}
