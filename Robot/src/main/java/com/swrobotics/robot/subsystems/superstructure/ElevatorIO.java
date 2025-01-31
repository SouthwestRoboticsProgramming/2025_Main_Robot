package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface ElevatorIO {
    final class Inputs extends AutoLoggedInputs {
        public double currentHeightMeters;
    }

    void updateInputs(Inputs inputs);

    void setTargetHeight(double heightMeters);

    void setNeutral();

    void setClimbMode(boolean activelyClimbing);
}
