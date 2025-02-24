package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.config.Constants;

public final class ElevatorIOSim implements ElevatorIO {
    private double prevPosition;
    private double position;

    public ElevatorIOSim() {
        prevPosition = 0;
        position = 0;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.currentHeightPct = position;
        inputs.currentVelocityPctPerSec = (position - prevPosition) / Constants.kPeriodicTime;

        prevPosition = position;
    }

    @Override
    public void setTarget(double heightPct, double ffVelocityPctPerSec) {
        position = heightPct;
    }

    @Override
    public void setClimbMode(boolean activelyClimbing) {}
}
