package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.config.Constants;

public final class ElevatorIOSim implements ElevatorIO {
    private double prevPosition;
    private double position;

    private boolean voltageControl;
    private double volts;

    public ElevatorIOSim() {
        prevPosition = 0;
        position = 0;

        voltageControl = false;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        if (voltageControl) {
            position += volts * Constants.kPeriodicTime * 0.2;

            if (volts == 0.0) {
                position = 0.0; // Fall
            }
        }

        inputs.currentHeightPct = position;
        inputs.currentVelocityPctPerSec = (position - prevPosition) / Constants.kPeriodicTime;

        prevPosition = position;
    }

    @Override
    public void setTarget(double heightPct, double ffVelocityPctPerSec, double ffAccelPctPerSecSq) {
        voltageControl = false;
        position = heightPct;
    }

    @Override
    public void setVoltage(double volts) {
        // Simulate force from robot climbing
        if (volts < 0)
            volts += Constants.kElevatorClimbHoldVolts.get();

        voltageControl = true;
        this.volts = volts;
    }
}
