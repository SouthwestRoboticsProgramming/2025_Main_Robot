package com.swrobotics.robot.subsystems.tunnel;

public final class TunnelIOSim implements TunnelIO {
    @Override
    public void updateInputs(Inputs inputs) {
        inputs.hasCoral = false;
    }

    @Override
    public void setVoltage(double volts) {

    }
}
