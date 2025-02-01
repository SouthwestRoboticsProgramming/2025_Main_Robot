package com.swrobotics.robot.subsystems.outtake;

public class IndexerIOSim implements IndexerIO {
    private double voltage;

    @Override
    public void updateInputs(Inputs inputs) {
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
    
}
