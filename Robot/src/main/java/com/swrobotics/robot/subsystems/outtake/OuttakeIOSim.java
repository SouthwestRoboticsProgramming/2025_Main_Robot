package com.swrobotics.robot.subsystems.outtake;

public class OuttakeIOSim implements OuttakeIO {
    @Override
    public void updateInputs(Inputs inputs) {
        inputs.hasPiece = false;
    }

    @Override
    public void setVoltage(double voltage) {
    }

    @Override
    public void setHoldPosition(double position) {
    }

    @Override
    public void setBeamBreakIgnored(boolean ignored) {
    }
}
