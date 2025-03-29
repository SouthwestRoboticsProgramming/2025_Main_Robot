package com.swrobotics.robot.subsystems.outtake.coral;

public class CoralOuttakeIOSim implements CoralOuttakeIO {
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
