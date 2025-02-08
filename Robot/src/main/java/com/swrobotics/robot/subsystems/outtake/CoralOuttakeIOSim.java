package com.swrobotics.robot.subsystems.outtake;

public class CoralOuttakeIOSim implements CoralOuttakeIO {
    @Override
    public void updateInputs(Inputs inputs) {
        inputs.hasPiece = false;
    }

    @Override
    public void setVoltage(double voltage) {

    }
}
