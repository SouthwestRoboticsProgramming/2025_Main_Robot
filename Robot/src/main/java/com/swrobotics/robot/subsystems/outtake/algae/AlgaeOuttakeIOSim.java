package com.swrobotics.robot.subsystems.outtake.algae;

public class AlgaeOuttakeIOSim implements AlgaeOuttakeIO {
    @Override
    public void updateInputs(Inputs inputs) {
        inputs.hasPiece = false;
    }

    @Override
    public void setVoltage(double voltage) {}
    
}
