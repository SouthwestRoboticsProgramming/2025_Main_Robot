package com.swrobotics.robot.subsystems.algae;

import com.swrobotics.lib.utils.MathUtil;

public class AlgaeIOSim implements AlgaeIO {
    private double currentAngle;
    private double targetAngle;
    private double voltage;

    @Override
    public void updateInputs(Inputs inputs) {
        currentAngle = MathUtil.lerp(currentAngle, targetAngle, 0.1);
        inputs.currentAngleDeg = currentAngle;
        inputs.voltageOut = voltage;
    }

    @Override
    public void setTargetAngle(double targetAngleDeg) {
        targetAngle = targetAngleDeg;
    }

    @Override
    public void setVoltage(double targetVoltage) {
        voltage = targetVoltage;
    }
    
}
