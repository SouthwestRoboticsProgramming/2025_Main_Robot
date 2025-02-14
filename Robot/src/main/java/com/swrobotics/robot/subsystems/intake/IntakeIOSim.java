package com.swrobotics.robot.subsystems.intake;

import com.swrobotics.lib.utils.MathUtil;

public final class IntakeIOSim implements IntakeIO {
    private double currentAngle;
    private double targetAngle;
    private double voltage;

    @Override
    public void updateInputs(Inputs inputs) {
        currentAngle = MathUtil.lerp(currentAngle, targetAngle, 0.1);
        inputs.currentAngleRot = currentAngle;
        inputs.voltageOut = voltage;
    }

    @Override
    public void setTargetAngle(double angleRot) {
        targetAngle = angleRot;
    }

    @Override
    public void setVoltage(double targetVoltage) {
        voltage = targetVoltage;
    }

    @Override
    public void calibrateEncoder() {}
}
