package com.swrobotics.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.swrobotics.lib.utils.MathUtil;

import edu.wpi.first.units.measure.Angle;

public class AlgaeIOSim implements AlgaeIO {
    private double currentAngle;
    private double targetAngle;
    private double voltage;

    @Override
    public void updateInputs(Inputs inputs) {
        currentAngle = MathUtil.lerp(currentAngle, targetAngle, 0.1);
        inputs.currentAngleRot = currentAngle;
        inputs.statorCurrent = voltage; // good enough approximation
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
