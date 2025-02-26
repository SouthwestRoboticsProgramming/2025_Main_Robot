package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.config.Constants;

public final class OuttakePivotIOSim implements OuttakePivotIO {
    private double prevPosition;
    private double position;

    public OuttakePivotIOSim() {
        prevPosition = position = 0.25;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.currentAngleRot = position;
        inputs.currentVelocityRotPerSec = (position - prevPosition) / Constants.kPeriodicTime;
        inputs.absoluteAngleRot = position;
        inputs.absoluteAngleOK = true;

        prevPosition = position;
    }

    @Override
    public void setTarget(double targetAngleRot, double ffVelocityRotPerSec, boolean hasCoral) {
        position = targetAngleRot;
    }

    @Override
    public void setCurrentPosition(double currentAngleRot) {
        position = currentAngleRot;
    }

    @Override
    public void calibrateEncoder() {
        prevPosition = position = 0.25;
    }
}
