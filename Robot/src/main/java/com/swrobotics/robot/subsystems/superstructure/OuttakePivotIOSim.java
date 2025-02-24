package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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

        prevPosition = position;
    }

    @Override
    public void setTarget(double targetAngleRot, double ffVelocityRotPerSec, boolean hasCoral) {
        position = targetAngleRot;
    }

    @Override
    public void syncWithEncoder() {
        // no
    }

    @Override
    public void calibrateEncoder() {
        prevPosition = position = 0.25;
    }
}
