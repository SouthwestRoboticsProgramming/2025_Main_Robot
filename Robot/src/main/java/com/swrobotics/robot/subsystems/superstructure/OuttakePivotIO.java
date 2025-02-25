package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface OuttakePivotIO {
    class Inputs extends AutoLoggedInputs {
        // Rotations, from horizontal
        public double currentAngleRot;
        public double currentVelocityRotPerSec;

        public double absoluteAngleRot;
        public boolean absoluteAngleOK;
    }

    void updateInputs(Inputs inputs);

    void setTarget(double targetAngleRot, double ffVelocityRotPerSec, boolean hasCoral);

    void setCurrentPosition(double currentAngleRot);

    void calibrateEncoder();
}
