package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface OuttakePivotIO {
    class Inputs extends AutoLoggedInputs {
        // Rotations, from horizontal
        public double currentAngle;
    }

    void updateInputs(Inputs inputs);

    void setTargetAngle(double targetAngleRot);

    void calibrateEncoder();
}
