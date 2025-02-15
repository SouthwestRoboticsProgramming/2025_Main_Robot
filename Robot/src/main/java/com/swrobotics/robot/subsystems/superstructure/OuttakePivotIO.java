package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface OuttakePivotIO {
    class Inputs extends AutoLoggedInputs {
        // Rotations, from horizontal
        public double currentAngleRot;
    }

    void updateInputs(Inputs inputs);

    void setTargetAngle(double targetAngleRot, boolean hasCoral);

    void syncWithEncoder();

    void calibrateEncoder();
}
