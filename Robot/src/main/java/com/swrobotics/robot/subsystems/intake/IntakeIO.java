package com.swrobotics.robot.subsystems.intake;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface IntakeIO {
    class Inputs extends AutoLoggedInputs {
        // Rotations, from horizontal
        public double currentAngleRot;

        public double voltageOut;
    }

    void updateInputs(Inputs inputs);

    void setTargetAngle(double targetAngleRot);

    void setVoltage(double targetVoltage);

    void calibrateEncoder();
}
