package com.swrobotics.robot.subsystems.algae;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface AlgaeIO {
    class Inputs extends AutoLoggedInputs {
        // Rotations, from horizontal
        public double currentAngleDeg;

        public double voltageOut;
    }

    void updateInputs(Inputs inputs);

    void setTargetAngle(double targetAngleRot);

    void setVoltage(double targetVoltage);
}
