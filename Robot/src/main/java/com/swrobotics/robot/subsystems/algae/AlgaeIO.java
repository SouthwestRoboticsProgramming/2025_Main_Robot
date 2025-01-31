package com.swrobotics.robot.subsystems.algae;

import com.swrobotics.robot.logging.AutoLoggedInputs;

import edu.wpi.first.units.measure.Angle;

public interface AlgaeIO {
    class Inputs extends AutoLoggedInputs {
        // Rotations, from horizontal
        public double currentAngleRot;

        public double voltageOut;
    }

    void updateInputs(Inputs inputs);

    void setTargetAngle(Angle targetAngle);

    void setVoltage(double targetVoltage);

    void calibrateEncoder();
}
