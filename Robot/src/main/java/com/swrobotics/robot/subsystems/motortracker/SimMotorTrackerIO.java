package com.swrobotics.robot.subsystems.motortracker;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class SimMotorTrackerIO implements MotorTrackerIO {
    // We don't have anything physical to measure, just assume room temp
    private static final double TEMP = 22;

    private final List<String> names = new ArrayList<>();

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.names = names.toArray(new String[0]);
        inputs.temperatures = new double[names.size()];
        inputs.supplyCurrents = new double[names.size()];
        inputs.statorCurrents = new double[names.size()];
        Arrays.fill(inputs.temperatures, TEMP + 5 * Math.random());
    }

    @Override
    public void addMotor(String name, TalonFX motor) {
        names.add(name);
    }
}
