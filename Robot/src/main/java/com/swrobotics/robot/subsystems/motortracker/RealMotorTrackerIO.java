package com.swrobotics.robot.subsystems.motortracker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class RealMotorTrackerIO implements MotorTrackerIO {
    private record TrackedMotor(
            String name,
            String canBus,
            StatusSignal<Double> tempStatus,
            StatusSignal<Double> supplyCurrentStatus,
            StatusSignal<Double> statorCurrentStatus) {}

    private final List<TrackedMotor> motors = new ArrayList<>();
    private BaseStatusSignal[][] allSignals = null;

    @Override
    public void updateInputs(Inputs inputs) {
        int count = motors.size();

        // Only allocate arrays once
        if (allSignals == null) {
            Map<String, List<BaseStatusSignal>> groups = new HashMap<>();
            for (TrackedMotor motor : motors) {
                List<BaseStatusSignal> group = groups.computeIfAbsent(motor.canBus, (b) -> new ArrayList<>());
                group.add(motor.tempStatus);
                group.add(motor.supplyCurrentStatus);
                group.add(motor.statorCurrentStatus);
            }

            allSignals = new BaseStatusSignal[groups.size()][];
            int groupIdx = 0;
            for (List<BaseStatusSignal> group : groups.values()) {
                BaseStatusSignal[] signals = group.toArray(new BaseStatusSignal[0]);
                allSignals[groupIdx++] = signals;
            }

            inputs.names = new String[count];
            inputs.temperatures = new double[count];
            inputs.supplyCurrents = new double[count];
            inputs.statorCurrents = new double[count];
        }

        // Refresh all status signals grouped by CAN bus
        for (BaseStatusSignal[] group : allSignals) {
            BaseStatusSignal.refreshAll(group);
        }

        for (int i = 0; i < count; i++) {
            TrackedMotor motor = motors.get(i);
            inputs.names[i] = motor.name();
            inputs.temperatures[i] = motor.tempStatus().getValue();
            inputs.supplyCurrents[i] = motor.supplyCurrentStatus().getValue();
            inputs.statorCurrents[i] = motor.statorCurrentStatus().getValue();
        }
    }

    @Override
    public void addMotor(String name, TalonFX motor) {
        if (allSignals != null)
            throw new IllegalStateException("Can only add motors in robotInit()");

        StatusSignal<Double> tempStatus = motor.getDeviceTemp();
        StatusSignal<Double> supplyCurrentStatus = motor.getSupplyCurrent();
        StatusSignal<Double> statorCurrentStatus = motor.getStatorCurrent();

        StatusSignal.setUpdateFrequencyForAll(
                4, // 4 Hz is minimum update frequency supported
                supplyCurrentStatus,
                statorCurrentStatus
        );

        motors.add(new TrackedMotor(name, motor.getNetwork(), tempStatus, supplyCurrentStatus, statorCurrentStatus));
    }
}
