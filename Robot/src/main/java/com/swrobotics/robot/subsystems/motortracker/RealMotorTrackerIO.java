package com.swrobotics.robot.subsystems.motortracker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import com.swrobotics.lib.utils.CTREUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class RealMotorTrackerIO implements MotorTrackerIO {
    private record TrackedMotor(
            String name,
            String canBus,
            StatusSignal<Temperature> tempStatus,
            StatusSignal<Current> supplyCurrentStatus,
            StatusSignal<Current> statorCurrentStatus,
            StatusSignal<Integer> faultsStatus,
            StatusSignal<Integer> stickyFaultsStatus) {}

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
            inputs.faults = new int[count];
            inputs.stickyFaults = new int[count];
        }

        // Refresh all status signals grouped by CAN bus
        for (BaseStatusSignal[] group : allSignals) {
            BaseStatusSignal.refreshAll(group);
        }

        for (int i = 0; i < count; i++) {
            TrackedMotor motor = motors.get(i);
            inputs.names[i] = motor.name();
            inputs.temperatures[i] = motor.tempStatus().getValueAsDouble(); // Celsius
            inputs.supplyCurrents[i] = motor.supplyCurrentStatus().getValueAsDouble(); // Amps
            inputs.statorCurrents[i] = motor.statorCurrentStatus().getValueAsDouble(); // Amps
            inputs.faults[i] = motor.faultsStatus().getValue();
            inputs.stickyFaults[i] = motor.stickyFaultsStatus().getValue();
        }
    }

    @Override
    public void addMotor(String name, TalonFX motor) {
        if (allSignals != null)
            throw new IllegalStateException("Can only add motors in robotInit()");

        StatusSignal<Temperature> tempStatus = motor.getDeviceTemp();
        StatusSignal<Current> supplyCurrentStatus = motor.getSupplyCurrent();
        StatusSignal<Current> statorCurrentStatus = motor.getStatorCurrent();
        StatusSignal<Integer> faultsStatus = motor.getFaultField();
        StatusSignal<Integer> stickyFaultsStatus = motor.getStickyFaultField();

        motors.add(new TrackedMotor(
                name,
                motor.getNetwork(),
                tempStatus,
                supplyCurrentStatus,
                statorCurrentStatus,
                faultsStatus,
                stickyFaultsStatus
        ));
    }
}
