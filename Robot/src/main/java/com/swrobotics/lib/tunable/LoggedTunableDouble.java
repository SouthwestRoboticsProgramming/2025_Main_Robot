package com.swrobotics.lib.tunable;

import edu.wpi.first.networktables.NetworkTableEntry;
import org.littletonrobotics.junction.LogTable;

public final class LoggedTunableDouble extends LoggedTunableValue<Double> {
    public LoggedTunableDouble(String key, double defaultValue) {
        super(key, defaultValue);
    }

    @Override
    protected Double getValue(NetworkTableEntry entry, Double defaultValue) {
        return entry.getDouble(defaultValue);
    }

    @Override
    protected void setValue(NetworkTableEntry entry, Double value) {
        entry.setDouble(value);
    }

    @Override
    protected void toLog(LogTable table, String key, Double value) {
        table.put(key, value);
    }

    @Override
    protected Double fromLog(LogTable table, String key, Double defaultValue) {
        return table.get(key, defaultValue);
    }
}
