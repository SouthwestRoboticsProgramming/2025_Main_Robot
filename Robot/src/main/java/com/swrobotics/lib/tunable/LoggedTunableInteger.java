package com.swrobotics.lib.tunable;

import edu.wpi.first.networktables.NetworkTableEntry;
import org.littletonrobotics.junction.LogTable;

public final class LoggedTunableInteger extends LoggedTunableValue<Integer> {
    public LoggedTunableInteger(String key, int defaultValue) {
        super(key, defaultValue);
    }

    @Override
    protected Integer getValue(NetworkTableEntry entry, Integer defaultValue) {
        return (int) entry.getDouble(defaultValue);
    }

    @Override
    protected void setValue(NetworkTableEntry entry, Integer value) {
        entry.setDouble(value);
    }

    @Override
    protected void toLog(LogTable table, String key, Integer value) {
        table.put(key, value.doubleValue());
    }

    @Override
    protected Integer fromLog(LogTable table, String key, Integer defaultValue) {
        return (int) table.get(key, defaultValue.doubleValue());
    }
}
