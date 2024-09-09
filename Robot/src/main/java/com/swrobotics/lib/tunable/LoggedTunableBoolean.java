package com.swrobotics.lib.tunable;

import edu.wpi.first.networktables.NetworkTableEntry;
import org.littletonrobotics.junction.LogTable;

public final class LoggedTunableBoolean extends LoggedTunableValue<Boolean> {
    public LoggedTunableBoolean(String key, Boolean defaultValue) {
        super(key, defaultValue);
    }

    @Override
    protected Boolean getValue(NetworkTableEntry entry, Boolean defaultValue) {
        return entry.getBoolean(defaultValue);
    }

    @Override
    protected void setValue(NetworkTableEntry entry, Boolean value) {
        entry.setBoolean(value);
    }

    @Override
    protected void toLog(LogTable table, String key, Boolean value) {
        table.put(key, value);
    }

    @Override
    protected Boolean fromLog(LogTable table, String key, Boolean defaultValue) {
        return table.get(key, defaultValue);
    }
}
