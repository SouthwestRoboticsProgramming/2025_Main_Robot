package com.swrobotics.lib.tunable;

import edu.wpi.first.networktables.NetworkTableEntry;
import org.littletonrobotics.junction.LogTable;

public final class LoggedTunableString extends LoggedTunableValue<String> {
    public LoggedTunableString(String key, String defaultValue) {
        super(key, defaultValue);
    }

    @Override
    protected String getValue(NetworkTableEntry entry, String defaultValue) {
        return entry.getString(defaultValue);
    }

    @Override
    protected void setValue(NetworkTableEntry entry, String value) {
        entry.setString(value);
    }

    @Override
    protected void toLog(LogTable table, String key, String value) {
        table.put(key, value);
    }

    @Override
    protected String fromLog(LogTable table, String key, String defaultValue) {
        return table.get(key, defaultValue);
    }
}
