package com.swrobotics.lib.tunable;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A value that is tunable in NetworkTables and logged using AdvantageKit.
 * Note: these do not save when the robot restarts! Always copy the value into
 * {@link com.swrobotics.robot.config.Constants} after tuning.
 *
 * @param <T> type of the value
 */
public abstract class LoggedTunableValue<T> implements Supplier<T> {
    public static final String PREFIX = "Tunables/";

    private static final List<LoggedTunableValue<?>> TUNABLES = new ArrayList<>();

    public static void updateAll() {
        for (LoggedTunableValue<?> value : TUNABLES) {
            value.update();
        }
    }

    private final NetworkTableEntry entry;
    private final T defaultValue;
    private T value;

    private final List<Consumer<T>> changeListeners;
    private final LoggableInputs inputs;

    protected abstract T getValue(NetworkTableEntry entry, T defaultValue);
    protected abstract void setValue(NetworkTableEntry entry, T value);

    protected abstract void toLog(LogTable table, String key, T value);
    protected abstract T fromLog(LogTable table, String key, T defaultValue);

    public LoggedTunableValue(String key, T defaultValue) {
        this.defaultValue = defaultValue;
        changeListeners = new ArrayList<>();

        String path = PREFIX + key;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("");
        String[] parts = path.split("/");
        for (int i = 0; i < parts.length - 1; i++) {
            table = table.getSubTable(parts[i]);
        }
        entry = table.getEntry(parts[parts.length - 1]);

        if (!entry.exists())
            setValue(entry, defaultValue);
        value = getValue(entry, defaultValue);

        inputs = new LoggableInputs() {
            @Override
            public void toLog(LogTable table) {
                LoggedTunableValue.this.toLog(table, key, value);
            }

            @Override
            public void fromLog(LogTable table) {
                value = LoggedTunableValue.this.fromLog(table, key, defaultValue);
            }
        };

        TUNABLES.add(this);
    }

    @Override
    public T get() {
        return value;
    }

    public void set(T value) {
        setValue(entry, value);
        this.value = value;
    }

    public void onChange(Consumer<T> listener) {
        changeListeners.add(listener);
    }

    public void nowAndOnChange(Consumer<T> listener) {
        listener.accept(value);
        onChange(listener);
    }

    private void update() {
        T oldValue = value;
        if (!Logger.hasReplaySource()) {
            value = getValue(entry, defaultValue);
        }
        Logger.processInputs(PREFIX, inputs);

        if (!Objects.equals(oldValue, value)) {
            for (Consumer<T> listener : changeListeners) {
                listener.accept(value);
            }
        }
    }
}
