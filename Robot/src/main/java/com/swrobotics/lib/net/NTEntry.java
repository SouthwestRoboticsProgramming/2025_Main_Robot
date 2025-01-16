package com.swrobotics.lib.net;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.*;
import java.util.function.Supplier;

/**
 * A data entry stored in NetworkTables. These are also logged to AdvantageKit
 * in case NetworkTables data is lost.
 *
 * @param <T> type of data stored in the entry
 */
public abstract class NTEntry<T> implements Supplier<T> {
    private static final String PREFIX = "NTEntry";
    private static final List<NTEntry<?>> ALL_ENTRIES = new ArrayList<>();
    private static boolean persistentCleaned = false;

    /**
     * Updates all NTEntries. Should be called once per periodic.
     */
    public static void updateAll() {
        for (NTEntry<?> entry : ALL_ENTRIES) {
            entry.update();
        }
    }

    /**
     * Removes any unused persistent values left over in NetworkTables. This
     * should be called after all persistent NTEntries have been created.
     */
    public static void cleanPersistent() {
        // Find the names of the persistent values the code uses
        Set<String> expected = new HashSet<>();
        for (NTEntry<?> entry : ALL_ENTRIES) {
            if (entry.entry.isPersistent()) {
                expected.add(entry.entry.getName());
            }
        }

        // Delete values we don't use
        Topic[] allTopics = NetworkTableInstance.getDefault().getTopics();
        for (Topic topic : allTopics) {
            String name = topic.getName();
            if (topic.isPersistent() && !expected.contains(name)) {
                String value;
                try (GenericSubscriber sub = topic.genericSubscribe()) {
                    value = Objects.toString(sub.get().getValue());
                }

                // Provide several ways of getting the value back if it was
                // deleted unintentionally
                Logger.recordOutput("NTEntry/Deleted" + name, value);
                DriverStation.reportWarning("NTEntry: Deleted " + name + " (value was `" + value + "`)", false);

                // Keep it around until robot or code is restarted
                topic.setRetained(true);

                // Remove the value
                topic.setPersistent(false);
            }
        }

        // Record that we've done this so we can detect if an entry is created
        // after this was called, to prevent values from being wrongly deleted
        persistentCleaned = true;
    }

    private final NetworkTableEntry entry;
    private final List<Runnable> changeListeners;
    private final LoggableInputs inputs;

    private final T defaultValue;
    private T prevValue, value;

    protected abstract T getValue(NetworkTableEntry entry, T defaultValue);
    protected abstract void setValue(NetworkTableEntry entry, T value);

    // Despite all the get() and put() methods on LogTable having the same
    // name, they are different methods so we need to abstract them
    protected abstract void toLog(LogTable table, String key, T value);
    protected abstract T fromLog(LogTable table, String key, T defaultValue);

    /**
     * Creates a new entry with a specified path. The path can be split using the '/' character to
     * organize entries into groups.
     *
     * @param path path
     * @param defaultValue value to set if no value is present
     */
    public NTEntry(String path, T defaultValue) {
        this.defaultValue = defaultValue;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("");
        String[] parts = path.split("/");
        for (int i = 0; i < parts.length - 1; i++) {
            table = table.getSubTable(parts[i]);
        }
        entry = table.getEntry(parts[parts.length - 1]);

        if (!entry.exists())
            setValue(entry, defaultValue);
        prevValue = value = getValue(entry, defaultValue);

        changeListeners = new ArrayList<>();
        inputs = new LoggableInputs() {
            @Override
            public void toLog(LogTable table) {
                NTEntry.this.toLog(table, path, value);
            }

            @Override
            public void fromLog(LogTable table) {
                value = NTEntry.this.fromLog(table, path, defaultValue);
            }
        };

        ALL_ENTRIES.add(this);
    }

    @Override
    public T get() {
        return value;
    }

    /**
     * Sets the value stored within the entry. This will call change listeners
     * if the value is different than the current value.
     *
     * @param value value to set
     */
    public void set(T value) {
        this.value = value;
        setValue(entry, value);
    }

    /**
     * Marks this entry to be stored if the robot is turned off. The value is
     * stored on the RoboRIO, so it is lost if the RoboRIO is re-imaged! You
     * can back up the persistent values by copying the
     * {@code /home/lvuser/networktables.json} file from the RoboRIO to
     * somewhere safe.
     *
     * @return this
     * @throws IllegalStateException if called after {@link #cleanPersistent()}
     *      was called
     */
    public NTEntry<T> setPersistent() {
        if (persistentCleaned) {
            throw new IllegalStateException("Persistent values have already "
                    + "been cleaned, so this value would have been deleted! "
                    + "Define persistent values in Constants.");
        }

        entry.setPersistent();

        if (!Objects.equals(defaultValue, get()))
            DriverStation.reportWarning("NT value differs from default: " + entry.getName(), false);

        return this;
    }

    /**
     * Registers a change listener. The function provided will be called
     * whenever the value within the entry changes.
     *
     * @param listener listener to register
     */
    public void onChange(Runnable listener) {
        changeListeners.add(listener);
    }

    private void update() {
        if (!Logger.hasReplaySource()) {
            value = getValue(entry, defaultValue);
        }
        Logger.processInputs(PREFIX, inputs);

        if (!Objects.equals(value, prevValue)) {
            for (Runnable listener : changeListeners) {
                listener.run();
            }
        }
        prevValue = value;
    }
}
