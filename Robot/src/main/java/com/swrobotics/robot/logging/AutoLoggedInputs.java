package com.swrobotics.robot.logging;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

/**
 * Helper for input classes to automatically log their fields. This will log
 * all fields that are public and not static or final. Only fields of the types
 * {@code int}, {@code long}, {@code float}, {@code double}, {@code String},
 * {@code boolean}, or single-dimensional arrays of those types, as well as
 * {@code byte[]} are supported. If other types are needed, you will need to
 * implement {@code LoggableInputs} manually.
 */
public abstract class AutoLoggedInputs implements LoggableInputs {
    private enum LogType {
        Raw,
        Boolean,
        Integer,
        Long,
        Float,
        Double,
        String,
        BooleanArray,
        IntegerArray,
        LongArray,
        FloatArray,
        DoubleArray,
        StringArray
    }
    
    private static final class LogEntry {
        private final LogType type;
        private final Field field;
        private final Object defVal;

        public LogEntry(AutoLoggedInputs in, Field field) {
            this.field = field;

            Class<?> clazz = field.getType();
            if (clazz == byte[].class)
                type = LogType.Raw;
            else if (clazz == boolean.class || clazz == Boolean.class)
                type = LogType.Boolean;
            else if (clazz == int.class || clazz == Integer.class)
                type = LogType.Integer;
            else if (clazz == long.class || clazz == Long.class)
                type = LogType.Long;
            else if (clazz == float.class || clazz == Float.class)
                type = LogType.Float;
            else if (clazz == double.class || clazz == Double.class)
                type = LogType.Double;
            else if (clazz == String.class)
                type = LogType.String;
            else if (clazz == boolean[].class)
                type = LogType.BooleanArray;
            else if (clazz == int[].class)
                type = LogType.IntegerArray;
            else if (clazz == long[].class)
                type = LogType.LongArray;
            else if (clazz == float[].class)
                type = LogType.FloatArray;
            else if (clazz == double[].class)
                type = LogType.DoubleArray;
            else if (clazz == String[].class)
                type = LogType.StringArray;
            else
                throw new AssertionError("Field is not a loggable type: " + field);

            try {
                defVal = field.get(in);
            } catch (ReflectiveOperationException e) {
                throw new RuntimeException("Failed to get default value", e);
            }
        }
    }

    private final LogEntry[] entries;

    public AutoLoggedInputs() {
        // Index the available fields in this class beforehand. This is
        // somewhat slow, so we do it in the constructor, which should
        // only be called once at the start of the robot program.
        List<LogEntry> entries = new ArrayList<>();
        for (Field field : getClass().getFields()) {
            // Skip static and final fields
            int mods = field.getModifiers();
            if (Modifier.isFinal(mods) || Modifier.isStatic(mods))
                continue;

            entries.add(new LogEntry(this, field));
        }
        this.entries = entries.toArray(new LogEntry[0]);
    }

    private int[] toInts(long[] longs) {
        int[] ints = new int[longs.length];
        for (int i = 0; i < ints.length; i++)
            ints[i] = (int) longs[i];
        return ints;
    }

    private long[] toLongs(int[] ints) {
        long[] longs = new long[ints.length];
        for (int i = 0; i < ints.length; i++)
            longs[i] = ints[i];
        return longs;
    }
    
    @Override
    public void toLog(LogTable table) {
        for (LogEntry entry : entries) {
            String name = entry.field.getName();
            Object val;
            try {
                val = entry.field.get(this);
            } catch (ReflectiveOperationException e) {
                System.err.println("Failed to log " + entry.field);
                e.printStackTrace();
                return;
            }

            switch (entry.type) {
                case Raw -> table.put(name, (byte[]) val);
                case Boolean -> table.put(name, (boolean) val);
                case Integer -> table.put(name, (int) val);
                case Long -> table.put(name, (long) val);
                case Float -> table.put(name, (float) val);
                case Double -> table.put(name, (double) val);
                case String -> table.put(name, (String) val);
                case BooleanArray -> table.put(name, (boolean[]) val);
                case IntegerArray -> table.put(name, toLongs((int[]) val));
                case LongArray -> table.put(name, (long[]) val);
                case DoubleArray -> table.put(name, (double[]) val);
                case StringArray -> table.put(name, (String[]) val);
                case FloatArray -> table.put(name, (float[]) val);
            }
        }
    }

    @Override
    public void fromLog(LogTable table) {
        for (LogEntry entry : entries) {
            String name = entry.field.getName();
            Object def = entry.defVal;
            Object val;
            switch (entry.type) {
                case Raw:
                    val = table.get(name, (byte[]) def);
                    break;
                case Boolean:
                    val = table.get(name, (Boolean) def);
                    break;
                case Integer:
                    val = table.get(name, (Integer) def);
                    break;
                case Long:
                    val = table.get(name, (Long) def);
                    break;
                case Double:
                    val = table.get(name, (Double) def);
                    break;
                case String:
                    val = table.get(name, (String) def);
                    break;
                case BooleanArray:
                    val = table.get(name, (boolean[]) def);
                    break;
                case IntegerArray:
                    val = toInts(table.get(name, toLongs((int[]) def)));
                    break;
                case LongArray:
                    val = table.get(name, (long[]) def);
                    break;
                case DoubleArray:
                    val = table.get(name, (double[]) def);
                    break;
                case StringArray:
                    val = table.get(name, (String[]) def);
                    break;
                default:
                    return;
            }

            try {
                entry.field.set(this, val);
            } catch (ReflectiveOperationException e) {
                System.err.println("Failed to set field " + entry.field);
                e.printStackTrace();
            }
        }
    }
}
