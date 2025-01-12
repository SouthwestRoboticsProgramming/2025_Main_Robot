package com.swrobotics.lib.input;

import com.swrobotics.lib.utils.MathUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

/**
 * Represents an analog input. Most axes will have an output from either -1 to 1, or 0 to 1. Note
 * that most analog inputs have a slight "drift", where the value will not be exactly zero when not
 * touching the input.
 */
public final class InputAxis implements Supplier<Double> {
    private final Supplier<Double> getter;
    private final double deadband;

    /**
     * Creates a new input axis that reads its value from a given getter function.
     *
     * @param getter value getter
     */
    public InputAxis(Supplier<Double> getter, double deadband) {
        this.getter = getter;
        this.deadband = deadband;
    }

    /**
     * Gets the current position of this axis with deadband applied.
     *
     * @return deadbanded value
     */
    @Override
    public Double get() {
        return MathUtil.deadband(getter.get(), deadband);
    }

    /**
     * Gets the current position of this axis without deadband applied.
     *
     * @return raw value
     */
    public double getRaw() {
        return getter.get();
    }

    /**
     * @param range range to check, centered around 0
     * @return whether the current position is outside of the range
     */
    public boolean isOutside(double range) {
        return Math.abs(getter.get()) > range;
    }

    /**
     * @param range range to check, centered around 0
     * @return trigger of whether the current position is outside of the range
     */
    public Trigger triggerOutside(double range) {
        return new Trigger(() -> isOutside(range));
    }
}
