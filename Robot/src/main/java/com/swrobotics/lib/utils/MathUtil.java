package com.swrobotics.lib.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/** Contains various mathematical utilities not provided by Java's built-in libraries. */
public final class MathUtil {
    /** The mathematical constant tau. This is defined as exactly twice the value of pi. */
    public static final double TAU = Math.PI * 2;

    /** A very small, effectively zero value to account for floating-point imprecision. */
    public static final double EPSILON = 0.0001;

    /** Constant containing exactly half the value of pi. */
    public static final double HALF_PI = Math.PI / 2;

    /** Gravitational acceleration on Earth in meters/sec^2. */
    public static final double G_ACCEL = 9.80665;

    /**
     * Clamps a value within a specified range. If the value is below the minimum, it will be
     * clamped up to the minimum, and if it is above the maximum, it will be clamped down to the
     * maximum.
     *
     * @param val input value
     * @param min minimum output
     * @param max maximum output
     * @return clamped value
     * @throws IllegalArgumentException if min is greater than max
     */
    public static double clamp(double val, double min, double max) {
        checkValidRange(min, max);
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }

    /**
     * Linearly interpolates between two values based on a given percentage from 0 to 1. The output
     * can exceed the specified range if the input goes out of this range.
     *
     * @param min minimum output
     * @param max maximum output
     * @param factor percentage
     * @return interpolated value
     */
    public static double lerp(double min, double max, double factor) {
        return min + (max - min) * factor;
    }

    /**
     * Gets the percentage of the way from the minimum to the maximum the value is from 0 to 1. If
     * the input value is outside of the specified range, the percentage can be lower than 0 or
     * higher than 1.
     *
     * @param val input value
     * @param min range minimum
     * @param max range maximum
     * @return percentage
     */
    public static double percent(double val, double min, double max) {
        double delta = max - min;
        if (delta == 0)
            return 0;
        return (val - min) / delta;
    }

    /**
     * Maps a value from one range to another. The output value can exceed the target range if the
     * input value exceeds the source range.
     *
     * @param val input value
     * @param inMin source minimum
     * @param inMax source maximum
     * @param outMin target minimum
     * @param outMax target maximum
     * @return mapped value
     */
    public static double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return lerp(outMin, outMax, percent(val, inMin, inMax));
    }

    /**
     * Returns the floor modulus of the given values. This is needed as Java Math only contains
     * implementations for {@code int}s and {@code long}s.
     *
     * @param x dividend
     * @param y divisor
     * @return floor modulus
     * @see java.lang.Math#floorMod
     */
    public static double floorMod(double x, double y) {
        if (y == 0) throw new ArithmeticException("Divide by zero");

        return x - Math.floor(x / y) * y;
    }

    /**
     * Wraps a value within specified bounds. If the value exceeds the bounds in either direction,
     * it will wrap around to the other side.
     *
     * @param val value to wrap
     * @param min minimum bound
     * @param max maximum bound
     * @return wrapped value
     */
    public static double wrap(double val, double min, double max) {
        checkValidRange(min, max);
        return floorMod(val - min, max - min) + min;
    }

    public static double deadband(double val, double band) {
        if (band < 0)
            throw new IllegalArgumentException("Deadband must be greater than or equal to zero");

        double abs = Math.abs(val);
        if (abs < band) return 0;
        return Math.copySign(map(abs, band, 1, 0, 1), val);
    }

    public static Translation2d deadband2d(double x, double y, double band) {
        return deadband2d(new Translation2d(x, y), band);
    }

    public static Translation2d deadband2d(Translation2d val, double band) {
        double rawNorm = val.getNorm();
        if (rawNorm == 0) {
            return new Translation2d(0, 0);
        }

        double mag = deadband(rawNorm, band);
        double outX = val.getX() / rawNorm * mag;
        double outY = val.getY() / rawNorm * mag;

        return new Translation2d(outX, outY);
    }

    // Checks if a given range is valid (i.e. max > min)
    private static void checkValidRange(double min, double max) {
        if (min >= max)
            throw new IllegalArgumentException("Minimum must not be greater than maximum");
    }

    public static double square(double in) {
        return in * in;
    }

    public static double powerWithSign(double value, double power) {
        double powered = Math.pow(Math.abs(value), power);
        return Math.copySign(powered, value);
    }

    public static boolean fuzzyEquals(double a, double b) {
        return Math.abs(a - b) <= EPSILON;
    }

    public static double signedPercentError(double measured, double target) {
        return (measured - target) / target;
    }

    public static double percentError(double measured, double target) {
        return Math.abs(signedPercentError(measured, target));
    }

    public static double absDiffRad(double a, double b) {
        double wrapA = wrap(a, 0, TAU);
        double wrapB = wrap(b, 0, TAU);

        double diff = wrapB - wrapA;
        double direct = Math.abs(diff);
        double wrapped = TAU - direct;

        return Math.min(direct, wrapped);
    }

    public static Twist2d multiplyTwist(Twist2d twist, double factor) {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }
}
