package com.swrobotics.lib.utils;

/**
 * Helper to build a double array sequentially.
 */
public final class DoubleOutput {
    private final double[] array;
    private int index;

    /**
     * @param capacity number of elements in the array
     */
    public DoubleOutput(int capacity) {
        array = new double[capacity];
        index = 0;
    }

    /**
     * Adds a value to the next slot in the array.
     *
     * @param value value to add
     * @throws ArrayIndexOutOfBoundsException if the array is full
     */
    public void add(double value) {
        array[index++] = value;
    }

    /**
     * @return the array
     */
    public double[] toArray() {
        return array;
    }
}
