package com.swrobotics.robot.control;

import edu.wpi.first.wpilibj.Timer;

/**
 * Acceleration limiter for the drive base. This helps reduce the current draw
 * while accelerating and allows the driver to be more aggressive by pushing
 * the stick all the way to the edge. Deceleration is not limited to allow the
 * driver to stop immediately instead of "coasting" for a little bit.
 */
public final class DriveAccelFilter {
    private final double maxAccel;
    private double currentVelocity;

    // NaN is used here to indicate that the timestamp hasn't been set yet
    private double prevTimestamp = Double.NaN;

    /**
     * @param maxAccel maximum acceleration per second
     */
    public DriveAccelFilter(double maxAccel) {
        if (maxAccel <= 0)
            throw new IllegalArgumentException("maxAccel must be positive");
        this.maxAccel = maxAccel;

        currentVelocity = 0;
    }

    /**
     * Calculates the velocity that should be sent to the drive base.
     *
     * @param targetVelocity driver's desired velocity
     * @return filtered velocity that respects the maximum acceleration limit
     */
    public double calculate(double targetVelocity) {
        // Allow instant deceleration
        if (Math.abs(currentVelocity) > Math.abs(targetVelocity)) {
            currentVelocity = targetVelocity;
            return targetVelocity;
        }

        // Find how much time has passed since the last call to this method
        double time = Timer.getTimestamp();
        double delta = 0;
        if (!Double.isNaN(prevTimestamp))
            delta = time - prevTimestamp;
        prevTimestamp = time;

        double error = targetVelocity - currentVelocity;
        double toChange = Math.min(maxAccel * delta, Math.abs(error));

        currentVelocity += Math.copySign(toChange, error);

        return currentVelocity;
    }

    /**
     * Resets the velocity to the specified velocity. The velocity is set
     * immediately, ignoring the acceleration limit.
     *
     * @param currentVelocity current velocity of the robot
     */
    public void reset(double currentVelocity) {
        this.currentVelocity = currentVelocity;
    }
}
