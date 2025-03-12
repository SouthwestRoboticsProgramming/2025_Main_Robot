package com.swrobotics.robot.control;

import com.swrobotics.lib.utils.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.ToDoubleFunction;

public final class DriveAccelFilter2d {
    private final ToDoubleFunction<Translation2d> maxAccelGetter;

    private double prevTimestamp;
    private Translation2d currentVelocity;

    public DriveAccelFilter2d(ToDoubleFunction<Translation2d> maxAccelGetter) {
        this.maxAccelGetter = maxAccelGetter;
        prevTimestamp = Timer.getTimestamp();
        currentVelocity = Translation2d.kZero;
    }

    public Translation2d calculate(Translation2d desiredVelocity) {
        double timestamp = Timer.getTimestamp();
        double dt = timestamp - prevTimestamp;
        prevTimestamp = timestamp;

//        System.out.println("Current velocity: " + currentVelocity);

        Translation2d desiredDeltaV = desiredVelocity.minus(currentVelocity);
        double desiredAccelMag = desiredDeltaV.getNorm() / dt;
        if (desiredAccelMag < MathUtil.EPSILON) {
            currentVelocity = desiredVelocity;
            return desiredVelocity;
        }

        double maxAccelMag = maxAccelGetter.applyAsDouble(desiredDeltaV);
//        System.out.println("Max acceleration: " + maxAccelMag + " m/s^2");
        double appliedAccelMag = Math.min(desiredAccelMag, maxAccelMag);

        currentVelocity = currentVelocity.plus(desiredDeltaV.div(desiredDeltaV.getNorm()).times(appliedAccelMag * dt));
        return currentVelocity;
    }

    public void reset(Translation2d currentVelocity) {
        this.currentVelocity = currentVelocity;
    }
}
