package com.swrobotics.robot;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.geometry.Translation2d;

public final class RobotPhysics {
    public static double getCenterOfGravityHeight(double elevatorHeightPct) {
        return MathUtil.lerp(
                Constants.kCOGHeightWithElevatorDown,
                Constants.kCOGHeightWithElevatorUp,
                elevatorHeightPct
        );
    }

    public static double getMaxAccelerationWithoutTipping(Translation2d robotRelDirection, double elevatorHeightPct) {
        Translation2d normDirection = robotRelDirection.div(robotRelDirection.getNorm());

        // Find point along edge of wheelbase
        double halfSpacingX = Constants.kDriveWheelSpacingX;
        double halfSpacingY = Constants.kDriveWheelSpacingY;
        double frameRelX = normDirection.getX() / halfSpacingX;
        double frameRelY = normDirection.getY() / halfSpacingY;
        double max = Math.max(Math.abs(frameRelX), Math.abs(frameRelY));
        frameRelX /= max;
        frameRelY /= max;
        double edgeX = frameRelX * halfSpacingX;
        double edgeY = frameRelY * halfSpacingY;

        double w = Math.hypot(edgeX, edgeY);
        double h = getCenterOfGravityHeight(elevatorHeightPct);

        // When about to tip, net torque is 0, and all normal force upward is
        // from the back wheel. Let θ be the angle from horizontal of the
        // vector from the wheel to the center of gravity. Then
        //               Στ = τ_accel - τ_norm
        //                0 = F_accel * sin(θ) - F_norm * cos(θ)
        //                0 = m * a * sin(θ) - m * g * cos(θ)
        //   m * a * sin(θ) = m * g * cos(θ)
        //       a * sin(θ) = g * cos(θ)
        //                a = g * cos(θ) / sin(θ)
        //                a = g * cot(θ)
        //                a = g * w / h
        return MathUtil.G_ACCEL * w / h;
    }
}
