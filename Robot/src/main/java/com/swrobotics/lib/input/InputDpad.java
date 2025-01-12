package com.swrobotics.lib.input;

import com.swrobotics.lib.utils.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Supplier;

/**
 * Represents a D-pad (POV) input on a controller. A D-pad can act as four independent buttons, two
 * axes, or an angle input.
 */
public final class InputDpad {
    private final Supplier<Integer> getter;
    public final InputButton up, down, left, right;
    public final InputAxis vertical, horizontal;

    /**
     * Creates a new D-pad input that reads its value from a provided getter function. The getter
     * should return a clockwise angle in degrees, with up being zero, and return -1 when no buttons
     * are pressed, as the WPILib POV input does.
     *
     * @param getter value getter
     */
    public InputDpad(Supplier<Integer> getter) {
        this.getter = getter;

        up = new InputButton(() -> isDirectionDown(0, 45, 315));
        down = new InputButton(() -> isDirectionDown(135, 180, 225));
        left = new InputButton(() -> isDirectionDown(225, 270, 315));
        right = new InputButton(() -> isDirectionDown(45, 90, 135));

        vertical = new InputAxis(() -> up.get() ? 1.0 : (down.get() ? -1.0 : 0.0), 0);
        horizontal =
                new InputAxis(() -> right.get() ? 1.0 : (left.get() ? -1.0 : 0.0), 0);
    }

    private boolean isDirectionDown(int angle1, int angle2, int angle3) {
        int angleDeg = getter.get();
        return angleDeg == angle1 || angleDeg == angle2 || angleDeg == angle3;
    }

    /**
     * Gets whether any button on the D-pad is pressed.
     *
     * @return pressed
     */
    public boolean isPressed() {
        return getter.get() >= 0;
    }

    /**
     * Gets the angle that the D-pad is currently pressed. Zero represents
     * right.
     *
     * @return angle, ccw positive
     */
    public Rotation2d getAngle() {
        int angle = getter.get();
        if (angle < 0)
            angle = 0;

        return Rotation2d.fromDegrees(MathUtil.wrap(angle + 90, -180, 180));
    }
}
