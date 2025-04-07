package com.swrobotics.robot.commands;

import com.swrobotics.robot.subsystems.lights.LightsSubsystem;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;

public final class LightCommands {
    /**
     * @param lights LightsSubsystem instance
     * @param blinks number of times to blink
     * @param color color to blink the lights
     * @param totalTimeSeconds total time in seconds for the blink sequence
     * @param onPercent duty cycle of the blinking from 0 to 1, higher means on
     *                  more of the time
     * @return blink command
     */
    public static Command blink(LightsSubsystem lights, int blinks, Color color, double totalTimeSeconds, double onPercent) {
        if (onPercent < 0 || onPercent > 1)
            throw new IllegalArgumentException("onPercent must be between 0 and 1");

        if (onPercent == 0)
            return Commands.none();
        if (onPercent == 1)
            return setColorForTime(lights, color, totalTimeSeconds);

        double onTime = totalTimeSeconds / blinks * onPercent;
        double offTime = totalTimeSeconds / blinks - onTime;

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        for (int i = 0; i < blinks; i++) {
            sequence.addCommands(
                    setColorForTime(lights, color, onTime),
                    new WaitCommand(offTime)
            );
        }

        return sequence;
    }

    /**
     * Blinks the lights for 0.5 seconds with 50% duty cycle.
     *
     * @param lights LightsSubsystem instance
     * @param blinks number of times to blink
     * @param color color to blink the lights
     * @return blink command
     */
    public static Command blink(LightsSubsystem lights, int blinks, Color color) {
        return blink(lights, blinks, color, 0.5, 0.5);
    }

    /**
     * Blinks the lights 3 times in 0.5 seconds with 50% duty cycle.
     *
     * @param lights LightsSubsystem instance
     * @param color color to blink the lights
     * @return blink command
     */
    public static Command blink(LightsSubsystem lights, Color color) {
        return blink(lights, 3, color);
    }

    /**
     * @param lights LightsSubsystem instance
     * @param color color to set the lights
     * @param seconds how long to set that color
     * @return set color command
     */
    public static Command setColorForTime(LightsSubsystem lights, Color color, double seconds) {
        return Commands.run(() -> lights.setCommandRequest(color)).withTimeout(seconds);
    }

    public static Command showColor(LightsSubsystem lights, Color color) {
        return Commands.run(() -> lights.setCommandRequest(color));
    }

    public static Command showSnappingToAngle(LightsSubsystem lights) {
        return showColor(lights, Color.kDarkCyan);
    }

    public static Command showSnappingToPose(LightsSubsystem lights) {
        return showColor(lights, Color.kWhite)
                .alongWith(Commands.run(() -> lights.setFullBright(true)));
    }
}
