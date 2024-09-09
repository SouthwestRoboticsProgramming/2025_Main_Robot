package com.swrobotics.lib.input;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/** Represents a binary input (pressed or not pressed). */
public final class InputButton implements InputElement {
    private final Supplier<Boolean> getter;
    private final List<Runnable> onRising, onFalling;
    private boolean pressed, wasPressed;

    /**
     * Creates a new input button that reads its value from a provided getter function.
     *
     * @param getter value getter
     */
    public InputButton(Supplier<Boolean> getter) {
        this.getter = getter;

        onRising = new ArrayList<>();
        onFalling = new ArrayList<>();

        pressed = wasPressed = getter.get();
    }

    /**
     * Gets whether this button is currently pressed.
     *
     * @return pressed
     */
    public boolean isPressed() {
        return pressed;
    }

    /**
     * Gets whether this button was just pressed during this periodic cycle. This is when the button
     * was pressed the previous periodic, but is now pressed.
     *
     * @return if button was just released
     */
    public boolean isRising() {
        return pressed && !wasPressed;
    }

    /**
     * Gets whether this button was just released during this periodic cycle. This is when the
     * button was not pressed the previous periodic, but is now pressed.
     *
     * @return if button was just pressed
     */
    public boolean isFalling() {
        return !pressed && wasPressed;
    }

    /**
     * Adds a function that will be called whenever the button is pressed. This function will be
     * invoked on each periodic where {@link #isRising()} returns {@code true}.
     *
     * @param risingFn function to call
     */
    public void onRising(Runnable risingFn) {
        onRising.add(risingFn);
    }

    public void onRising(Command command) {
        onRising(() -> CommandScheduler.getInstance().schedule(command));
    }

    /**
     * Adds a function that will be called whenever the button is release. This function will be
     * invoked on each periodic where {@link #isFalling()} returns {@code true}.
     *
     * @param fallingFn function to call
     */
    public void onFalling(Runnable fallingFn) {
        onFalling.add(fallingFn);
    }

    public void onFalling(Command command) {
        onFalling(() -> CommandScheduler.getInstance().schedule(command));
    }

    public void onHeld(Command command, double seconds) {
        new Trigger(this::isPressed).debounce(seconds).onTrue(command);
    }

    public void onHeld(Command command) {
        onHeld(command, 1.0);
    }

    @Override
    public void update() {
        wasPressed = pressed;
        pressed = getter.get();
        if (isRising()) {
            for (Runnable risingFn : onRising) {
                risingFn.run();
            }
        }
        if (isFalling()) {
            for (Runnable fallingFn : onFalling) {
                fallingFn.run();
            }
        }
    }
}
