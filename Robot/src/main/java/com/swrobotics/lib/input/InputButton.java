package com.swrobotics.lib.input;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/** Represents a binary input (pressed or not pressed). */
public final class InputButton implements Supplier<Boolean> {
    private final Supplier<Boolean> getter;

    /**
     * Creates a new input button that reads its value from a provided getter
     * function.
     *
     * @param getter value getter
     */
    public InputButton(Supplier<Boolean> getter) {
        this.getter = getter;
    }

    /**
     * @return whether this button is currently pressed
     */
    @Override
    public Boolean get() {
        return getter.get();
    }

    public Trigger trigger() {
        return new Trigger(getter::get);
    }

    /**
     * Adds a function that will be called whenever the button is pressed. This
     * function will be invoked on each periodic where {@link #wasPressed()}
     * returns {@code true}.
     *
     * @param pressedFn function to call
     * @return this
     */
    public InputButton onPressed(Runnable pressedFn) {
        return onPressed(Commands.runOnce(pressedFn));
    }

    /**
     * Adds a command that will be called whenever the button is pressed.
     *
     * @param command command to schedule
     * @return this
     */
    public InputButton onPressed(Command command) {
        trigger().onTrue(command);
        return this;
    }

    /**
     * Adds a function that will be called whenever the button is released.
     *
     * @param releasedFn function to call
     * @return this
     */
    public InputButton onReleased(Runnable releasedFn) {
        return onReleased(Commands.runOnce(releasedFn));
    }

    /**
     * Adds a command that will be called whenever the button is released.
     *
     * @param command command to schedule
     * @return this
     */
    public InputButton onReleased(Command command) {
        trigger().onFalse(command);
        return this;
    }

    /**
     * Adds a command that will be called whenever the button is held down for
     * a specified amount of time.
     *
     * @param command command to schedule
     * @param seconds time the button has to be held
     * @return this
     */
    public InputButton onHeld(Command command, double seconds) {
        trigger().debounce(seconds).onTrue(command);
        return this;
    }

    /**
     * Adds a command that will be called whenever the button is held down for
     * one second.
     *
     * @param command command to schedule
     * @return this
     */
    public InputButton onHeld(Command command) {
        return onHeld(command, 1.0);
    }
}
