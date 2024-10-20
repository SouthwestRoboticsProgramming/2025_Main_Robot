package com.swrobotics.lib.input;

import com.swrobotics.lib.utils.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.util.Arrays;
import java.util.List;

/** Represents an Xbox controller attached to the driver station. */
public final class XboxController extends InputSource {
    private final edu.wpi.first.wpilibj.XboxController xbox;
    private final List<InputElement> elements;
    private final double deadband;

    // All the inputs on the Xbox controller
    public final InputButton a, b, x, y;
    public final InputButton back, start;
    public final InputButton leftStickButton, rightStickButton;
    public final InputButton leftBumper, rightBumper;
    public final InputAxis leftStickX, leftStickY;
    public final InputAxis rightStickX, rightStickY;
    public final InputAxis leftTrigger, rightTrigger;
    public final InputDpad dpad;

    /**
     * Creates a new Xbox controller connected to a given port.
     *
     * @param port port the controller is on
     * @param deadband deadband range to apply to all analog inputs. If you
     *                 want the raw, undeadbanded input you can use
     *                 {@link InputAxis#getRaw()}
     */
    public XboxController(int port, double deadband) {
        xbox = new edu.wpi.first.wpilibj.XboxController(port);
        this.deadband = deadband;

        a = new InputButton(xbox::getAButton);
        b = new InputButton(xbox::getBButton);
        x = new InputButton(xbox::getXButton);
        y = new InputButton(xbox::getYButton);
        back = new InputButton(xbox::getBackButton);
        start = new InputButton(xbox::getStartButton);
        leftStickButton = new InputButton(xbox::getLeftStickButton);
        rightStickButton = new InputButton(xbox::getRightStickButton);
        leftBumper = new InputButton(xbox::getLeftBumperButton);
        rightBumper = new InputButton(xbox::getRightBumperButton);

        leftStickX = new InputAxis(xbox::getLeftX, deadband);
        leftStickY = new InputAxis(xbox::getLeftY, deadband);
        rightStickX = new InputAxis(xbox::getRightX, deadband);
        rightStickY = new InputAxis(xbox::getRightY, deadband);
        leftTrigger = new InputAxis(xbox::getLeftTriggerAxis, deadband);
        rightTrigger = new InputAxis(xbox::getRightTriggerAxis, deadband);

        dpad = new InputDpad(xbox::getPOV);

        elements =
                Arrays.asList(
                        a,
                        b,
                        x,
                        y,
                        back,
                        start,
                        leftStickButton,
                        rightStickButton,
                        leftBumper,
                        rightBumper,
                        leftStickX,
                        leftStickY,
                        rightStickX,
                        rightStickY,
                        leftTrigger,
                        rightTrigger,
                        dpad);
    }

    /**
     * Sets the rumble feedback output of the controller.
     *
     * @param amount percentage of rumble from 0 to 1
     */
    public void setRumble(double amount) {
        xbox.setRumble(GenericHID.RumbleType.kLeftRumble, amount);
        xbox.setRumble(GenericHID.RumbleType.kRightRumble, amount);
    }

    /**
     * Sets the rumble feedback output of the controller.
     *
     * @param type type of rumble to set
     * @param amount percentage of rumble from 0 to 1
     */
    public void setRumble(RumbleType type, double amount) {
        xbox.setRumble(type, amount);
    }

    /**
     * @return left analog stick input as a vector
     */
    public Translation2d getLeftStick() {
        return MathUtil.deadband2d(leftStickX.getRaw(), leftStickY.getRaw(), deadband);
    }

    /**
     * @return right analog stick input as a vector
     */
    public Translation2d getRightStick() {
        return MathUtil.deadband2d(rightStickX.getRaw(), rightStickY.getRaw(), deadband);
    }

    @Override
    protected Iterable<InputElement> getElements() {
        return elements;
    }
}
