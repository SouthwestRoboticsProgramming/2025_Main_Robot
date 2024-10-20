package com.swrobotics.robot.subsystems.lights;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Look into new WPILib LED strip patterns API
public final class LightsSubsystem extends SubsystemBase {
    private final RobotContainer robot;
    private final AddressableLED leds;
    private final AddressableLEDBuffer data;

    private final Debouncer batteryLowDebounce;
    private final PrideSequencer prideSequencer;

    private Color commandRequest = null;

    public LightsSubsystem(RobotContainer robot) {
        this.robot = robot;
        leds = new AddressableLED(IOAllocation.RIO.PWM_LEDS);
        leds.setLength(Constants.kLedStripLength);

        data = new AddressableLEDBuffer(Constants.kLedStripLength);
        leds.setData(data);

        leds.start();

        batteryLowDebounce = new Debouncer(10);
        prideSequencer = new PrideSequencer();
    }

    private void showOverheating() {
        // Flashing red lights
        applySolid(Timer.getFPGATimestamp() % 0.4 > 0.2 ? Color.kRed : Color.kBlack);
    }

    private void showLowBattery() {
        // Flashing orange lights
        applySolid(Timer.getFPGATimestamp() % 0.4 > 0.2 ? Color.kOrange : Color.kBlack);
    }

    private void showAutoDriving() {
        // Rainbow
        applyStripes(5f,
                new Stripe(Color.kRed, 1),
                new Stripe(Color.kOrange, 1),
                new Stripe(Color.kYellow, 1),
                new Stripe(Color.kGreen, 1),
                new Stripe(Color.kBlue, 1),
                new Stripe(Color.kPurple, 1));
    }

    private void showIdle() {
        // Lights off
        applySolid(Color.kBlack);
    }

    @Override
    public void periodic() {
        boolean overCurrent = robot.pdp.getInputs().pdpTotalCurrent > Constants.kLedCurrentShutoffThreshold;
        boolean overheating = robot.motorTracker.isOverheating();
        boolean batteryLow = RobotController.getBatteryVoltage() < Constants.kLowBatteryThreshold;

        if (overCurrent) {
            applySolid(Color.kBlack); // Black means LEDs off
        } else if (overheating) {
            showOverheating();
        } else if (batteryLowDebounce.calculate(batteryLow)) {
            showLowBattery();
        } else if (commandRequest != null) {
            applySolid(commandRequest);
        } else if (DriverStation.isDisabled()) {
            prideSequencer.apply(this);
        } else if (robot.drive.getLastSelectedPriority() == SwerveDriveSubsystem.Priority.AUTO) {
            showAutoDriving();
        } else {
            showIdle();
        }

        commandRequest = null;
    }

    /** Sets all the LEDs to the same color */
    private void applySolid(Color color) {
        for (int i = 0; i < Constants.kLedStripLength; i++) {
            data.setLED(i, color);
        }
        leds.setData(data);
    }

    public static final record Stripe(Color color, float weight) {}

    // Blends between two colors
    private Color interpolate(Color a, Color b, float percent) {
        return new Color(
                MathUtil.lerp(a.red, b.red, percent),
                MathUtil.lerp(a.green, b.green, percent),
                MathUtil.lerp(a.blue, b.blue, percent)
        );
    }

    // Scroll speed is seconds per full pass through the pattern
    public void applyStripes(float scrollSpeed, Stripe... stripes) {
        float scroll;
        Color wrapColor;
        if (scrollSpeed == 0) {
            scroll = 0;
            wrapColor = stripes[stripes.length - 1].color;
        } else {
            scroll = (float) (Timer.getFPGATimestamp() / scrollSpeed) % 1;
            wrapColor = stripes[0].color;
        }

        float totalWeight = 0;
        for (Stripe stripe : stripes)
            totalWeight += stripe.weight;

        float weightPerPixel = totalWeight / Constants.kLedStripLength;

        float position = (1 - scroll) * totalWeight;
        for (int i = 0; i < Constants.kLedStripLength; i++) {
            position += weightPerPixel;
            position %= totalWeight;

            Color color = null;
            Color nextColor = wrapColor;
            float acc = 0;
            for (Stripe stripe : stripes) {
                float end = acc + stripe.weight;
                if (end > position) {
                    if (color != null) {
                        nextColor = stripe.color;
                        break;
                    }
                    color = stripe.color;
                }
                acc = end;
            }

            // Shouldn't happen, but fallback just in case
            if (color == null)
                color = Color.kBlack;

            float diff = acc - position;
            float pixelsToEnd = diff / weightPerPixel;
            if (pixelsToEnd > 1)
                data.setLED(i, color);
            else
                data.setLED(i, interpolate(color, nextColor, 1 - pixelsToEnd));
        }

        leds.setData(data);
    }

    public void disabledInit() {
        prideSequencer.reset();
    }

    /**
     * Called by commands in {@link com.swrobotics.robot.commands.LightCommands}.
     * Should not be called manually.
     *
     * @param color color to set
     */
    public void setCommandRequest(Color color) {
        commandRequest = color;
    }
}
