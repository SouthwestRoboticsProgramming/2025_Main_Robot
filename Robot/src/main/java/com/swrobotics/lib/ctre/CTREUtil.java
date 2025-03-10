package com.swrobotics.lib.ctre;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public final class CTREUtil {
    private static final int kMaxAttempts = 10;

    public static void retryUntilOk(ParentDevice device, Supplier<StatusCode> fn) {
        int attemptsLeft = kMaxAttempts;
        StatusCode status;
        do {
            status = fn.get();
        } while (!status.isOK() && attemptsLeft-- > 0);

        String deviceDesc = device.getClass().getSimpleName()
                + " (" + (device.getNetwork().isEmpty() ? "rio" : device.getNetwork())
                + " " + device.getDeviceID() + ")";
        if (status.isError()) {
            DriverStation.reportError("Failure on " + deviceDesc + ": " + status.getDescription(), true);
        } else if (status.isWarning()) {
            DriverStation.reportWarning("Warning on " + deviceDesc + ": " + status.getDescription(), true);
        }
    }
}
