package com.swrobotics.robot.subsystems.swerve.io;

// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.SPI;

/**
 * GyroIO implementation for the NavX2 gyroscope in the RoboRIO MXP port.
 */
public final class NavXGyroIO implements GyroIO {
    // private final AHRS navx;

    public NavXGyroIO() {
        throw new RuntimeException("Uncomment these once Studica fixes the NavX vendordep");

        // Explicitly set the NavX update rate, workaround for NavX vendordep issue
        // See https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487
        // byte updateRate = (byte) 50;
        // navx = new AHRS(SPI.Port.kMXP, updateRate);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        // inputs.connected = navx.isConnected();
        // if (inputs.connected)
        //     inputs.yaw = navx.getRotation2d().getRadians();
    }
}
