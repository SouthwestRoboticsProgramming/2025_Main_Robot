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
        // navx = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        // inputs.connected = navx.isConnected();
        // if (inputs.connected)
        //     inputs.yaw = navx.getRotation2d().getRadians();
    }
}
