package com.swrobotics.robot.subsystems.tunnel;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class TunnelSubsystem extends SubsystemBase {
    public enum State {
        RECEIVE_FROM_INTAKE,
        FEED_TO_OUTTAKE,
        EJECT
    }

    private final TunnelIO io;
    private final TunnelIO.Inputs inputs;

    private State state;

    public TunnelSubsystem() {
        if (RobotBase.isReal()) {
            io = new TunnelIOReal();
        } else {
            io = new TunnelIOSim();
        }
        inputs = new TunnelIO.Inputs();

        state = State.RECEIVE_FROM_INTAKE;
    }

    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tunnel", inputs);

        double volts = switch (state) {
            case RECEIVE_FROM_INTAKE -> inputs.hasCoral
                    ? 0.0 // Hold coral
                    : Constants.kTunnelReceiveVoltage.get();
            case FEED_TO_OUTTAKE -> Constants.kTunnelFeedVoltage.get();
            case EJECT -> Constants.kTunnelEjectVoltage.get();
        };

        io.setVoltage(volts);
    }
}
