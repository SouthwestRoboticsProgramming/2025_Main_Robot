package com.swrobotics.robot.subsystems.tunnel;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class TunnelSubsystem extends SubsystemBase {
    private final TunnelIO io;
    private final TunnelIO.Inputs inputs;

    public TunnelSubsystem() {
        if (RobotBase.isReal()) {
            io = new TunnelIOReal();
        } else {
            io = new TunnelIOSim();
        }
        inputs = new TunnelIO.Inputs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tunnel", inputs);
    }
}
