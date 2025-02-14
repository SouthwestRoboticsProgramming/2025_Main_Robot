package com.swrobotics.robot.subsystems.tunnel;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public final class TunnelIOReal implements TunnelIO {
    private final TalonFX motor;
    private final DigitalInput beamBreak;

    private final VoltageOut voltageControl;

    public TunnelIOReal() {
        motor = IOAllocation.CAN.kTunnelMotor.createTalonFX();
        beamBreak = new DigitalInput(IOAllocation.RIO.kDIO_TunnelBeamBreak);

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME
        config.apply(motor);

        MotorTrackerSubsystem.getInstance().addMotor("Tunnel", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        voltageControl = new VoltageOut(0)
                .withEnableFOC(true);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.hasCoral = !beamBreak.get();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }
}
