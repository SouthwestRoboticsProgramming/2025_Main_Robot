package com.swrobotics.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.CTREUtil;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

public class CoralOuttakeIOReal implements CoralOuttakeIO {
    private final TalonFX motor;

    private final VoltageOut voltageControl;

    public CoralOuttakeIOReal() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor = IOAllocation.CAN.kOuttakeMotor.createTalonFX();
        CTREUtil.retryUntilOk(motor, () -> motor.getConfigurator().apply(config));

        MotorTrackerSubsystem.getInstance().addMotor("Outtake", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        voltageControl = new VoltageOut(0)
            .withEnableFOC(true);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.voltage = 0;
        inputs.hasPiece = false;
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageControl.withOutput(voltage));
    }
    
}
