package com.swrobotics.robot.subsystems.outtake.algae;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.atomic.AtomicInteger;

public class AlgaeOuttakeIOReal implements AlgaeOuttakeIO {
    private final TalonFX motor;
    private final DigitalInput beamBreak;

    private final AtomicInteger successfulDaqs, failedDaqs;

    private final VoltageOut voltageControl;

    public AlgaeOuttakeIOReal() {
        motor = IOAllocation.CAN.kAlgaeOuttakeMotor.createTalonFX();
        beamBreak = new DigitalInput(IOAllocation.RIO.kDIO_AlgaeOuttakeBeamBreak);

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.apply(motor);

        MotorTrackerSubsystem.getInstance().addMotor("Algae Outtake", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        // Better solution to the goofy reverse thing
        // Fixed the problem when code restarts, rather than power cycle
        motor.setPosition(0);

        voltageControl = new VoltageOut(0)
                .withEnableFOC(true);

        successfulDaqs = new AtomicInteger(0);
        failedDaqs = new AtomicInteger(0);

    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.voltage = 0;
        inputs.hasPiece = !beamBreak.get();
        inputs.successfulDaqs = successfulDaqs.get();
        inputs.failedDaqs = failedDaqs.get();
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Algae Outtake/State", "Voltage");
        Logger.recordOutput("Algae Outtake/Voltage Out", voltage);
        motor.setControl(voltageControl.withOutput(voltage));
    }
}
