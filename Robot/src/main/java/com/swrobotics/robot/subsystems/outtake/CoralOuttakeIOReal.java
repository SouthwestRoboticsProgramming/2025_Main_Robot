package com.swrobotics.robot.subsystems.outtake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.CTREUtil;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.atomic.AtomicInteger;

public class CoralOuttakeIOReal implements CoralOuttakeIO {
    private final TalonFX motor;
    private final DigitalInput beamBreak;

    private final StatusSignal<Angle> positionStatus;

    private volatile boolean ignoreBeamBreak;
    private volatile boolean hasPiece;
    private volatile double positionAtPieceDetect;
    private final AtomicInteger successfulDaqs, failedDaqs;

    private final VoltageOut voltageControl;
    private final PositionVoltage positionVoltage;

    public CoralOuttakeIOReal() {
        motor = IOAllocation.CAN.kOuttakeMotor.createTalonFX();
        beamBreak = new DigitalInput(IOAllocation.RIO.kDIO_OuttakeBeamBreak);

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.addTunable(Constants.kOuttakeRollerPID);
        config.apply(motor);

        MotorTrackerSubsystem.getInstance().addMotor("Outtake", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        // Better solution to the goofy reverse thing
        // Fixed the problem when code restarts, rather than power cycle
        motor.setPosition(0);

        positionStatus = motor.getPosition();
        CTREUtil.retryUntilOk(motor, () -> positionStatus.setUpdateFrequency(Constants.kOuttakeRefreshFreq));

        voltageControl = new VoltageOut(0)
                .withEnableFOC(true);
        positionVoltage = new PositionVoltage(0)
                .withEnableFOC(true);

        ignoreBeamBreak = false;

        successfulDaqs = new AtomicInteger(0);
        failedDaqs = new AtomicInteger(0);
        new Thread(this::runFastThread).start();
    }

    private void runFastThread() {
        Threads.setCurrentThreadPriority(true, 1);

        while (true) {
            Timer.delay(1.0 / Constants.kOuttakeRefreshFreq);
            positionStatus.refresh();

            if (positionStatus.getStatus().isOK())
                successfulDaqs.incrementAndGet();
            else
                failedDaqs.incrementAndGet();

            boolean hadPiece = hasPiece;
            boolean hasPiece = !ignoreBeamBreak && !beamBreak.get();
            if (!hadPiece && hasPiece)
                positionAtPieceDetect = positionStatus.getValueAsDouble();
            this.hasPiece = hasPiece;
        }
    }

    @Override
    public void updateInputs(Inputs inputs) {
        positionStatus.refresh();
        inputs.voltage = 0;
        inputs.positionAtPieceDetect = positionAtPieceDetect;
        inputs.hasPiece = hasPiece;
        inputs.successfulDaqs = successfulDaqs.get();
        inputs.failedDaqs = failedDaqs.get();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void setHoldPosition(double position) {
        motor.setControl(positionVoltage.withPosition(position));
    }

    @Override
    public void setBeamBreakIgnored(boolean ignored) {
        ignoreBeamBreak = ignored;
    }
}
