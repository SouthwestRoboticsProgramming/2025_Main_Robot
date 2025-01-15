package com.swrobotics.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

import java.util.function.Consumer;

public final class ElevatorIOReal implements ElevatorIO {
    private final TalonFX motor1, motor2;

    private final StatusSignal<Angle> positionStatus;

    private final MotionMagicVoltage positionControl;
    private final NeutralOut neutralControl;

    public ElevatorIOReal() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0 = getSlot0Configs();
        config.MotionMagic = getMotionMagicConfigs();

        motor1 = IOAllocation.CAN.kElevatorMotor1.createTalonFX();
        motor2 = IOAllocation.CAN.kElevatorMotor2.createTalonFX();
        motor1.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);

        motor2.setControl(new Follower(IOAllocation.CAN.kElevatorMotor1.id(), true));

        MotorTrackerSubsystem.getInstance().addMotor("Elevator Motor 1", motor1);
        MotorTrackerSubsystem.getInstance().addMotor("Elevator Motor 2", motor2);
        MusicSubsystem.getInstance().addInstrument(motor1);
        MusicSubsystem.getInstance().addInstrument(motor2);

        motor1.setPosition(0); // Start fully down
        positionStatus = motor1.getPosition();

        positionControl = new MotionMagicVoltage(0)
                .withEnableFOC(true);
        neutralControl = new NeutralOut();

        Consumer<Double> updateSlot0Configs = (v) -> {
            Slot0Configs configs = getSlot0Configs();
            motor1.getConfigurator().apply(configs);
            motor2.getConfigurator().apply(configs);
        };
        Consumer<Double> updateMotionMagicConfigs = (v) -> {
            MotionMagicConfigs configs = getMotionMagicConfigs();
            motor1.getConfigurator().apply(configs);
            motor2.getConfigurator().apply(configs);
        };
        Constants.kElevatorKg.onChange(updateSlot0Configs);
        Constants.kElevatorKs.onChange(updateSlot0Configs);
        Constants.kElevatorKv.onChange(updateSlot0Configs);
        Constants.kElevatorKa.onChange(updateSlot0Configs);
        Constants.kElevatorKp.onChange(updateSlot0Configs);
        Constants.kElevatorKd.onChange(updateSlot0Configs);
        Constants.kElevatorMMCruiseVelocity.onChange(updateMotionMagicConfigs);
        Constants.kElevatorMMAcceleration.onChange(updateMotionMagicConfigs);
        Constants.kElevatorMMJerk.onChange(updateMotionMagicConfigs);
    }

    private Slot0Configs getSlot0Configs() {
        return new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKG(Constants.kElevatorKg.get())
                .withKS(Constants.kElevatorKs.get())
                .withKV(Constants.kElevatorKv.get())
                .withKA(Constants.kElevatorKa.get())
                .withKP(Constants.kElevatorKp.get())
                .withKD(Constants.kElevatorKd.get());
    }

    private MotionMagicConfigs getMotionMagicConfigs() {
        return new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.kElevatorMMCruiseVelocity.get())
                .withMotionMagicAcceleration(Constants.kElevatorMMAcceleration.get())
                .withMotionMagicJerk(Constants.kElevatorMMJerk.get());
    }

    @Override
    public void updateInputs(Inputs inputs) {
        positionStatus.refresh();
        double position = positionStatus.getValue().in(Units.Rotations);
        inputs.currentHeight = position / Constants.kElevatorRotationsForFullHeight;
    }

    @Override
    public void setTargetHeight(double heightPct) {
        double positionRot = heightPct * Constants.kElevatorRotationsForFullHeight;
        motor1.setControl(positionControl.withPosition(positionRot));
    }

    @Override
    public void setNeutral() {
        motor1.setControl(neutralControl);
    }
}
