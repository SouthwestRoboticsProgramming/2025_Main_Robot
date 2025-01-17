package com.swrobotics.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.swrobotics.lib.utils.CTREUtil;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class OuttakePivotIOReal implements OuttakePivotIO {
    private final TalonFX motor;
    private final CANcoder canCoder;

    private final StatusSignal<Angle> motorPositionStatus;
    private final StatusSignal<Angle> canCoderPositionStatus;

    private final MotionMagicVoltage positionControl;

    public OuttakePivotIOReal() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = Constants.kOuttakePivotMotorToArmRatio;
        motorConfig.Slot0 = getSlot0Configs();

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // FIXME

        motor = IOAllocation.CAN.kOuttakePivotMotor.createTalonFX();
        canCoder = IOAllocation.CAN.kOuttakePivotEncoder.createCANcoder();
        CTREUtil.retryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfig));
        CTREUtil.retryUntilOk(canCoder, () -> canCoder.getConfigurator().apply(canCoderConfig));

        MotorTrackerSubsystem.getInstance().addMotor("Outtake Pivot", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        motorPositionStatus = motor.getPosition();
        canCoderPositionStatus = canCoder.getAbsolutePosition(true);
        CTREUtil.setUpdateFrequency(motor, Constants.kStatusSignalFreq, motorPositionStatus);
        CTREUtil.setUpdateFrequency(canCoder, Constants.kStatusSignalFreq, canCoderPositionStatus);
        CTREUtil.optimizeBusUtilization(motor, canCoder);

        positionControl = new MotionMagicVoltage(0)
                .withEnableFOC(true);

        CTREUtil.retryUntilOk(canCoder, () -> canCoderPositionStatus.waitForUpdate(1).getStatus());
        canCoderPositionStatus.waitForUpdate(1);

        double canCoderPos = canCoderPositionStatus.getValue().in(Units.Rotations);
        double armPos = MathUtil.wrap(
                canCoderPos + Constants.kOuttakePivotEncoderOffset.get(),
                -0.5, 0.5
        ) / Constants.kOuttakePivotCANcoderToArmRatio;

        CTREUtil.retryUntilOk(motor, () -> motor.setPosition(armPos));

        Runnable updateSlot0Configs = () -> CTREUtil.retryUntilOk(
                motor, () -> motor.getConfigurator().apply(getSlot0Configs()));
        Runnable updateMotionMagicConfigs = () -> CTREUtil.retryUntilOk(
                motor, () -> motor.getConfigurator().apply(getMotionMagicConfigs()));
        Constants.kOuttakePivotKg.onChange(updateSlot0Configs);
        Constants.kOuttakePivotKs.onChange(updateSlot0Configs);
        Constants.kOuttakePivotKv.onChange(updateSlot0Configs);
        Constants.kOuttakePivotKa.onChange(updateSlot0Configs);
        Constants.kOuttakePivotKp.onChange(updateSlot0Configs);
        Constants.kOuttakePivotKd.onChange(updateSlot0Configs);
        Constants.kOuttakePivotMMCruiseVelocity.onChange(updateMotionMagicConfigs);
        Constants.kOuttakePivotMMAcceleration.onChange(updateMotionMagicConfigs);
        Constants.kOuttakePivotMMJerk.onChange(updateMotionMagicConfigs);
    }

    private Slot0Configs getSlot0Configs() {
        return new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(Constants.kOuttakePivotKg.get())
                .withKS(Constants.kOuttakePivotKs.get())
                .withKV(Constants.kOuttakePivotKv.get())
                .withKA(Constants.kOuttakePivotKa.get())
                .withKP(Constants.kOuttakePivotKp.get())
                .withKD(Constants.kOuttakePivotKd.get());
    }

    private MotionMagicConfigs getMotionMagicConfigs() {
        return new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.kOuttakePivotMMCruiseVelocity.get())
                .withMotionMagicAcceleration(Constants.kOuttakePivotMMAcceleration.get())
                .withMotionMagicJerk(Constants.kOuttakePivotMMJerk.get());
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.currentAngle = motorPositionStatus.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(double targetAngleRot) {
        motor.setControl(positionControl.withPosition(targetAngleRot));
    }

    @Override
    public void calibrateEncoder() {
        // Assumes that the arm is currently in horizontal position (angle 0)
        canCoderPositionStatus.refresh();
        double canCoderPos = canCoderPositionStatus.getValue().in(Units.Rotations);
        Constants.kOuttakePivotEncoderOffset.set(-canCoderPos);

        CTREUtil.retryUntilOk(motor, () -> motor.setPosition(0));
    }
}
