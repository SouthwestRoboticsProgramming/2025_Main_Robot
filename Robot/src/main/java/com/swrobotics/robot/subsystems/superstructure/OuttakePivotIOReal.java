package com.swrobotics.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.swrobotics.lib.ctre.CTREUtil;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class OuttakePivotIOReal implements OuttakePivotIO {
    private final TalonFX motor;
    private final CANcoder canCoder;

    private final StatusSignal<Angle> motorPositionStatus;
    private final StatusSignal<Angle> canCoderPositionStatus;

    private final MotionMagicVoltage positionControl;

    public OuttakePivotIOReal() {
        motor = IOAllocation.CAN.kOuttakePivotMotor.createTalonFX();
        canCoder = IOAllocation.CAN.kOuttakePivotEncoder.createCANcoder();

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // FIXME
        canCoderConfig.MagnetSensor.MagnetOffset = Constants.kOuttakePivotEncoderOffset.get();
        CTREUtil.retryUntilOk(canCoder, () -> canCoder.getConfigurator().apply(canCoderConfig));

        TalonFXConfigHelper motorConfig = new TalonFXConfigHelper();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.FeedbackRemoteSensorID = IOAllocation.CAN.kOuttakePivotEncoder.id();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = Constants.kOuttakePivotCANcoderToArmRatio;
        motorConfig.Feedback.RotorToSensorRatio = Constants.kOuttakePivotMotorToArmRatio / Constants.kOuttakePivotCANcoderToArmRatio;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.withTunable(Constants.kOuttakePivotPID);
        motorConfig.apply(motor);

        MotorTrackerSubsystem.getInstance().addMotor("Outtake Pivot", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        motorPositionStatus = motor.getPosition();
        canCoderPositionStatus = canCoder.getAbsolutePosition(true);

        positionControl = new MotionMagicVoltage(0)
                .withEnableFOC(true);

        CTREUtil.retryUntilOk(canCoder, () -> canCoderPositionStatus.waitForUpdate(1).getStatus());
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.currentAngleRot = motorPositionStatus.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(double targetAngleRot) {
        motor.setControl(positionControl.withPosition(targetAngleRot));
    }

    @Override
    public void calibrateEncoder() {
        // Assumes that the arm is currently in vertical position (angle 90)
        CTREUtil.retryUntilOk(canCoder, () -> canCoder.setPosition(0.25));
        // TODO: Check that the motor resets when the CANCoder does
    }
}
