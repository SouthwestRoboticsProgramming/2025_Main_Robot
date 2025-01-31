package com.swrobotics.robot.subsystems.algae;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.swrobotics.lib.ctre.CTREUtil;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

import edu.wpi.first.units.measure.Angle;

public class AlgaeIOReal implements AlgaeIO {
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final CANcoder canCoder;

    private final StatusSignal<Angle> pivotMotorPositionStatus;

    private final MotionMagicVoltage positionControl;
    private final VoltageOut rollerControl;

    public AlgaeIOReal() {
        pivotMotor = IOAllocation.CAN.kAlgaeIntakePivotMotor.createTalonFX();
        rollerMotor = IOAllocation.CAN.kAlgaeIntakeSpinMotor.createTalonFX();
        canCoder = IOAllocation.CAN.kAlgaeIntakePivotEncoder.createCANcoder();

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // FIXME
        CTREUtil.retryUntilOk(canCoder, () -> canCoder.getConfigurator().apply(canCoderConfig));

        TalonFXConfigHelper pivotConfig = new TalonFXConfigHelper();
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Feedback.FeedbackRemoteSensorID = IOAllocation.CAN.kAlgaeIntakePivotEncoder.id();
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.kAlgaePivotCANcoderToArmRatio;
        pivotConfig.Feedback.RotorToSensorRatio = Constants.kAlgaePivotMotorToArmRatio / Constants.kAlgaePivotCANcoderToArmRatio;
        pivotConfig.withTunable(Constants.kAlgaePivotPID);
        CTREUtil.retryUntilOk(pivotMotor, () -> pivotMotor.getConfigurator().apply(pivotConfig));

        TalonFXConfigHelper rollerConfig = new TalonFXConfigHelper();
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CTREUtil.retryUntilOk(rollerMotor, () -> rollerMotor.getConfigurator().apply(rollerConfig));

        MotorTrackerSubsystem.getInstance().addMotor("Algae Pivot", pivotMotor);
        MotorTrackerSubsystem.getInstance().addChild("Algae Roller", rollerMotor);
        MusicSubsystem.getInstance().addInstrument(pivotMotor);
        MusicSubsystem.getInstance().addInstrument(rollerMotor);

        pivotMotorPositionStatus = pivotMotor.getPosition();

        positionControl = new MotionMagicVoltage(0)
                .withEnableFOC(true);

        rollerControl = new VoltageOut(0);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.currentAngleRot = pivotMotorPositionStatus.getValueAsDouble();
        inputs.voltageOut = rollerMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setTargetAngle(Angle targetAngle) {
        pivotMotor.setControl(positionControl.withPosition(targetAngle));
    }

    @Override
    public void setVoltage(double targetVoltage) {
        rollerMotor.setControl(rollerControl.withOutput(targetVoltage));
    }

    @Override
    public void calibrateEncoder() {
        // Assumes that the arm is currently in horizontal position (angle 0)
        Constants.kAlgaePivotEncoderOffset.set(canCoder.getAbsolutePosition().getValueAsDouble());

        CTREUtil.retryUntilOk(canCoder, () -> canCoder.setPosition(0));
    }
    
}
