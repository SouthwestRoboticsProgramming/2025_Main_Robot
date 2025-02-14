package com.swrobotics.robot.subsystems.intake;

import com.ctre.phoenix6.signals.*;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.lib.ctre.CTREUtil;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class IntakeIOReal implements IntakeIO {
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final CANcoder canCoder;

    private final StatusSignal<Angle> pivotMotorPositionStatus;
    private final StatusSignal<Angle> canCoderPositionStatus;

    private final MotionMagicVoltage positionControl;
    private final VoltageOut rollerControl;

    public IntakeIOReal() {
        pivotMotor = IOAllocation.CAN.kIntakePivotMotor.createTalonFX();
        rollerMotor = IOAllocation.CAN.kIntakeSpinMotor.createTalonFX();
        canCoder = IOAllocation.CAN.kIntakePivotEncoder.createCANcoder();

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CTREUtil.retryUntilOk(canCoder, () -> canCoder.getConfigurator().apply(canCoderConfig));

        TalonFXConfigHelper pivotConfig = new TalonFXConfigHelper();
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.kIntakePivotMotorToArmRatio;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.addTunable(Constants.kIntakePivotPID);
        pivotConfig.addTunable(Constants.kIntakePivotMotionMagic);
        pivotConfig.apply(pivotMotor);

        TalonFXConfigHelper rollerConfig = new TalonFXConfigHelper();
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.kIntakeCurrentLimit.get();
        rollerConfig.apply(rollerMotor);

        MotorTrackerSubsystem.getInstance().addMotor("Intake Pivot", pivotMotor);
        MotorTrackerSubsystem.getInstance().addMotor("Intake Roller", rollerMotor);
        MusicSubsystem.getInstance().addInstrument(pivotMotor);
        MusicSubsystem.getInstance().addInstrument(rollerMotor);

        pivotMotorPositionStatus = pivotMotor.getPosition();
        canCoderPositionStatus = canCoder.getAbsolutePosition();

        CTREUtil.retryUntilOk(canCoder, () -> canCoderPositionStatus.waitForUpdate(1).getStatus());

        double centerOfRange = 45 / 360.0;
        double canCoderPos = canCoderPositionStatus.getValue().in(Units.Rotations);
        double armPos = MathUtil.wrap(
                (canCoderPos + Constants.kIntakePivotEncoderOffset.get())
                        / Constants.kIntakePivotCANcoderToArmRatio,
                centerOfRange - 0.5/Constants.kIntakePivotCANcoderToArmRatio,
                centerOfRange + 0.5/Constants.kIntakePivotCANcoderToArmRatio
        );
        CTREUtil.retryUntilOk(pivotMotor, () -> pivotMotor.setPosition(armPos));

        positionControl = new MotionMagicVoltage(0)
                .withEnableFOC(true);

        rollerControl = new VoltageOut(0);

        Constants.kIntakeCurrentLimit.onChange(() -> {
            rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.kIntakeCurrentLimit.get();
            rollerConfig.reapply();
        });
    }

    @Override
    public void updateInputs(Inputs inputs) {
        canCoderPositionStatus.refresh();
        Logger.recordOutput("Intake/CANcoder Position", canCoderPositionStatus.getValueAsDouble());

        pivotMotorPositionStatus.refresh();
        Logger.recordOutput("Intake/Motor Position", pivotMotorPositionStatus.getValueAsDouble());
        inputs.currentAngleRot = pivotMotorPositionStatus.getValueAsDouble();

        Logger.recordOutput("Intake/Rotor Position", pivotMotor.getRotorPosition().getValueAsDouble());
    }

    @Override
    public void setTargetAngle(double targetAngleRot) {
         pivotMotor.setControl(positionControl.withPosition(targetAngleRot));
    }

    @Override
    public void setVoltage(double targetVoltage) {
        rollerMotor.setControl(rollerControl.withOutput(targetVoltage));
    }

    @Override
    public void calibrateEncoder() {
        // Assumes that the arm is currently in horizontal position (angle 0)
        CTREUtil.retryUntilOk(canCoder, () -> canCoderPositionStatus.waitForUpdate(1).getStatus());
        Constants.kIntakePivotEncoderOffset.set(-canCoderPositionStatus.getValueAsDouble());

        CTREUtil.retryUntilOk(pivotMotor, () -> pivotMotor.setPosition(0));
    }
}
