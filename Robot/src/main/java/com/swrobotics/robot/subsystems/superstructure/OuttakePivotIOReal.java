package com.swrobotics.robot.subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class OuttakePivotIOReal implements OuttakePivotIO {
    private final TalonFX motor;
    private final CANcoder canCoder;

    private final StatusSignal<Angle> motorPositionStatus;
    private final StatusSignal<AngularVelocity> motorVelocityStatus;
    private final StatusSignal<Angle> canCoderPositionStatus;

    private final PositionVoltage positionControl;
    private final PositionVoltage positionControlWithCoral;

    private double motorPositionOffset;

    public OuttakePivotIOReal() {
        motor = IOAllocation.CAN.kOuttakePivotMotor.createTalonFX();
        canCoder = IOAllocation.CAN.kOuttakePivotEncoder.createCANcoder();

        TalonFXConfigHelper motorConfig = new TalonFXConfigHelper();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = Constants.kOuttakePivotMotorToArmRatio;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.addTunable(Constants.kOuttakePivotPID);
        motorConfig.addTunable(Constants.kOuttakePivotPIDWithCoral);
        motorConfig.apply(motor);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        CTREUtil.retryUntilOk(canCoder, () -> canCoder.getConfigurator().apply(canCoderConfig));

        MotorTrackerSubsystem.getInstance().addMotor("Outtake Pivot", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        motorPositionStatus = motor.getPosition();
        motorVelocityStatus = motor.getVelocity();
        canCoderPositionStatus = canCoder.getAbsolutePosition(true);

        positionControl = new PositionVoltage(0)
                .withSlot(0)
                .withEnableFOC(true);
        positionControlWithCoral = new PositionVoltage(0)
                .withSlot(1)
                .withEnableFOC(true);

        CTREUtil.retryUntilOk(motor, () -> motor.setPosition(0, 1));
        motorPositionOffset = 0;

        CTREUtil.retryUntilOk(canCoder, () -> canCoderPositionStatus.waitForUpdate(1).getStatus());
        setCurrentPosition(calcAbsolutePosition(canCoderPositionStatus.getValueAsDouble()));
    }

    private double calcAbsolutePosition(double canCoderPos) {
        double centerOfRange = 0;
        return MathUtil.wrap(
                (canCoderPos + Constants.kOuttakePivotEncoderOffset.get())
                        / Constants.kOuttakePivotCANcoderToArmRatio,
                centerOfRange - 0.5/Constants.kOuttakePivotCANcoderToArmRatio,
                centerOfRange + 0.5/Constants.kOuttakePivotCANcoderToArmRatio
        );
    }

    @Override
    public void updateInputs(Inputs inputs) {
        BaseStatusSignal.refreshAll(motorPositionStatus, motorVelocityStatus, canCoderPositionStatus);
        inputs.currentAngleRot = motorPositionStatus.getValueAsDouble() + motorPositionOffset;
        inputs.currentVelocityRotPerSec = motorVelocityStatus.getValueAsDouble();
        inputs.absoluteAngleRot = calcAbsolutePosition(canCoderPositionStatus.getValueAsDouble());
        inputs.absoluteAngleOK = canCoderPositionStatus.getStatus().isOK();
    }

    @Override
    public void setTarget(double targetAngleRot, double ffVelocityRotPerSec, boolean hasCoral) {
        PositionVoltage control = hasCoral
                ? positionControl
                : positionControlWithCoral;

        // motor.setControl(control
        //         .withPosition(targetAngleRot - motorPositionOffset)
        //         .withVelocity(ffVelocityRotPerSec));
    }

    @Override
    public void setCurrentPosition(double currentAngleRot) {
        motorPositionOffset = currentAngleRot - motorPositionStatus.getValueAsDouble();
    }

    @Override
    public void calibrateEncoder() {
        // Assumes that the arm is currently in vertical position
        double calibrationAngle = 0.75; // Rotations from horizontal

        CTREUtil.retryUntilOk(canCoder, () -> canCoderPositionStatus.waitForUpdate(1).getStatus());
        double canCoderPos = canCoderPositionStatus.getValue().in(Units.Rotations);
        double offset = calibrationAngle * Constants.kOuttakePivotCANcoderToArmRatio - canCoderPos;
        Constants.kOuttakePivotEncoderOffset.set(offset);

        CTREUtil.retryUntilOk(motor, () -> motor.setPosition(calibrationAngle));
    }
}
