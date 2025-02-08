package com.swrobotics.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public final class OuttakePivotIOReal implements OuttakePivotIO {
    private final TalonFX motor;
    private final CANcoder canCoder;

    private final StatusSignal<Angle> motorPositionStatus;
    private final StatusSignal<Angle> canCoderPositionStatus;

    private final MotionMagicVoltage positionControl;
    private final MotionMagicVoltage positionControlWithCoral;

    public OuttakePivotIOReal() {
        motor = IOAllocation.CAN.kOuttakePivotMotor.createTalonFX();
        canCoder = IOAllocation.CAN.kOuttakePivotEncoder.createCANcoder();

        TalonFXConfigHelper motorConfig = new TalonFXConfigHelper();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = Constants.kOuttakePivotMotorToArmRatio;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.withTunable(Constants.kOuttakePivotPID);
        motorConfig.withTunable(Constants.kOuttakePivotPIDWithCoral);
        motorConfig.apply(motor);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // FIXME
        CTREUtil.retryUntilOk(canCoder, () -> canCoder.getConfigurator().apply(canCoderConfig));

        MotorTrackerSubsystem.getInstance().addMotor("Outtake Pivot", motor);
        MusicSubsystem.getInstance().addInstrument(motor);

        motorPositionStatus = motor.getPosition();
        canCoderPositionStatus = canCoder.getAbsolutePosition(true);

        positionControl = new MotionMagicVoltage(0)
                .withEnableFOC(true);
        positionControlWithCoral = new MotionMagicVoltage(0)
                .withSlot(1)
                .withEnableFOC(true);

        CTREUtil.retryUntilOk(canCoder, () -> canCoderPositionStatus.waitForUpdate(1).getStatus());

        double centerOfRange = 45 / 360.0;
        double canCoderPos = canCoderPositionStatus.getValue().in(Units.Rotations);
        double armPos = MathUtil.wrap(
                (canCoderPos + Constants.kOuttakePivotEncoderOffset.get())
                        / Constants.kOuttakePivotCANcoderToArmRatio,
                centerOfRange - 0.5/Constants.kOuttakePivotCANcoderToArmRatio,
                centerOfRange + 0.5/Constants.kOuttakePivotCANcoderToArmRatio
        );
        CTREUtil.retryUntilOk(motor, () -> motor.setPosition(armPos));
    }

    @Override
    public void updateInputs(Inputs inputs) {
        motorPositionStatus.refresh();
        inputs.currentAngleRot = motorPositionStatus.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(double targetAngleRot, boolean hasCoral) {
        MotionMagicVoltage control = hasCoral
                ? positionControl
                : positionControlWithCoral;

        motor.setControl(control.withPosition(targetAngleRot));
    }

    @Override
    public void calibrateEncoder() {
        // Assumes that the arm is currently in vertical position
        double calibrationAngle = 0.25; // Rotations from horizontal

        CTREUtil.retryUntilOk(canCoder, () -> canCoderPositionStatus.waitForUpdate(1).getStatus());
        double canCoderPos = canCoderPositionStatus.getValue().in(Units.Rotations);
        double offset = calibrationAngle * Constants.kOuttakePivotCANcoderToArmRatio - canCoderPos;
        Constants.kOuttakePivotEncoderOffset.set(offset);

        CTREUtil.retryUntilOk(motor, () -> motor.setPosition(calibrationAngle));
    }
}
