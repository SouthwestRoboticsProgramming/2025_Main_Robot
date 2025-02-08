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

    public OuttakePivotIOReal() {
        motor = IOAllocation.CAN.kOuttakePivotMotor.createTalonFX();
        canCoder = IOAllocation.CAN.kOuttakePivotEncoder.createCANcoder();

        TalonFXConfigHelper motorConfig = new TalonFXConfigHelper();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = Constants.kOuttakePivotMotorToArmRatio;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.withTunable(Constants.kOuttakePivotPID);
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

        CTREUtil.retryUntilOk(canCoder, () -> canCoderPositionStatus.waitForUpdate(1).getStatus());

        // double canCoderPos = canCoderPositionStatus.getValue().in(Units.Rotations);
        // double armPos = MathUtil.wrap(
        //         canCoderPos + Constants.kOuttakePivotEncoderOffset.get(),
        //         -0.5, 0.5
        // ) / Constants.kOuttakePivotCANcoderToArmRatio;

        double armPos = Math.PI / 2;

        CTREUtil.retryUntilOk(motor, () -> motor.setPosition(armPos));
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.currentAngleRot = motorPositionStatus.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(double targetAngleRot) {
        motor.setControl(new CoastOut());
        // motor.setControl(positionControl.withPosition(targetAngleRot));
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
