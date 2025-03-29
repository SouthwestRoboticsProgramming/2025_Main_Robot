package com.swrobotics.robot.subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.CTREUtil;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class ElevatorIOReal implements ElevatorIO {
    private static final NTBoolean BRAKE_MODE = new NTBoolean("Superstructure/Elevator/Brake Mode", true);

    private final TalonFX motor1, motor2;

    private final StatusSignal<Angle> positionStatus;
    private final StatusSignal<AngularVelocity> velocityStatus;

    private final PositionVoltage positionControl;
    private final VoltageOut voltageControl;

    public ElevatorIOReal() {
        motor1 = IOAllocation.CAN.kElevatorMotor1.createTalonFX();
        motor2 = IOAllocation.CAN.kElevatorMotor2.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.addTunable(Constants.kElevatorPID);
        config.apply(motor1, motor2);

        CTREUtil.retryUntilOk(motor2, () -> motor2.setControl(new Follower(IOAllocation.CAN.kElevatorMotor1.id(), true)));

        MotorTrackerSubsystem.getInstance().addMotor("Elevator Motor 1", motor1);
        MotorTrackerSubsystem.getInstance().addMotor("Elevator Motor 2", motor2);
        MusicSubsystem.getInstance().addInstrument(motor1);
        MusicSubsystem.getInstance().addInstrument(motor2);

        motor1.setPosition(0); // Start fully down
        positionStatus = motor1.getPosition();
        velocityStatus = motor1.getVelocity();

        positionControl = new PositionVoltage(0)
                .withEnableFOC(true);
        voltageControl = new VoltageOut(0)
                .withEnableFOC(true);

        BRAKE_MODE.onChange(() -> {
            config.MotorOutput.NeutralMode = BRAKE_MODE.get()
                    ? NeutralModeValue.Brake
                    : NeutralModeValue.Coast;
            config.reapply();
        });
    }

    @Override
    public void updateInputs(Inputs inputs) {
        BaseStatusSignal.refreshAll(positionStatus, velocityStatus);
        double position = positionStatus.getValue().in(Units.Rotations);
        double velocity = velocityStatus.getValue().in(Units.RotationsPerSecond);

        inputs.currentHeightPct = position / Constants.kElevatorMaxHeightRotations;
        inputs.currentVelocityPctPerSec = velocity / Constants.kElevatorMaxHeightRotations;
    }

    @Override
    public void setTarget(double heightPct, double ffVelocityPctPerSec) {
        heightPct = MathUtil.clamp(heightPct, 0, 1);

        double positionRot = heightPct * Constants.kElevatorMaxHeightRotations;
        double velocityRot = ffVelocityPctPerSec * Constants.kElevatorMaxHeightRotations;

        // motor1.setControl(positionControl
        //         .withPosition(positionRot)
        //         .withVelocity(velocityRot));
    }

    @Override
    public void setVoltage(double volts) {
        // motor1.setControl(voltageControl.withOutput(volts));
    }
}
