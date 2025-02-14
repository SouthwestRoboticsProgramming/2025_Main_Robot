package com.swrobotics.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.ctre.CTREUtil;
import com.swrobotics.lib.ctre.TalonFXConfigHelper;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class ElevatorIOReal implements ElevatorIO {
    private static final NTBoolean BRAKE_MODE = new NTBoolean("Superstructure/Elevator/Brake Mode", true);

    private final TalonFX motor1, motor2;

    private final StatusSignal<Angle> positionStatus;

    private final MotionMagicVoltage positionControl;
    private final NeutralOut neutralControl;

    private boolean climbMode;

    public ElevatorIOReal() {
        motor1 = IOAllocation.CAN.kElevatorMotor1.createTalonFX();
        motor2 = IOAllocation.CAN.kElevatorMotor2.createTalonFX();

        TalonFXConfigHelper config = new TalonFXConfigHelper();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.addTunable(Constants.kElevatorPID);
        config.addTunable(Constants.kElevatorMotionMagic);
        config.apply(motor1, motor2);

        CTREUtil.retryUntilOk(motor2, () -> motor2.setControl(new Follower(IOAllocation.CAN.kElevatorMotor1.id(), true)));

        MotorTrackerSubsystem.getInstance().addMotor("Elevator Motor 1", motor1);
        MotorTrackerSubsystem.getInstance().addMotor("Elevator Motor 2", motor2);
        MusicSubsystem.getInstance().addInstrument(motor1);
        MusicSubsystem.getInstance().addInstrument(motor2);

        motor1.setPosition(0); // Start fully down
        positionStatus = motor1.getPosition();

        positionControl = new MotionMagicVoltage(0)
                .withEnableFOC(true);
        neutralControl = new NeutralOut();

        BRAKE_MODE.onChange(() -> {
            config.MotorOutput.NeutralMode = BRAKE_MODE.get()
                    ? NeutralModeValue.Brake
                    : NeutralModeValue.Coast;
            config.reapply();
        });
    }

    @Override
    public void updateInputs(Inputs inputs) {
        positionStatus.refresh();
        double position = positionStatus.getValue().in(Units.Rotations);
        inputs.currentHeightPct = position / Constants.kElevatorMaxHeightRotations;
    }

    @Override
    public void setTargetHeight(double heightPct) {
        heightPct = Math.min(heightPct, 1.0);

        double positionRot = heightPct * Constants.kElevatorMaxHeightRotations;

        double climbFF = climbMode ? Constants.kElevatorClimbKg.get() : 0;
        motor1.setControl(positionControl
                .withPosition(positionRot)
                .withFeedForward(climbFF));
    }

    @Override
    public void setNeutral() {
        motor1.setControl(neutralControl);
    }

    @Override
    public void setClimbMode(boolean activelyClimbing) {
        climbMode = activelyClimbing;
    }
}
