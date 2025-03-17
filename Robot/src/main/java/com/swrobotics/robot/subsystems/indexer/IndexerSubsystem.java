package com.swrobotics.robot.subsystems.indexer;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swrobotics.robot.config.IOAllocation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
//    private final TalonSRX motor;

    public IndexerSubsystem() {
//        motor = new TalonSRX(IOAllocation.CAN.kIndexerMotor.id());
    }

    @Override
    public void periodic() {
//        motor.set(TalonSRXControlMode.PercentOutput, com.swrobotics.robot.config.Constants.kIndexerIntakeVoltage.get() / 12.0);
    }
}
