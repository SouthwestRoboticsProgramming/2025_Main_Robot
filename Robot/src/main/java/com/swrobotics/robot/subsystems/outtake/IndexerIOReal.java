package com.swrobotics.robot.subsystems.outtake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.swrobotics.robot.config.IOAllocation;

public class IndexerIOReal implements IndexerIO {
    private final VictorSPX motor;
    

    public IndexerIOReal() {
        motor = new VictorSPX(IOAllocation.CAN.kIndexerMotor.id());
        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(12);
    }

    @Override
    public void updateInputs(Inputs inputs) {
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Indexer/Voltage", voltage);
        motor.set(VictorSPXControlMode.PercentOutput, voltage / 12.0);
    }
    
}
