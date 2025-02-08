package com.swrobotics.lib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;

public final class NTSlot1MotionMagicConfigs extends NTSlot1Configs {
    private final NTEntry<Double> mmCruiseVelocity;
    private final NTEntry<Double> mmAcceleration;
    private final NTEntry<Double> mmJerk;

    public NTSlot1MotionMagicConfigs(
            String table,
            double kP, double kD,
            double kG, double kS, double kV, double kA,
            double mmCruiseVelocity, double mmAcceleration, double mmJerk) {
        super(table, kP, kD, kG, kS, kV, kA);

        this.mmCruiseVelocity = new NTDouble(table + "/MM Cruise Velocity", mmCruiseVelocity).setPersistent();
        this.mmAcceleration = new NTDouble(table + "/MM Acceleration", mmAcceleration).setPersistent();
        this.mmJerk = new NTDouble(table + "/MM Jerk", mmJerk).setPersistent();
    }

    @Override
    public void setAndBind(TalonFXConfiguration config, Runnable applyFn) {
        super.setAndBind(config, applyFn);

        config.MotionMagic.MotionMagicCruiseVelocity = mmCruiseVelocity.get();
        config.MotionMagic.MotionMagicAcceleration = mmAcceleration.get();
        config.MotionMagic.MotionMagicJerk = mmJerk.get();

        mmCruiseVelocity.onChange(() -> {
            config.MotionMagic.MotionMagicCruiseVelocity = mmCruiseVelocity.get();
            applyFn.run();
        });
        mmAcceleration.onChange(() -> {
            config.MotionMagic.MotionMagicAcceleration = mmAcceleration.get();
            applyFn.run();
        });
        mmJerk.onChange(() -> {
            config.MotionMagic.MotionMagicJerk = mmJerk.get();
            applyFn.run();
        });
    }
}
