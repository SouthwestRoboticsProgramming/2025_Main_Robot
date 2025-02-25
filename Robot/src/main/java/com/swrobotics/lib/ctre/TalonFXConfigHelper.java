package com.swrobotics.lib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;
import java.util.List;

public final class TalonFXConfigHelper extends TalonFXConfiguration {
    private final List<TalonFX> motors;

    public TalonFXConfigHelper() {
        motors = new ArrayList<>();

        this.Audio.BeepOnBoot = true;
        this.Audio.BeepOnConfig = true;
        this.Audio.AllowMusicDurDisable = true;
    }

    public TalonFXConfigHelper addTunable(TunableConfig tunable) {
        tunable.setAndBind(this, () -> {
            for (TalonFX motor : motors) {
                CTREUtil.retryUntilOk(motor, () -> motor.getConfigurator().apply(this));
            }
        });
        return this;
    }

    public void apply(TalonFX... motors) {
        for (TalonFX fx : motors) {
            this.motors.add(fx);
            CTREUtil.retryUntilOk(fx, () -> fx.getConfigurator().apply(this));
        }
    }

    public void reapply() {
        for (TalonFX fx : motors) {
            CTREUtil.retryUntilOk(fx, () -> fx.getConfigurator().apply(this));
        }
    }
}
