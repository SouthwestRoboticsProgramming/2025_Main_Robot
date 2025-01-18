package com.swrobotics.lib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public interface TunableConfig {
    void setAndBind(TalonFXConfiguration config, Runnable applyFn);
}
