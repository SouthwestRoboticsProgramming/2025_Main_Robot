package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.robot.config.Constants;

public final class SimSwerveIO extends CtreSwerveIO {
    @Override
    public void updateInputs(Inputs inputs) {
        getDrivetrain().updateSimState(Constants.kPeriodicTime, 12.0);
        super.updateInputs(inputs);
    }
}
