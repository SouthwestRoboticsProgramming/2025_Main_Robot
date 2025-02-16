package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.lib.utils.MathUtil;

public final class ElevatorIOSim implements ElevatorIO {
    private double currentHeight;
    private double targetHeight;

    public ElevatorIOSim() {
        currentHeight = 0;
        targetHeight = 0;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        currentHeight = MathUtil.lerp(currentHeight, targetHeight, 0.2);
        inputs.currentHeightPct = currentHeight;
    }

    @Override
    public void setTargetHeight(double heightPct) {
        targetHeight = heightPct;
    }

    @Override
    public void setNeutral() {
        // Fall down
        currentHeight = 0;
        targetHeight = 0;
    }

    @Override
    public void setClimbMode(boolean activelyClimbing) {}
}
