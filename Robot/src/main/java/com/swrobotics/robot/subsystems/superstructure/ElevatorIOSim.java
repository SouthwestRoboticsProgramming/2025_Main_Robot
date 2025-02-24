package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class ElevatorIOSim implements ElevatorIO {
    private final TrapezoidProfile profile;

    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State targetState;

    public ElevatorIOSim() {
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Constants.kElevatorMotionMagic.getCruiseVelocity() / Constants.kElevatorMaxHeightRotations,
                Constants.kElevatorMotionMagic.getAcceleration() / Constants.kElevatorMaxHeightRotations
        ));

        currentState = new TrapezoidProfile.State(0, 0);
        targetState = new TrapezoidProfile.State(0, 0);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        currentState = profile.calculate(Constants.kPeriodicTime, currentState, targetState);
        inputs.currentHeightPct = currentState.position;
        inputs.currentVelocityPctPerSec = currentState.velocity;
    }

    @Override
    public void setTargetHeight(double heightPct) {
        targetState = new TrapezoidProfile.State(heightPct, 0);
    }

    @Override
    public void setNeutral() {
        // Fall down
        currentState.position = 0;
        currentState.velocity = 0;
        targetState = new TrapezoidProfile.State(0, 0);
    }
}
