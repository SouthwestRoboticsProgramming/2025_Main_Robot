package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class OuttakePivotIOSim implements OuttakePivotIO {
    private final TrapezoidProfile profile;

    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State targetState;

    public OuttakePivotIOSim() {
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Constants.kOuttakePivotMotionMagic.getCruiseVelocity(),
                Constants.kOuttakePivotMotionMagic.getAcceleration()
        ));

        currentState = new TrapezoidProfile.State(
                Units.degreesToRotations(Constants.kOuttakePivotInAngle.get()),
                0
        );
        targetState = currentState;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        currentState = profile.calculate(Constants.kPeriodicTime, currentState, targetState);
        inputs.currentAngleRot = currentState.position;
        inputs.currentVelocityRotPerSec = currentState.velocity;
    }

    @Override
    public void setTargetAngle(double targetAngleRot, boolean hasCoral) {
        targetState = new TrapezoidProfile.State(targetAngleRot, 0);
    }

    @Override
    public void syncWithEncoder() {
        // no
    }

    @Override
    public void calibrateEncoder() {
        // no encoder
        currentState.position = 0.25;
    }
}
