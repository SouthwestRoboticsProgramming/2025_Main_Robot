package com.swrobotics.robot.subsystems.swerve.io;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Swerve module implementation for simulation. This is used instead of the
 * CTRE simulation API because simulation of the CTRE swerve API is currently
 * somewhat unstable.
 */
public final class SimSwerveModuleIO extends SwerveModuleIO {
    // Current module state
    private SwerveModuleState state;
    // Total distance traveled by the module (drive motor encoder, meters)
    private double simDistance;
    // Offset of the CANcoder to the wheel angle (rotations)
    private double magnetOffset;

    public SimSwerveModuleIO(String name, double magnetOffset) {
        super(name);

        state = new SwerveModuleState();
        simDistance = 0;
        this.magnetOffset = magnetOffset;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        simDistance += state.speedMetersPerSecond * Constants.kPeriodicTime;

        inputs.angle = state.angle.getRotations();
        inputs.canCoderPos = MathUtil.wrap(inputs.angle + magnetOffset, 0, 1);
        inputs.driveVelocity = state.speedMetersPerSecond;
        inputs.drivePosition = simDistance;
    }

    @Override
    public void setCANcoderMagnetOffset(double offset) {
        magnetOffset = offset;
    }

    @Override
    public void setTarget(SwerveModuleState state, LegacySwerveModule.DriveRequestType driveRequestType) {
        // Set current state directly to target state
        SwerveModuleState copy = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
        copy.optimize(this.state.angle); // Mutates copy
        this.state = copy;
    }
}
