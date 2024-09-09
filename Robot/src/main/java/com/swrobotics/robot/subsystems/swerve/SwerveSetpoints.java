package com.swrobotics.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class SwerveSetpoints {
    /**
     * Creates the initial setpoints (not moving).
     * @param moduleCount number of swerve modules
     * @return initial setpoints
     */
    public static SwerveSetpoints createInitial(int moduleCount) {
        SwerveModuleState[] states = new SwerveModuleState[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            states[i] = new SwerveModuleState();
        }
        return new SwerveSetpoints(new ChassisSpeeds(), null, states);
    }

    public final ChassisSpeeds chassisSpeeds;
    public final SwerveModuleState[] desiredModuleStates; // The eventual target state
    public final SwerveModuleState[] moduleStates; // The setpoints for this periodic

    public SwerveSetpoints(ChassisSpeeds chassisSpeeds, SwerveModuleState[] desiredModuleStates, SwerveModuleState[] initialStates) {
        this.chassisSpeeds = chassisSpeeds;
        this.desiredModuleStates = desiredModuleStates;
        this.moduleStates = initialStates;
    }
}
