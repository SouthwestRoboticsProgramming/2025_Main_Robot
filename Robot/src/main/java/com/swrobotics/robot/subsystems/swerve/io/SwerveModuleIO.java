package com.swrobotics.robot.subsystems.swerve.io;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.swrobotics.robot.logging.AutoLoggedInputs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModuleIO {
    private final String name;

    public SwerveModuleIO(String name) {
        this.name = name;
    }

    public abstract void updateInputs(Inputs inputs);

    /**
     * Reconfigures the CANcoder's magnet offset.
     *
     * @param offset new magnet offset in rotations
     */
    public abstract void setCANcoderMagnetOffset(double offset);

    /**
     * Sets the module's target state.
     *
     * @param state target module state
     * @param driveRequestType type of drive request to use
     */
    public abstract void setTarget(SwerveModuleState state, DriveRequestType driveRequestType);

    public String getName() {
        return name;
    }

    public static final class Inputs extends AutoLoggedInputs {
        /** Total distance the module has traveled in meters */
        public double drivePosition = 0;
        /** Current velocity of the drive in meters/second */
        public double driveVelocity = 0;
        /** Current angle of the wheel in CCW rotations */
        public double angle = 0;
        /** Current position of the CANcoder in CCW rotations */
        public double canCoderPos = 0;

        public SwerveModuleState getState() {
            return new SwerveModuleState(driveVelocity, Rotation2d.fromRotations(angle));
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(drivePosition, Rotation2d.fromRotations(angle));
        }
    }
}
