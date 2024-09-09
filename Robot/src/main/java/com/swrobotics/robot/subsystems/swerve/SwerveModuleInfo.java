package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.lib.tunable.LoggedTunableDouble;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.math.geometry.Translation2d;

public record SwerveModuleInfo(
        String canBus,
        int driveId, int turnId, int encoderId,
        Translation2d position,
        LoggedTunableDouble offset,
        String name
) {
    public SwerveModuleInfo(IOAllocation.SwerveIDs ids, double x, double y, LoggedTunableDouble offset, String name) {
        this(
                ids.drive.bus(),
                ids.drive.id(), ids.turn.id(), ids.encoder.id(),
                new Translation2d(x, y),
                offset,
                name
        );
    }
}
