package com.swrobotics.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.pathfinding.pathplanner.AutoBuilderExt;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import com.swrobotics.robot.config.PathEnvironments;
import com.swrobotics.robot.subsystems.superstructure.SuperstructureSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class RobotCommands {
    public static Command autoPathfindAndScore(RobotContainer robot, int branch, int level) {
        // FIXME: Get from constants
//        final PathConstraints constraints = new PathConstraints(
//                1, 1, 1, 1
//        );

        PathConstraints constraints = new PathConstraints(
                Constants.kDriveMaxAchievableSpeed,
                Constants.kDriveControlMaxAccel,
                Units.rotationsToRadians(Constants.kDriveControlMaxTurnSpeed),
                Units.rotationsToRadians(Constants.kDriveControlMaxTurnSpeed / 0.3));

        final double switchToSnapDist = 0.2;
        final double snapTolerance = 0.03;

        return Commands.sequence(
                robot.superstructure.commandSetStateOnce(
                        SuperstructureSubsystem.State.forScoring(level)
                ),

                AutoBuilderExt.pathfindToPoseFlipped(
                        PathEnvironments.kFieldWithAutoGamePieces,
                        FieldPositions.getBlueReefScoringTarget(branch),
                        constraints
                ).until(() -> robot.drive.isCloseTo(
                        FieldPositions.getAllianceReefScoringTarget(branch).getTranslation(),
                        switchToSnapDist
                )),

                DriveCommands.snapToPoseUntilInTolerance(
                        robot.drive,
                        () -> FieldPositions.getAllianceReefScoringTarget(branch),
                        () -> snapTolerance
                ),

                Commands.waitUntil(robot.superstructure::isInTolerance)
                        .withTimeout(1.5),

                Commands.print("Would eject coral now, but we don't have an outtake yet!")
        );
    }
}
