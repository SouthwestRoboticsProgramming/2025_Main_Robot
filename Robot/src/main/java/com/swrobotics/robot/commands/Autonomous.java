package com.swrobotics.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.pathfinding.pathplanner.AutoBuilderExt;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import com.swrobotics.robot.config.PathEnvironments;
import com.swrobotics.robot.subsystems.outtake.CoralHandlingSubsystem;
import com.swrobotics.robot.subsystems.superstructure.SuperstructureSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;

public final class Autonomous {
    /*
     * Reef layout:
     *
     *  [ Barge/Cage ]
     *       7  6
     *    8 /----\ 5
     *   9 /      \ 4
     *  10 \      / 3
     *   11 \----/ 2
     *       0  1
     * [Driver stations]
     *
     */

    public static Command behindReef1Piece(RobotContainer robot) {
        return scoreAt(robot, 6, 4);
    }

    public static Command leftSide4Piece(RobotContainer robot) {
        return Commands.sequence(
                scoreAt(robot, 9, 4),
                humanPlayerPickupLeft(robot),
                scoreAt(robot, 0, 4),
                humanPlayerPickupLeft(robot),
                scoreAt(robot, 11, 4),
                humanPlayerPickupLeft(robot),
                scoreAt(robot, 10, 4)
        );
    }

    private static PathConstraints getPathConstraints() {
        return new PathConstraints(
                Constants.kAutoMaxDriveSpeed,
                Constants.kAutoMaxDriveAccel,
                Units.rotationsToRadians(Constants.kAutoMaxTurnSpeed),
                Units.rotationsToRadians(Constants.kAutoMaxTurnAccel)
        );
    }

    private static Command scoreAt(RobotContainer robot, int position, int height) {
        PathConstraints constraints = getPathConstraints();

        return Commands.sequence(
                robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.forScoring(height)),

                Commands.print("GOING"),
                AutoBuilderExt.pathfindToPoseFlipped(
                        PathEnvironments.kFieldWithAutoGamePieces,
                        FieldPositions.getBlueReefScoringTarget(position),
                        constraints
                ).until(() -> robot.drive.isCloseTo(
                        FieldPositions.getAllianceReefScoringTarget(position).getTranslation(),
                        Constants.kAutoSwitchToSnapDist
                )),

                        Commands.print("SNAPPY"),

                DriveCommands.snapToPose(robot.drive, () -> FieldPositions.getAllianceReefScoringTarget(position))
                        .until(() -> {
                            Pose2d pose = robot.drive.getEstimatedPose();

                            Pose2d allianceTarget = FieldPositions.getAllianceReefScoringTarget(position);
                            boolean xy = pose.getTranslation().getDistance(allianceTarget.getTranslation())
                                    < Constants.kAutoScoreXYTolerance.get();
                            boolean angle = MathUtil.absDiffRad(pose.getRotation().getDegrees(), allianceTarget.getRotation().getDegrees())
                                    < Constants.kAutoScoreAngleTolerance.get();
                            boolean superstructure = robot.superstructure.isInTolerance();

                            return xy && angle && superstructure;
                        })
                        .withTimeout(Constants.kAutoToleranceTimeout),
                Commands.print("SCOREY"),

                robot.coralHandler.score(Constants.kAutoCoralEjectTime)
                ,Commands.print("YAYYY")
        );
    }

    private static Command humanPlayerPickupLeft(RobotContainer robot) {
        return humanPlayerPickup(robot, FieldPositions.getLeftCoralStationPickup());
    }

    private static Command humanPlayerPickupRight(RobotContainer robot) {
        return humanPlayerPickup(robot, FieldPositions.getRightCoralStationPickup());
    }

    private static Command humanPlayerPickup(RobotContainer robot, Pose2d pickupPose) {
        PathConstraints constraints = getPathConstraints();

        return Commands.sequence(
                Commands.print("GOING HP"),
                AutoBuilderExt.pathfindToPoseFlipped(
                        PathEnvironments.kFieldWithAutoGamePieces,
                        pickupPose,
                        constraints
                ).alongWith(Commands.sequence(
                        Commands.waitSeconds(Constants.kAutoElevatorDownDelay),
                        robot.coralHandler.commandSetStateOnce(CoralHandlingSubsystem.State.INTAKE),
                        robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.RECEIVE_CORAL_FROM_INDEXER)
                )),

                Commands.print("WAITING FOR HP"),
                // No timeout because it's better to wait long then leave without coral
                Commands.waitUntil(() -> robot.coralHandler.hasPiece() || RobotBase.isSimulation())
        );
    }
}
