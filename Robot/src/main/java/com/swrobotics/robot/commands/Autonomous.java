package com.swrobotics.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import com.swrobotics.lib.pathfinding.pathplanner.AutoBuilderExt;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import com.swrobotics.robot.config.PathEnvironments;
import com.swrobotics.robot.subsystems.outtake.CoralOuttakeSubsystem;
import com.swrobotics.robot.subsystems.superstructure.SuperstructureSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

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

    private record ScoringPosition(int position, int height) {}

    public static Command goSideways5Meters(RobotContainer robot) {
        return Commands.defer(() -> AutoBuilderExt.pathfindToPose(
                PathEnvironments.kField,
                new Pose2d(
                        robot.drive.getEstimatedPose().getTranslation().plus(new Translation2d(0, -5)),
                        robot.drive.getEstimatedPose().getRotation()
                ),
                getPathConstraints(),
                0
        ),
                Set.of(robot.drive));
    }

    public static Command everythingRandomly(RobotContainer robot) {
        List<ScoringPosition> positions = new ArrayList<>();
        for (int position = 0; position < 12; position++) {
            for (int height = 2; height <= 4; height++) {
                positions.add(new ScoringPosition(position, height));
            }
        }

        List<ScoringPosition> randomOrder = new ArrayList<>();
        while (!positions.isEmpty()) {
            randomOrder.add(positions.remove((int) (Math.random() * positions.size())));
        }

        List<Command> sequence = new ArrayList<>();
        for (ScoringPosition position : randomOrder) {
            sequence.add(scoreAt(robot, position.position, position.height));
            sequence.add(humanPlayerPickupClosest(robot));
        }
        sequence.add(backUp(robot));

        return new SequentialCommandGroup(sequence.toArray(new Command[0]));
    }

    public static Command behindReef1PieceLeft(RobotContainer robot) {
        return Commands.sequence(
                // Give the elevator a bit to go up before we drive into the reef
                robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.SCORE_L4),
                Commands.waitSeconds(1),

                scoreAt(robot, 7, 4),
                backUp(robot)
        );
    }

    public static Command behindReef1PieceRight(RobotContainer robot) {
        return Commands.sequence(
                robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.SCORE_L4),
                Commands.waitSeconds(1),
                scoreAt(robot, 6, 4),
                backUp(robot)
        );
    }

    private static final class SegmentBuilder {
        private final List<Waypoint> waypoints;
        private final List<RotationTarget> rotationTargets;
        private final PathConstraints globalConstraints;
        private final IdealStartingState startingState;
        private final GoalEndState goalEndState;

        public SegmentBuilder(Pose2d start, Pose2d end, PathConstraints constraints) {
            waypoints = List.of(
                    new Waypoint(null, start.getTranslation(), end.getTranslation()),
                    new Waypoint(start.getTranslation(), end.getTranslation(), null)
            );
            rotationTargets = new ArrayList<>();
            globalConstraints = constraints;
            startingState = new IdealStartingState(0, start.getRotation());
            goalEndState = new GoalEndState(0, end.getRotation());
        }

        public SegmentBuilder addRotationTarget(RotationTarget rotationTarget) {
            rotationTargets.add(rotationTarget);
            return this;
        }

        public PathPlannerPath build() {
            return new PathPlannerPath(
                    waypoints,
                    rotationTargets,
                    Collections.emptyList(),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    globalConstraints,
                    startingState,
                    goalEndState,
                    false
            );
        }
    }

    private static final double kScoreTime = 0.4;

    private static Command doScore(RobotContainer robot, PathPlannerPath toScoringPosition) {
        return Commands.sequence(
                // TODO: Snap after following path
                AutoBuilder.followPath(toScoringPosition)
                        .deadlineFor(Commands.sequence(
                                Commands.waitUntil(robot.coralOuttake::hasPiece),
                                robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L4)
                                        .until(robot.superstructure::isInTolerance)
                        )),
                robot.coralOuttake.score(kScoreTime)
        );
    }
    

    public static Command leftSide4PieceV2(RobotContainer robot) {
        Pose2d hp = new Pose2d(new Translation2d(1.561, 7.315), Rotation2d.fromDegrees(-54.013));
        Pose2d score1 = FieldPositions.getBlueReefScoringTarget(8);
        Pose2d score2 = FieldPositions.getBlueReefScoringTarget(11);
        Pose2d score3 = FieldPositions.getBlueReefScoringTarget(10);
        Pose2d score4 = FieldPositions.getBlueReefScoringTarget(9);
        Pose2d start = new Pose2d(new Translation2d(FieldPositions.kStartingLineX, score1.getY()), Rotation2d.k180deg);

        PathConstraints constraints = getPathConstraints();
        PathPlannerPath startToScore1 = new SegmentBuilder(start, score1, constraints).build();
        PathPlannerPath score1ToHP = new SegmentBuilder(score1, hp, constraints)
                .addRotationTarget(new RotationTarget(0.25, score1.getRotation()))
                .build();
        PathPlannerPath hpToScore2 = new SegmentBuilder(hp, score2, constraints).build();
        PathPlannerPath score2ToHP = new SegmentBuilder(score2, hp, constraints).build();
        PathPlannerPath hpToScore3 = new SegmentBuilder(hp, score3, constraints).build();
        PathPlannerPath score3ToHp = new SegmentBuilder(score3, hp, constraints).build();
        PathPlannerPath hpToScore4 = new SegmentBuilder(hp, score4, constraints)
                .addRotationTarget(new RotationTarget(0.80, score4.getRotation()))
                .build();

        return Commands.sequence(

        );
    }

    public static Command leftSide4Piece(RobotContainer robot) {
        return Commands.sequence(
                scoreAt(robot, 9, 4),
                humanPlayerPickupLeft(robot),
                scoreAt(robot, 10, 4),
                humanPlayerPickupLeft(robot),
                scoreAt(robot, 11, 4),
                humanPlayerPickupLeft(robot),
                scoreAt(robot, 0, 4),
                backUp(robot)
        );
    }

    public static Command rightSide4Piece(RobotContainer robot) {
        return Commands.sequence(
                scoreAt(robot, 4, 4),
                humanPlayerPickupRight(robot),
                scoreAt(robot, 3, 4),
                humanPlayerPickupRight(robot),
                scoreAt(robot, 2, 4),
                humanPlayerPickupRight(robot),
                scoreAt(robot, 1, 4),
                backUp(robot)
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

                            Logger.recordOutput("Auto/XY In Tolerance", xy);
                            Logger.recordOutput("Auto/Angle In Tolerance", angle);
                            Logger.recordOutput("Auto/Superstructure In Tolerance", superstructure);

                            return xy && angle && superstructure;
                        })
                        .withTimeout(Constants.kAutoToleranceTimeout),
                Commands.print("SCOREY"),

                robot.coralOuttake.score(Constants.kAutoCoralEjectTime)
                ,Commands.print("YAYYY")
        );
    }

    private static Command humanPlayerPickupLeft(RobotContainer robot) {
        return humanPlayerPickup(robot, FieldPositions.getLeftCoralStationPickup());
    }

    private static Command humanPlayerPickupRight(RobotContainer robot) {
        return humanPlayerPickup(robot, FieldPositions.getRightCoralStationPickup());
    }

    private static Command humanPlayerPickupClosest(RobotContainer robot) {
        PathConstraints constraints = getPathConstraints();

        return Commands.sequence(
                Commands.print("GOING HP"),
                AutoBuilderExt.pathfindToClosestPoseFlipped(
                        PathEnvironments.kFieldWithAutoGamePieces,
                        List.of(
                                FieldPositions.getLeftCoralStationPickup(),
                                FieldPositions.getRightCoralStationPickup()
                        ),
                        constraints,
                        null
                ).alongWith(Commands.sequence(
                        Commands.waitSeconds(Constants.kAutoElevatorDownDelay),
                        robot.coralOuttake.commandSetStateOnce(CoralOuttakeSubsystem.State.INTAKE),
                        robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.RECEIVE_CORAL_FROM_INDEXER)
                )),

                Commands.print("WAITING FOR HP"),
                // No timeout because it's better to wait long then leave without coral
                Commands.waitUntil(() -> robot.coralOuttake.hasPiece() || RobotBase.isSimulation())

                        // Quick fix for Week 0 since we weren't able to get the robot to drive to the right spot
                        // FIXME: Actually go to the spot
                        .raceWith(DriveCommands.driveRobotRelative(robot.drive, () -> new Translation2d(-0.6, 0), () -> 0.0))
        );
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
                        robot.coralOuttake.commandSetStateOnce(CoralOuttakeSubsystem.State.INTAKE),
                        robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.RECEIVE_CORAL_FROM_INDEXER)
                )),

                Commands.print("WAITING FOR HP"),
                // No timeout because it's better to wait long then leave without coral
                Commands.waitUntil(() -> robot.coralOuttake.hasPiece() || RobotBase.isSimulation())

                        // Quick fix for Week 0 since we weren't able to get the robot to drive to the right spot
                        // FIXME: Actually go to the spot
                        .raceWith(DriveCommands.driveRobotRelative(robot.drive, () -> new Translation2d(-0.6, 0), () -> 0.0))
        );
    }

    private static Command backUp(RobotContainer robot) {
        return DriveCommands.driveRobotRelative(robot.drive, () -> new Translation2d(-1, 0), () -> 0.0)
                .withTimeout(0.4);
    }
}
