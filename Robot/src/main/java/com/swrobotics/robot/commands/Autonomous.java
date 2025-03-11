package com.swrobotics.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import com.swrobotics.lib.pathfinding.pathplanner.AutoBuilderExt;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import com.swrobotics.robot.config.PathEnvironments;
import com.swrobotics.robot.subsystems.outtake.OuttakeSubsystem;
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
        private final Pose2d start;
        private final Pose2d end;
        private final List<RotationTarget> rotationTargets;
        private final PathConstraints globalConstraints;
        private final IdealStartingState startingState;
        private final GoalEndState goalEndState;
        private double curveStart;
        private double curveEnd;

        public SegmentBuilder(Pose2d start, Pose2d end, PathConstraints constraints) {
            this.start = start;
            this.end = end;
            rotationTargets = new ArrayList<>();
            globalConstraints = constraints;
            startingState = new IdealStartingState(0, start.getRotation());
            goalEndState = new GoalEndState(0, end.getRotation());
            curveStart = 0;
            curveEnd = 0;
        }

        public SegmentBuilder addRotationTarget(RotationTarget rotationTarget) {
            rotationTargets.add(rotationTarget);
            return this;
        }

        public SegmentBuilder withCurveStart(double curveStart) {
            this.curveStart = curveStart;
            return this;
        }

        public SegmentBuilder withCurveEnd(double curveEnd) {
            this.curveEnd = curveEnd;
            return this;
        }

        public PathPlannerPath build() {
            Translation2d startPos = start.getTranslation();
            Translation2d endPos = end.getTranslation();
            Translation2d third = endPos.minus(startPos).div(3);

            return new PathPlannerPath(
                    List.of(
                            new Waypoint(null, startPos, startPos.plus(third.rotateBy(Rotation2d.fromDegrees(curveStart)))),
                            new Waypoint(endPos.minus(third.rotateBy(Rotation2d.fromDegrees(curveEnd))), endPos, null)
                    ),
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

    private static final double kAlignTimeout = 0.2;
    private static final double kScoreTimeout = 0.4;
    private static final double kElevatorUpEarlyTime = 0.4;

    private static Command doScore(RobotContainer robot, PathPlannerPath toScoringPosition, Pose2d scoringPosition) {
        double pathTime = toScoringPosition.getIdealTrajectory(Constants.kPathPlannerRobotConfig)
                .orElseThrow()
                .getTotalTimeSeconds();

        return Commands.sequence(
                Commands.parallel(
                        Commands.sequence(
                                AutoBuilder.followPath(toScoringPosition),
                                DriveCommands.snapToPose(robot.drive, robot.lights, () -> Constants.kField.flipPoseForAlliance(scoringPosition))
                        ),
                        Commands.sequence(
                                Commands.parallel(
                                        RobotBase.isReal()
                                            ? Commands.waitUntil(robot.outtake::hasPiece)
                                            : Commands.waitSeconds(0.2),

                                        // Bring elevator up as late as possible
                                        Commands.defer(() -> Commands.waitSeconds(
                                                pathTime - kElevatorUpEarlyTime - robot.superstructure.calculateIndexerToL4TravelTime()
                                        ), Collections.emptySet())
                                ),

                                robot.superstructure.commandSetState(SuperstructureSubsystem.State.SCORE_L4)
                        )
                ).until(() -> {
                    Pose2d pose = robot.drive.getEstimatedPose();

                    Pose2d allianceTarget = Constants.kField.flipPoseForAlliance(scoringPosition);
                    boolean xy = pose.getTranslation().getDistance(allianceTarget.getTranslation())
                            < Constants.kAutoScoreXYTolerance.get();
                    boolean angle = MathUtil.absDiffRad(pose.getRotation().getDegrees(), allianceTarget.getRotation().getDegrees())
                            < Constants.kAutoScoreAngleTolerance.get();
                    boolean superstructure = robot.superstructure.isInTolerance();

                    Logger.recordOutput("Auto/XY In Tolerance", xy);
                    Logger.recordOutput("Auto/Angle In Tolerance", angle);
                    Logger.recordOutput("Auto/Superstructure In Tolerance", superstructure);

                    return xy && angle && superstructure;
                }).withTimeout(pathTime + kAlignTimeout),

                Commands.race(
                        // Continue snapping in case it got timed out above
                        DriveCommands.snapToPose(robot.drive, robot.lights, () -> Constants.kField.flipPoseForAlliance(scoringPosition)),

                        robot.outtake.commandSetState(OuttakeSubsystem.State.SCORE)
                                .until(() -> !robot.outtake.hasPiece() && RobotBase.isReal())
                                .withTimeout(kScoreTimeout)
                )
        );
    }

    private static final double kElevatorDownDelay = 0.5;
    private static final double kHumanPlayerWaitTimeout = 1;

    private static Command doHumanPlayerPickup(RobotContainer robot, PathPlannerPath toCoralStation) {
        return Commands.sequence(
                Commands.parallel(
                        AutoBuilder.followPath(toCoralStation),
                        Commands.sequence(
                                Commands.waitSeconds(kElevatorDownDelay),
                                robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.RECEIVE_CORAL_FROM_INDEXER),
                                robot.outtake.commandSetStateOnce(OuttakeSubsystem.State.INTAKE_CORAL)
                        )
                ),
                Commands.waitUntil(robot.outtake::hasPiece)
                        .withTimeout(kHumanPlayerWaitTimeout)
        );
    }

    public static Command fourPieceV2(RobotContainer robot, boolean rightSide) {
        Pose2d hp = rightSide
                ? new Pose2d(new Translation2d(1.371, Constants.kField.getHeight() - 7.289), Rotation2d.fromDegrees(54.013))
                : new Pose2d(new Translation2d(1.371, 7.289), Rotation2d.fromDegrees(-54.013));

        Pose2d score1 = FieldPositions.getBlueReefScoringTarget(rightSide ? 5 : 8);
        Pose2d score2 = FieldPositions.getBlueReefScoringTarget(rightSide ? 2 : 11);
        Pose2d score3 = FieldPositions.getBlueReefScoringTarget(rightSide ? 3 : 10);
        Pose2d score4 = FieldPositions.getBlueReefScoringTarget(rightSide ? 4 : 9);
        Pose2d start = new Pose2d(new Translation2d(FieldPositions.kStartingLineX, score1.getY()), Rotation2d.k180deg);

        // Rotation targets are to prevent rotation when touching the reef
        PathConstraints constraints = getPathConstraints();
        PathPlannerPath startToScore1 = new SegmentBuilder(start, score1, constraints).build();
        PathPlannerPath score1ToHP = new SegmentBuilder(score1, hp, constraints)
                .withCurveStart(rightSide ? 30 : -30)
                .addRotationTarget(new RotationTarget(0.2, score1.getRotation()))
                .build();
        PathPlannerPath hpToScore2 = new SegmentBuilder(hp, score2, constraints).build();
        PathPlannerPath score2ToHP = new SegmentBuilder(score2, hp, constraints).build();
        PathPlannerPath hpToScore3 = new SegmentBuilder(hp, score3, constraints).build();
        PathPlannerPath score3ToHP = new SegmentBuilder(score3, hp, constraints).build();
        PathPlannerPath hpToScore4 = new SegmentBuilder(hp, score4, constraints)
                .withCurveEnd(rightSide ? 30 : -30)
                .addRotationTarget(new RotationTarget(0.8, score4.getRotation()))
                .build();

        Command sequence = Commands.sequence(
                robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.SCORE_L4),
                Commands.waitSeconds(0.5),
                doScore(robot, startToScore1, score1),
                doHumanPlayerPickup(robot, score1ToHP),
                doScore(robot, hpToScore2, score2),
                doHumanPlayerPickup(robot, score2ToHP),
                doScore(robot, hpToScore3, score3),
                doHumanPlayerPickup(robot, score3ToHP),
                doScore(robot, hpToScore4, score4),
                backUp(robot)
        );

        if (RobotBase.isSimulation()) {
            sequence = sequence.beforeStarting(Commands.runOnce(
                    () -> robot.drive.resetPose(Constants.kField.flipPoseForAlliance(start))));
        }

        return sequence;
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

                DriveCommands.snapToPose(robot.drive, robot.lights, () -> FieldPositions.getAllianceReefScoringTarget(position))
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

                robot.outtake.score(Constants.kAutoCoralEjectTime)
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
                        robot.outtake.commandSetStateOnce(OuttakeSubsystem.State.INTAKE_CORAL),
                        robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.RECEIVE_CORAL_FROM_INDEXER)
                )),

                Commands.print("WAITING FOR HP"),
                // No timeout because it's better to wait long then leave without coral
                Commands.waitUntil(() -> robot.outtake.hasPiece() || RobotBase.isSimulation())

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
                        robot.outtake.commandSetStateOnce(OuttakeSubsystem.State.INTAKE_CORAL),
                        robot.superstructure.commandSetStateOnce(SuperstructureSubsystem.State.RECEIVE_CORAL_FROM_INDEXER)
                )),

                Commands.print("WAITING FOR HP"),
                // No timeout because it's better to wait long then leave without coral
                Commands.waitUntil(() -> robot.outtake.hasPiece() || RobotBase.isSimulation())

                        // Quick fix for Week 0 since we weren't able to get the robot to drive to the right spot
                        // FIXME: Actually go to the spot
                        .raceWith(DriveCommands.driveRobotRelative(robot.drive, () -> new Translation2d(-0.6, 0), () -> 0.0))
        );
    }

    private static Command backUp(RobotContainer robot) {
        return DriveCommands.driveRobotRelative(robot.drive, () -> new Translation2d(-1, 0), () -> 0.0)
                .withTimeout(0.8);
    }
}
