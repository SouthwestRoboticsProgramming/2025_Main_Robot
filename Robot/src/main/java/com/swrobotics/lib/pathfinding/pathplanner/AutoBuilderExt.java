package com.swrobotics.lib.pathfinding.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import com.swrobotics.lib.pathfinding.PathEnvironment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Collections;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public final class AutoBuilderExt {
    private static PathfinderExt pathfinder;

    private static Supplier<Pose2d> poseSupplier;
    private static Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
    private static BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
    private static PathFollowingController controller;
    private static RobotConfig robotConfig;
    private static Subsystem[] driveRequirements;

    public static void configure(
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose,
            Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
            PathFollowingController controller,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlipPath,
            PathfinderExt pathfinder,
            Subsystem... driveRequirements
    ) {
        AutoBuilder.configure(
                poseSupplier, resetPose, robotRelativeSpeedsSupplier, output,
                controller, robotConfig, shouldFlipPath, driveRequirements);

        AutoBuilderExt.pathfinder = pathfinder;

        AutoBuilderExt.poseSupplier = poseSupplier;
        AutoBuilderExt.robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
        AutoBuilderExt.output = output;
        AutoBuilderExt.controller = controller;
        AutoBuilderExt.robotConfig = robotConfig;
        AutoBuilderExt.driveRequirements = driveRequirements;
    }

    public static PathfinderExt getPathfinder() {
        return pathfinder;
    }

    public static Command pathfindToClosestPose(PathEnvironment env, List<Pose2d> poses, PathConstraints constraints, double goalEndVelocity) {
        return new PathfindingCommandExt(
                env,
                poses,
                constraints,
                goalEndVelocity,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                driveRequirements
        );
    }

    public static Command pathfindToClosestPoseFlipped(PathEnvironment env, List<Pose2d> poses, PathConstraints constraints, double goalEndVelocity) {
        List<Pose2d> flipped = new ArrayList<>(poses.size());
        for (Pose2d pose : poses) {
            flipped.add(FlippingUtil.flipFieldPose(pose));
        }

        return Commands.either(
                pathfindToClosestPose(env, flipped, constraints, goalEndVelocity),
                pathfindToClosestPose(env, poses, constraints, goalEndVelocity),
                AutoBuilder::shouldFlip
        );
    }

    public static Command pathfindToPose(PathEnvironment env, Pose2d pose, PathConstraints constraints) {
        return pathfindToPose(env, pose, constraints, 0);
    }

    public static Command pathfindToPose(PathEnvironment env, Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
        return pathfindToClosestPose(env, Collections.singletonList(pose), constraints, goalEndVelocity);
    }

    public static Command pathfindToClosestPose(PathEnvironment env, List<Pose2d> poses, PathConstraints constraints) {
        return pathfindToClosestPose(env, poses, constraints, 0);
    }

    public static Command pathfindToPoseFlipped(PathEnvironment env, Pose2d pose, PathConstraints constraints) {
        return pathfindToPoseFlipped(env, pose, constraints, 0);
    }

    public static Command pathfindToPoseFlipped(PathEnvironment env, Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
        return pathfindToClosestPoseFlipped(env, Collections.singletonList(pose), constraints, goalEndVelocity);
    }

    public static Command pathfindToClosestPoseFlipped(PathEnvironment env, List<Pose2d> poses, PathConstraints constraints) {
        return pathfindToClosestPoseFlipped(env, poses, constraints, 0);
    }
}
