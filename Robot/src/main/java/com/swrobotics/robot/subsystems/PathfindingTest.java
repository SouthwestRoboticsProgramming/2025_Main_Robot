package com.swrobotics.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.pathfinding.*;
import com.swrobotics.robot.pathfinding.async.AsyncLoggedPathPlannerPathfinder;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public final class PathfindingTest extends SubsystemBase {
    private final SwerveDriveSubsystem drive;
    public final PathEnvironment environment;

    public PathfindingTest(SwerveDriveSubsystem drive) {
        this.drive = drive;

        try {
            List<Obstacle> obstacles = Obstacle.loadObstaclesFromJson(Constants.kPathfindingJson);
            // Test overlapping obstacles
//            obstacles.add(new Rectangle(
//                    new Translation2d(8, 4),
//                    new Translation2d(6, 4),
//                    0));
            environment = new PathEnvironment(obstacles, Constants.kRobotRadius +
                    Constants.kPathfindingTolerance);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        environment.getDebug().plot(FieldView.pathfindingDebug);

        FieldView.pathfindingGoal.setPose(new Pose2d(new Translation2d(2, 2), new Rotation2d()));
    }

    public Command getFollowCommand() {
        System.out.println("Pathfinding...");
        Pose2d goal = FieldView.pathfindingGoal.getPose();

        PathConstraints constraints = new PathConstraints(
                    Constants.kMaxAchievableSpeed,
                    Constants.kMaxAchievableSpeed / 0.7,
                    Constants.kDriveControlMaxTurnSpeed,
                    Constants.kDriveControlMaxTurnSpeed / 0.2);

        return Commands.sequence(
            Commands.runOnce(() -> AsyncLoggedPathPlannerPathfinder.setEnvironment(environment)),
            AutoBuilder.pathfindToPose(goal, constraints)
        );
    }

    @Override
    public void periodic() {
        Pose2d goal = FieldView.pathfindingGoal.getPose();
        Translation2d goalPos = goal.getTranslation();

        double startTime = Logger.getRealTimestamp();
        List<Translation2d> path = environment.findPath(drive.getEstimatedPose().getTranslation(), goalPos);

        double endTime = Logger.getRealTimestamp(); // Gives time in microseconds
        Logger.recordOutput("Pathfinding/Calc time (ms)", (endTime - startTime) / 1000);

        if (path != null) {
            List<Pose2d> poses = new ArrayList<>();

            // Visualize Bezier curves
            Translation2d p0 = path.get(0);
            for (int i = 1; i < path.size(); i += 3) {
                Translation2d p1 = path.get(i);
                Translation2d p2 = path.get(i + 1);
                Translation2d p3 = path.get(i + 2);

                // Add poses at a few points along the curve to approximate it visually
                for (int j = i == 1 ? 0 : 1; j <= 10; j++) {
                    double f = j / 10.0;

                    Translation2d q0 = p0.interpolate(p1, f);
                    Translation2d q1 = p1.interpolate(p2, f);
                    Translation2d q2 = p2.interpolate(p3, f);
                    Translation2d r0 = q0.interpolate(q1, f);
                    Translation2d r1 = q1.interpolate(q2, f);
                    Translation2d s = r0.interpolate(r1, f);

                    poses.add(new Pose2d(s, new Rotation2d()));
                }

                p0 = p3;
            }
            FieldView.pathfindingPath.setPoses(poses);

            Logger.recordOutput("Pathfinding/Live Path", poses.toArray(new Pose2d[0]));
        } else {
            // No poses
            FieldView.pathfindingPath.setPoses();

            Logger.recordOutput("Pathfinding/Live Path", new Pose2d[0]);
        }
    }
}
