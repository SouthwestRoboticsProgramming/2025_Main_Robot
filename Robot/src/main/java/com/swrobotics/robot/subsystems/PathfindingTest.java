package com.swrobotics.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.pathfinding.PathResult;
import com.swrobotics.lib.pathfinding.pathplanner.AutoBuilderExt;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.config.PathEnvironments;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class PathfindingTest extends SubsystemBase {
    private final SwerveDriveSubsystem drive;

    public PathfindingTest(SwerveDriveSubsystem drive) {
        this.drive = drive;

       PathEnvironments.kFieldWithAutoGamePieces.getDebug().plot(FieldView.pathfindingDebug);

        FieldView.pathfindingGoal.setPose(new Pose2d(new Translation2d(2, 2), new Rotation2d()));
        FieldView.pathfindingGoal2.setPose(new Pose2d(new Translation2d(2, 3), new Rotation2d()));
    }

    public Command getFollowCommand() {
        System.out.println("Pathfinding...");
        Pose2d goal = FieldView.pathfindingGoal.getPose();

        PathConstraints constraints = new PathConstraints(
                Constants.kDriveMaxAchievableSpeed,
                Constants.kDriveControlMaxAccel,
                Units.rotationsToRadians(Constants.kDriveControlMaxTurnSpeed),
                Units.rotationsToRadians(Constants.kDriveControlMaxTurnSpeed / 0.3));

        return AutoBuilderExt.pathfindToPose(
                PathEnvironments.kFieldWithAutoGamePieces,
                goal,
                constraints
        );
    }

    @Override
    public void periodic() {
        Pose2d goal = FieldView.pathfindingGoal.getPose();
        Pose2d goal2 = FieldView.pathfindingGoal2.getPose();

        double startTime = Timer.getFPGATimestamp();
        PathResult result = PathEnvironments.kFieldWithAutoGamePieces
               .findPathToClosest(drive.getEstimatedPose().getTranslation(), Arrays.asList(goal.getTranslation(), goal2.getTranslation()));

        double endTime = Timer.getFPGATimestamp();
        Logger.recordOutput("Pathfinding/Calc time (ms)", (endTime - startTime) * 1000);

        if (result != null) {
            List<Translation2d> bezier = result.bezierPoints();
            List<Pose2d> poses = new ArrayList<>();

            // Visualize Bezier curves
            Translation2d p0 = bezier.get(0);
            for (int i = 1; i < bezier.size(); i += 3) {
                Translation2d p1 = bezier.get(i);
                Translation2d p2 = bezier.get(i + 1);
                Translation2d p3 = bezier.get(i + 2);

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
