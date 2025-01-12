package com.swrobotics.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.lib.pathfinding.*;
import com.swrobotics.robot.subsystems.pathfinding.PathEnvironments;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public final class PathfindingTest extends SubsystemBase {
    private final SwerveDriveSubsystem drive;

    public PathfindingTest(SwerveDriveSubsystem drive) {
        this.drive = drive;

       PathEnvironments.kFieldWithAutoGamePieces.getDebug().plot(FieldView.pathfindingDebug);

        FieldView.pathfindingGoal.setPose(new Pose2d(new Translation2d(2, 2), new Rotation2d()));
    }

    public Command getFollowCommand() {
        System.out.println("Pathfinding...");
        Pose2d goal = FieldView.pathfindingGoal.getPose();

        PathConstraints constraints = new PathConstraints(
                    Constants.kDriveMaxAchievableSpeed,
                    Constants.kDriveMaxAchievableSpeed / 0.7,
                    Constants.kDriveControlMaxTurnSpeed,
                    Constants.kDriveControlMaxTurnSpeed / 0.2);

        return Commands.sequence(
            Commands.runOnce(() -> PathPlannerPathfinder.setEnvironment(PathEnvironments.kFieldWithAutoGamePieces)),
            AutoBuilder.pathfindToPose(goal, constraints)
        );
    }
    /*

     */
// PROBLEM IS: Clipping duplicates arcs, having multiple arcs in same spot causes is_segment_passable to fail due to only ignoring one
    // SOLUTION: Fix duplicating arcs
    /*
    Begin goal search
P2A tangents: P=(13.261876106262207, 1.4572594165802002) A=Arc { center: Vec2f { x: 16.328799999999998, y: 2.1971999999999996 }, radius: 1.0234230647113989, min_angle: -3.141592653589793, max_angle: -1.8490724094145665 } cw=-4.14528242664394 ccw=-1.6644218970211682
P2A tangents: P=(13.261876106262207, 1.4572594165802002) A=Arc { center: Vec2f { x: 16.328799999999998, y: 2.1971999999999996 }, radius: 1.0234230647113989, min_angle: 2.0365749458849844, max_angle: -3.141592653589793 } cw=-4.14528242664394 ccw=-1.6644218970211682
P2A tangents: P=(13.261876106262207, 1.4572594165802002) A=Arc { center: Vec2f { x: 16.328799999999998, y: 4.026 }, radius: 1.0234230647113989, min_angle: -3.141592653589793, max_angle: -2.036574945884984 } cw=-3.7564639307353467 ccw=-1.132263338534691
* P2A tangents: P=(13.261876106262207, 1.4572594165802002) A=Arc { center: Vec2f { x: 16.328799999999998, y: 4.026 }, radius: 1.0234230647113989, min_angle: 1.1703324517928593, max_angle: -3.141592653589793 } cw=-3.7564639307353467 ccw=-1.132263338534691
Begin goal search
P2A tangents: P=(15.352389842233839, 2.214223119002058) A=Arc { center: Vec2f { x: 16.328799999999998, y: 4.026 }, radius: 1.0234230647113989, min_angle: -3.141592653589793, max_angle: -2.036574945884984 } cw=-3.1154571203775427 ccw=-1.0147352576291926
P2A tangents: P=(15.352389842233839, 2.214223119002058) A=Arc { center: Vec2f { x: 16.328799999999998, y: 4.026 }, radius: 1.0234230647113989, min_angle: 1.1703324517928593, max_angle: -3.141592653589793 } cw=-3.1154571203775427 ccw=-1.0147352576291926
Begin goal search
P2A tangents: P=(15.255540035914654, 2.215911636645166) A=Arc { center: Vec2f { x: 16.328799999999998, y: 2.1971999999999996 }, radius: 1.0234230647113989, min_angle: -3.141592653589793, max_angle: -1.8490724094145665 } cw=2.8177415790880915 ccw=3.4305784731376456
P2A tangents: P=(15.255540035914654, 2.215911636645166) A=Arc { center: Vec2f { x: 16.328799999999998, y: 2.1971999999999996 }, radius: 1.0234230647113989, min_angle: 2.0365749458849844, max_angle: -3.141592653589793 } cw=2.8177415790880915 ccw=3.4305784731376456
End goal search
     */
    @Override
    public void periodic() {
        Pose2d goal = FieldView.pathfindingGoal.getPose();
        Translation2d goalPos = goal.getTranslation();

        // Failing case
        Translation2d debugStartPos = new Translation2d(1.189, 1.838);
        Translation2d debugGoalPos = new Translation2d(1.239, 7.148);

        double startTime = Timer.getFPGATimestamp();
        List<Translation2d> path = PathEnvironments.kFieldWithAutoGamePieces
                // .findPath(debugStartPos, debugGoalPos);
               .findPath(drive.getEstimatedPose().getTranslation(), goalPos);

        double endTime = Timer.getFPGATimestamp(); // Gives time in microseconds
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
