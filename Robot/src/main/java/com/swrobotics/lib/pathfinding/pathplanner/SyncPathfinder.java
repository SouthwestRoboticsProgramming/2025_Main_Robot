package com.swrobotics.lib.pathfinding.pathplanner;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.swrobotics.lib.pathfinding.PathEnvironment;
import com.swrobotics.lib.pathfinding.PathResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public final class SyncPathfinder implements PathfinderExt {
    private PathEnvironment environment;
    private Translation2d startPos;
    private List<Pose2d> goalPoses;
    private boolean paramsChanged;

    public SyncPathfinder() {
        paramsChanged = false;
    }

    @Override
    public boolean isNewPathAvailable() {
        return paramsChanged
                && environment != null
                && startPos != null
                && goalPoses != null;
    }

    @Override
    public void setEnvironment(PathEnvironment env) {
        paramsChanged = true;
        this.environment = env;
    }

    @Override
    public void setStartPosition(Translation2d startPos) {
        paramsChanged = true;
        this.startPos = startPos;
    }

    @Override
    public void setGoalPoses(List<Pose2d> goalPoses) {
        paramsChanged = true;
        this.goalPoses = goalPoses;
    }

    @Override
    public Path getCurrentPath(PathConstraints pathConstraints, double goalVelocityMPS) {
        paramsChanged = false;

        List<Translation2d> goalPositions = new ArrayList<>(goalPoses.size());
        for (Pose2d pose : goalPoses) {
            goalPositions.add(pose.getTranslation());
        }

        int goalIndex = 0;
        List<Translation2d> bezierPoints;
        Translation2d[] logPath;

        PathResult result = environment.findPathToClosest(startPos, goalPositions);
        if (result == null) {
            double closestDist = Double.POSITIVE_INFINITY;
            for (int i = 0; i < goalPositions.size(); i++) {
                double dist = startPos.getDistance(goalPositions.get(i));
                if (dist < closestDist) {
                    closestDist = dist;
                    goalIndex = i;
                }
            }

            // Fall back to straight line
            Translation2d goal = goalPositions.get(goalIndex);
            bezierPoints = List.of(startPos, startPos, goal, goal);
            logPath = new Translation2d[0];
        } else {
            goalIndex = result.goalIndex();
            bezierPoints = result.bezierPoints();
            logPath = new Translation2d[result.bezierPoints().size()];
            result.bezierPoints().toArray(logPath);
        }

        Pose2d[] logGoalPoses = new Pose2d[goalPoses.size()];
        goalPoses.toArray(logGoalPoses);

        Logger.recordOutput("Pathfinding/Start Position", startPos);
        Logger.recordOutput("Pathfinding/Goal Poses", logGoalPoses);
        Logger.recordOutput("Pathfinding/Path", logPath);

        List<Waypoint> waypoints = bezierPointsToWaypoints(bezierPoints);
        Rotation2d goalRotation = goalPoses.get(goalIndex).getRotation();

        return new Path(
                new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(goalVelocityMPS, goalRotation)),
                goalIndex
        );
    }

    public static List<Waypoint> bezierPointsToWaypoints(List<Translation2d> bezier) {
        // Bezier points alternate A, C, C, A, C, C, A, C, C, A

        int bezierCount = bezier.size();
        if (bezierCount % 3 != 1)
            throw new IllegalArgumentException("Bezier path is incomplete");

        List<Waypoint> out = new ArrayList<>();
        for (int i = -1; i < bezierCount; i += 3) {
            Translation2d prevControl = i >= 0 ? bezier.get(i) : null;
            Translation2d anchor = bezier.get(i + 1);
            Translation2d nextControl = i + 2 < bezierCount ? bezier.get(i + 2) : null;

            out.add(new Waypoint(prevControl, anchor, nextControl));
        }

        return out;
    }
}
