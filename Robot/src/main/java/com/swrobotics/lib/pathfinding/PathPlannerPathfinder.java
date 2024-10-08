package com.swrobotics.lib.pathfinding;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * PathPlanner pathfinding implementation using the arc pathfinder
 */
// TODO: Do we want to multithread? Depends on pathfinder's performance on RoboRIO
public final class PathPlannerPathfinder implements Pathfinder {
    private static PathEnvironment env;

    /**
     * Sets the environment for paths to be found within. An environment must
     * be set before any paths are requested.
     *
     * @param env new environment
     */
    public static void setEnvironment(PathEnvironment env) {
        PathPlannerPathfinder.env = env;
    }

    /**
     * Converts the bezier points returned from the pathfinder into Waypoints
     * for PathPlanner.
     *
     * @param bezier bezier points from pathfinder
     * @return waypoints for PathPlanner
     */
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

    private Translation2d startPos, goalPos;
    private boolean paramsChanged;

    public PathPlannerPathfinder() {
        paramsChanged = false;
    }

    @Override
    public boolean isNewPathAvailable() {
        if (env == null)
            DriverStation.reportError("No pathfinding environment set", false);

        // New path is always available - we can compute it in less than one periodic cycle
        return paramsChanged
                && env != null
                && startPos != null
                && goalPos != null;
    }

    @Override
    public void setStartPosition(Translation2d startPos) {
        paramsChanged = true;
        this.startPos = startPos;
    }

    @Override
    public void setGoalPosition(Translation2d goalPos) {
        paramsChanged = true;
        this.goalPos = goalPos;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints pathConstraints, GoalEndState goalEndState) {
        paramsChanged = false;

        List<Translation2d> bezierPoints = env.findPath(startPos, goalPos);

        Translation2d[] logPath;
        if (bezierPoints == null) {
            // PathPlanner won't accept impossible result, so give it a straight line as a fallback
            // This can only happen if the target is inside an obstacle, which is pretty
            // easy to avoid doing
            DriverStation.reportWarning("Pathfinding: No path found", false);
            bezierPoints = List.of(startPos, startPos, goalPos, goalPos);

            logPath = new Translation2d[0];
        } else {
            logPath = new Translation2d[bezierPoints.size()];
            bezierPoints.toArray(logPath);
        }

        Logger.recordOutput("Pathfinding/Start Position", startPos);
        Logger.recordOutput("Pathfinding/Goal Position", goalPos);
        Logger.recordOutput("Pathfinding/Path", logPath);

        List<Waypoint> waypoints = bezierPointsToWaypoints(bezierPoints);
        return new PathPlannerPath(waypoints, pathConstraints, null, goalEndState);
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Translation2d robotPos) {
        throw new UnsupportedOperationException("Can't set dynamic obstacles");
    }
}
