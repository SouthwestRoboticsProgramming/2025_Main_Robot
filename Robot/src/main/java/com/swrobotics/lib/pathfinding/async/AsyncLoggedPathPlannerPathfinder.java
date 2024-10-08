package com.swrobotics.lib.pathfinding.async;

import java.util.List;

import com.pathplanner.lib.path.Waypoint;
import com.swrobotics.lib.pathfinding.PathPlannerPathfinder;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.logging.Logging;
import com.swrobotics.lib.pathfinding.PathEnvironment;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * PathPlanner pathfinding implementation using the arc pathfinder running in
 * a separate thread, logged using AdvantageKit.
 */
// TODO: Better name?
public final class AsyncLoggedPathPlannerPathfinder implements Pathfinder {
    private static PathEnvironment env = null;

    /**
     * Sets the environment for paths to be found within. An environment must
     * be set before any paths are requested.
     *
     * @param env new environment
     */
    public static void setEnvironment(PathEnvironment env) {
        AsyncLoggedPathPlannerPathfinder.env = env;
    }

    private final AsyncPathfinderIO io;
    private final AsyncPathfinderIO.Inputs inputs;

    private Translation2d startPosition = null;
    private Translation2d goalPosition = null;

    private boolean reportedThisResult = false;

    public AsyncLoggedPathPlannerPathfinder() {
        if (RobotBase.isReal() || RobotContainer.SIM_MODE != Logging.SimMode.REPLAY) {
            io = new AsyncThreadedPathfinderIO();
        } else {
            // Don't bother starting solver thread, replay will overwrite
            // results. Instead use an IO that does nothing
            io = new AsyncPathfinderIO() {
                @Override
                public void requestPath(PathEnvironment env, Translation2d start, Translation2d goal) {}

                @Override
                public void updateInputs(Inputs inputs) {}
            };
        }
        inputs = new AsyncPathfinderIO.Inputs();
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        this.startPosition = startPosition;
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        if (env == null)
            throw new IllegalStateException("No environment set");
        if (startPosition == null)
            throw new IllegalStateException("No start position set");

        this.goalPosition = goalPosition;
        io.requestPath(env, startPosition, goalPosition);
        reportedThisResult = false;
    }

    @Override
    public boolean isNewPathAvailable() {
        // Only report path calculation result once. PathPlanner does not
        // follow the path correctly if it is continuously updated as the robot
        // travels
        if (reportedThisResult)
            return false;

        io.updateInputs(inputs);
        Logger.processInputs("Pathfinder", inputs);

        return inputs.pathReady;
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        reportedThisResult = true;

        List<Translation2d> bezierPoints = inputs.path;

        Translation2d[] logPath;
        if (bezierPoints == null) {
            // PathPlanner won't accept impossible result, so give it a straight line as a fallback
            // This can only happen if the target is inside an obstacle, which is pretty
            // easy to avoid doing
            DriverStation.reportWarning("Pathfinding: No path found", false);
            bezierPoints = List.of(startPosition, startPosition, goalPosition, goalPosition);

            logPath = new Translation2d[0];
        } else {
            logPath = new Translation2d[bezierPoints.size()];
            bezierPoints.toArray(logPath);
        }

        Logger.recordOutput("Pathfinding/Start Position", startPosition);
        Logger.recordOutput("Pathfinding/Goal Position", goalPosition);
        Logger.recordOutput("Pathfinding/Path", logPath);

        List<Waypoint> waypoints = PathPlannerPathfinder.bezierPointsToWaypoints(bezierPoints);
        return new PathPlannerPath(waypoints, constraints, null, goalEndState);
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        throw new UnsupportedOperationException("Can't set dynamic obstacles, set environment instead");
    }
}
