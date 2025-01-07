package com.swrobotics.lib.pathfinding;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * An environment to find paths within. This contains the set of obstacles for
 * the pathfinder to avoid.
 */
public final class PathEnvironment {
    public static final PathEnvironment EMPTY = new PathEnvironment(Collections.emptyList(), 0);

    private final List<Obstacle> obstacles;
    private final long handle;

    /**
     * @param obstacles obstacles within the environment
     * @param avoidanceRadius Minimum distance from the <i>center</i> of the
     *                        robot to the obstacles. This should be at least
     *                        the maximum radius of the robot.
     */
    public PathEnvironment(List<Obstacle> obstacles, double avoidanceRadius) {
        this.obstacles = obstacles;
        long obs = PathfindingJNI.newObstacleList();
        for (Obstacle obstacle : obstacles) {
            obstacle.addToJNIObstacleList(obs);
        }
        handle = PathfindingJNI.buildEnvironment(obs, avoidanceRadius);
    }

    /**
     * Searches for a path within the environment. While this is pretty fast
     * (about 6 ms on the RoboRIO), it still uses a significant portion of the
     * 20 ms cycle time, so this should be called infrequently.
     *
     * @param start position to start the search from (robot position)
     * @param goal position to find a path to
     * @return Bezier points of the path from start to goal, or null if no path
     *         could be found
     */
    public List<Translation2d> findPath(Translation2d start, Translation2d goal) {
        double[] result = PathfindingJNI.findPath(handle, start.getX(), start.getY(), goal.getX(), goal.getY());
        if (result == null)
            return null; // Didn't find a path

        List<Translation2d> bezierPoints = new ArrayList<>();
        for (int i = 0; i < result.length; i += 2) {
            double x = result[i];
            double y = result[i + 1];
            bezierPoints.add(new Translation2d(x, y));
        }
        return bezierPoints;
    }

    /**
     * Gets the debug data for the environment.
     * @return debug data
     */
    public PathfindingDebug getDebug() {
        return new PathfindingDebug(obstacles, PathfindingJNI.getDebugData(handle));
    }

    public List<Translation2d> debugFindSafe(Translation2d point) {
        double[] result = PathfindingJNI.debugFindSafe(handle, point.getX(), point.getY());
        
        List<Translation2d> points = new ArrayList<>();
        for (int i = 0; i < result.length; i += 2) {
            double x = result[i];
            double y = result[i + 1];
            points.add(new Translation2d(x, y));
        }
        return points;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        PathEnvironment that = (PathEnvironment) o;
        return handle == that.handle;
    }

    @Override
    public int hashCode() {
        return Objects.hash(handle);
    }
}
