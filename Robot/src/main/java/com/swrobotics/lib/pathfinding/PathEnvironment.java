package com.swrobotics.lib.pathfinding;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.*;

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
    public PathResult findPath(Translation2d start, Translation2d goal) {
        return findPathToClosest(start, Collections.singleton(goal));
    }

    public PathResult findPathToClosest(Translation2d start, Collection<Translation2d> goals) {
        if (goals.isEmpty())
            return null;

        double[] goalsData = new double[goals.size() * 2];
        int goalIdx = 0;
        for (Translation2d goal : goals) {
            goalsData[goalIdx++] = goal.getX();
            goalsData[goalIdx++] = goal.getY();
        }

        double[] result = PathfindingJNI.findPath(handle, start.getX(), start.getY(), goalsData);
        if (result == null)
            return null; // Didn't find a path

        int chosenGoalIdx = (int) result[0];
        List<Translation2d> bezierPoints = new ArrayList<>();
        for (int i = 1; i < result.length; i += 2) {
            double x = result[i];
            double y = result[i + 1];
            bezierPoints.add(new Translation2d(x, y));
        }

        return new PathResult(chosenGoalIdx, bezierPoints);
    }

    /**
     * Gets the debug data for the environment.
     * @return debug data
     */
    public PathfindingDebug getDebug() {
        return new PathfindingDebug(obstacles, PathfindingJNI.getDebugData(handle));
    }

    public double[] debugFindSafe(Translation2d point) {
        return PathfindingJNI.debugFindSafe(handle, point.getX(), point.getY());
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
