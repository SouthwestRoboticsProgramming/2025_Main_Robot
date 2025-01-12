package com.swrobotics.robot.subsystems.pathfinding;

import com.swrobotics.lib.pathfinding.Circle;
import com.swrobotics.lib.pathfinding.Obstacle;
import com.swrobotics.lib.pathfinding.PathEnvironment;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

public final class PathEnvironments {
    public static final PathEnvironment kField;
    public static final PathEnvironment kFieldWithAutoGamePieces;

    static {
        List<Obstacle> fieldObstacles = Obstacle.loadObstaclesFromJson(Constants.kPathfindingJson);

        final double blueX = Units.inchesToMeters(48);
        final double redX = 17.548 - blueX;
        final double y1 = 4.026 - Units.inchesToMeters(72);
        final double y2 = 4.026;
        final double y3 = 4.026 + Units.inchesToMeters(72);
        final double rad = Units.inchesToMeters(16) / 2;
        List<Obstacle> withAutoPieces = new ArrayList<>(fieldObstacles);
        withAutoPieces.add(new Circle(new Translation2d(blueX, y1), rad));
        withAutoPieces.add(new Circle(new Translation2d(blueX, y2), rad));
        withAutoPieces.add(new Circle(new Translation2d(blueX, y3), rad));
        withAutoPieces.add(new Circle(new Translation2d(redX, y1), rad));
        withAutoPieces.add(new Circle(new Translation2d(redX, y2), rad));
        withAutoPieces.add(new Circle(new Translation2d(redX, y3), rad));

        double avoidanceRadius = Constants.kRobotRadius + Constants.kPathfindingTolerance;
        kField = new PathEnvironment(fieldObstacles, avoidanceRadius);
        kFieldWithAutoGamePieces = new PathEnvironment(withAutoPieces, avoidanceRadius);
    }
}
