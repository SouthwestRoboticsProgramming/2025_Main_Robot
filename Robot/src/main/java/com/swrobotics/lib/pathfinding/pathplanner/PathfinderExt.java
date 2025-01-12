package com.swrobotics.lib.pathfinding.pathplanner;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.swrobotics.lib.pathfinding.PathEnvironment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

public interface PathfinderExt {
    boolean isNewPathAvailable();

    void setEnvironment(PathEnvironment env);

    void setStartPosition(Translation2d startPos);

    void setGoalPoses(List<Pose2d> goalPos);

    PathPlannerPath getCurrentPath(PathConstraints pathConstraints, double goalVelocityMPS);
}
