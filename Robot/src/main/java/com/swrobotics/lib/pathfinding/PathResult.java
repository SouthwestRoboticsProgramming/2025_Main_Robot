package com.swrobotics.lib.pathfinding;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

public record PathResult(int goalIndex, List<Translation2d> bezierPoints) {
}
