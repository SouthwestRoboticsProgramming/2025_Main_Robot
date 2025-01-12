package com.swrobotics.robot.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

public final class SnapTargets {
    private static final double kReefCenterX = 4.4895;
    private static final double kReefCenterY = Constants.kField.getHeight() / 2;
    private static final double kReefApothem = 4.4895 - 3.658;
    private static final double kReefBranchSpacing = Units.inchesToMeters(12.94);

    private static final List<Pose2d> kTargets;
    static {
        Translation2d center = new Translation2d(kReefCenterX, kReefCenterY);

        double offsetX = -kReefApothem - Constants.kFrameLengthWithBumpers / 2;
        Translation2d offset1 = new Translation2d(offsetX, kReefBranchSpacing / 2);
        Translation2d offset2 = new Translation2d(offsetX, -kReefBranchSpacing / 2);

        kTargets = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            Rotation2d rotation = Rotation2d.fromDegrees(i * 60);

            kTargets.add(new Pose2d(center.plus(offset1.rotateBy(rotation)), rotation));
            kTargets.add(new Pose2d(center.plus(offset2.rotateBy(rotation)), rotation));
        }
    }

    public static Pose2d getClosestTarget(Pose2d robotPose) {
        Pose2d chosen = null;
        double closestDistance = Double.POSITIVE_INFINITY;
        for (Pose2d target : kTargets) {
            Pose2d flipped = Constants.kField.flipPoseForAlliance(target);

            double distance = flipped.getTranslation().getDistance(robotPose.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                chosen = flipped;
            }
        }
        return chosen;
    }
}
