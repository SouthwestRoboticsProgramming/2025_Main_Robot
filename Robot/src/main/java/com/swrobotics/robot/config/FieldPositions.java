package com.swrobotics.robot.config;

import com.swrobotics.lib.field.FieldInfo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public final class FieldPositions {
    public enum CoralStation {
        LEFT,
        RIGHT
    }

    public static final double kStartingLineX = 1.775 + Units.inchesToMeters(231.66);

    private static final double kReefCenterX = 4.4895;
    private static final double kReefCenterY = Constants.kField.getHeight() / 2;
    private static final double kReefApothem = 4.4895 - 3.658;
    private static final double kReefBranchSpacing = Units.inchesToMeters(12.94);

    private static final double kCoralStationAngle = 35.987; // Degrees
    private static final Rotation2d kBlueRightCoralStationFacing = Rotation2d.fromDegrees(90 - kCoralStationAngle);
    private static final Rotation2d kBlueLeftCoralStationFacing = kBlueRightCoralStationFacing.unaryMinus();
    private static final Rotation2d kRedLeftCoralStationFacing = Rotation2d.fromDegrees(90 + kCoralStationAngle);
    private static final Rotation2d kRedRightCoralStationFacing = kRedLeftCoralStationFacing.unaryMinus();

    private static Pose2d kLeftCoralStationPickup;
    private static Pose2d kRightCoralStationPickup;

    private static final List<Pose2d> kReefPositions = new ArrayList<>(); // Aligned with branch
    private static final List<Pose2d> kReefAlgaePositions = new ArrayList<>(); // Centered on face
    private static void initPositions() {
        double offset = Constants.kSnapOffset.get();

        Translation2d leftCoralStationCenter = new Translation2d(1.775 / 2, 8.052 - 1.289 / 2);
        Translation2d leftCoralStationPos = leftCoralStationCenter.plus(new Translation2d(0, -Constants.kRobotLength / 2 + offset)
                .rotateBy(Rotation2d.fromDegrees(kCoralStationAngle)));
        kLeftCoralStationPickup = new Pose2d(leftCoralStationPos, kBlueLeftCoralStationFacing);

        Translation2d rightCoralStationCenter = new Translation2d(1.775 / 2, 1.289 / 2);
        Translation2d rightCoralStationPos = rightCoralStationCenter.plus(new Translation2d(0, Constants.kRobotLength / 2 + offset)
                .rotateBy(Rotation2d.fromDegrees(-kCoralStationAngle)));
        kRightCoralStationPickup = new Pose2d(rightCoralStationPos, kBlueRightCoralStationFacing);

        Translation2d center = new Translation2d(kReefCenterX, kReefCenterY);
        kReefPositions.clear();
        kReefAlgaePositions.clear();

        double offsetX = -kReefApothem - Constants.kRobotLength / 2 - offset;
        Translation2d branchOffset1 = new Translation2d(offsetX, kReefBranchSpacing / 2);
        Translation2d branchOffset2 = new Translation2d(offsetX, -kReefBranchSpacing / 2);
        Translation2d algaeOffset = new Translation2d(offsetX, 0);

        for (int i = 0; i < 6; i++) {
            Rotation2d rotation = Rotation2d.fromDegrees(i * 60);

            kReefPositions.add(new Pose2d(center.plus(branchOffset1.rotateBy(rotation)), rotation));
            kReefPositions.add(new Pose2d(center.plus(branchOffset2.rotateBy(rotation)), rotation));
            kReefAlgaePositions.add(new Pose2d(center.plus(algaeOffset.rotateBy(rotation)), rotation));
        }

        int i = 0;
        for (Pose2d reefPos : kReefPositions) {
            System.out.println("Reef position " + (i++) + ": " + reefPos.getTranslation());
        }
    }

    static {
        initPositions();
        Constants.kSnapOffset.onChange(FieldPositions::initPositions);
    }

    // Branches are numbered in the same order as Figure 5-8 in the game manual
    public static Pose2d getBlueReefScoringTarget(int branch) {
        return kReefPositions.get(branch);
    }

    public static Pose2d getAllianceReefScoringTarget(int branch) {
        return Constants.kField.flipPoseForAlliance(getBlueReefScoringTarget(branch));
    }

    // Faces are numbered counterclockwise starting with the one facing the
    // driver station
    public static Pose2d getBlueReefDealgifyingTarget(int face) {
        return kReefAlgaePositions.get(face);
    }

    public static Rotation2d getBlueCoralStationAngle(CoralStation station) {
        return station == CoralStation.LEFT
                ? kBlueLeftCoralStationFacing
                : kBlueRightCoralStationFacing;
    }

    public static Rotation2d getAllianceCoralStationAngle(CoralStation station) {
        if (FieldInfo.getAlliance() == DriverStation.Alliance.Blue) {
            return station == CoralStation.LEFT
                    ? kBlueLeftCoralStationFacing
                    : kBlueRightCoralStationFacing;
        } else {
            return station == CoralStation.LEFT
                    ? kRedLeftCoralStationFacing
                    : kRedRightCoralStationFacing;
        }
    }

    public static Rotation2d getClosestCoralStationAngle(Pose2d robotPose) {
        // Select based on which quadrant of the field we're in
        // This allows snapping to any coral station, even opponent ones
        if (robotPose.getX() < Constants.kField.getWidth() / 2) {
            return robotPose.getY() > Constants.kField.getHeight() / 2
                    ? kBlueLeftCoralStationFacing
                    : kBlueRightCoralStationFacing;
        } else {
            return robotPose.getY() < Constants.kField.getHeight() / 2
                    ? kRedLeftCoralStationFacing
                    : kRedRightCoralStationFacing;
        }
    }

    public static Pose2d getClosestSnapTarget(Pose2d robotPose) {
        Pose2d chosen = null;
        double closestDistance = Double.POSITIVE_INFINITY;
        for (Pose2d target : kReefPositions) {
            Pose2d flipped = Constants.kField.flipPoseForAlliance(target);

            double distance = flipped.getTranslation().getDistance(robotPose.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                chosen = flipped;
            }
        }
        return chosen;
    }

    public static Rotation2d getAllianceProcessorAngle() {
        return FieldInfo.getAllianceForwardAngle().plus(Rotation2d.kCW_90deg);
    }

    public static double getAllianceProcessorY() {
        double away = Constants.kRobotLength / 2 + Constants.kSnapProcessorSpace.get();
        if (FieldInfo.getAlliance() == DriverStation.Alliance.Blue) {
            return away;
        } else {
            return Constants.kField.getHeight() - away;
        }
    }

    public static Pose2d getLeftCoralStationPickup() {
        return kLeftCoralStationPickup;
    }

    public static Pose2d getRightCoralStationPickup() {
        return kRightCoralStationPickup;
    }
}
