package com.swrobotics.lib.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Represents the information related to the layout of the game field. */
public final class FieldInfo {
    /** Information for the 2023 Charged Up field */
    public static final FieldInfo CHARGED_UP_2023 =
            new FieldInfo(16.4846, 8.02, FieldSymmetry.LATERAL);

    /** Information for the 2024 Crescendo field */
    public static final FieldInfo CRESCENDO_2024 =
            new FieldInfo(16.541, 8.211, FieldSymmetry.LATERAL);

    /** Information for the 2025 Reefscape field */
    public static final FieldInfo REEFSCAPE_2025 =
            new FieldInfo(17.548, 8.052, FieldSymmetry.ROTATIONAL);

    public static DriverStation.Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(null);
    }

    /**
     * Gets the forward vector relative to the current alliance driver station.
     *
     * @return alliance forward vector
     */
    public static Translation2d getAllianceForward() {
        return getAlliance() == DriverStation.Alliance.Blue
                ? new Translation2d(1, 0)
                : new Translation2d(-1, 0);
    }

    /**
     * Gets the angle pointing forward relative to the current alliance driver station.
     *
     * @return alliance forward angle
     */
    public static Rotation2d getAllianceForwardAngle() {
        return getAlliance() == DriverStation.Alliance.Blue
                ? Rotation2d.kZero
                : Rotation2d.kPi;
    }

    /**
     * Gets the reverse vector relative to the current alliance driver station.
     *
     * @return alliance reverse vector
     */
    public static Translation2d getAllianceReverse() {
        return getAllianceForward().unaryMinus();
    }

    /**
     * Gets the angle pointing reverse relative to the current alliance driver station.
     *
     * @return alliance reverse angle
     */
    public static Rotation2d getAllianceReverseAngle() {
        return getAlliance() == DriverStation.Alliance.Blue
                ? Rotation2d.kPi
                : Rotation2d.kZero;
    }

    private final Translation2d size;
    private final FieldSymmetry symmetry;

    public FieldInfo(double width, double height, FieldSymmetry symmetry) {
        size = new Translation2d(width, height);
        this.symmetry = symmetry;
    }

    /**
     * Gets the size of the field in meters.
     *
     * @return size in meters
     */
    public Translation2d getSize() {
        return size;
    }

    /**
     * Gets the width of the field in meters. This is the horizontal distance relative to the
     * scoring table.
     *
     * @return width in meters
     */
    public double getWidth() {
        return size.getX();
    }

    /**
     * Gets the height of the field in meters. This is the vertical distance relative to the scoring
     * table.
     *
     * @return height in meters
     */
    public double getHeight() {
        return size.getY();
    }

    /**
     * Gets the center position of the field in meters.
     *
     * @return center position in meters
     */
    public Translation2d getCenter() {
        return size.div(2);
    }

    /**
     * Gets the type of symmetry for the two halves of the field.
     *
     * @return alliance symmetry
     */
    public FieldSymmetry getSymmetry() {
        return symmetry;
    }

    /**
     * Flips a pose to be relative to the current alliance. This is equivalent to {@code
     * getSymmetry().flipForAlliance(bluePose, this)}.
     *
     * @param bluePose pose as if the robot was on blue alliance
     * @return pose relative to the current alliance
     */
    public Pose2d flipPoseForAlliance(Pose2d bluePose) {
        return symmetry.flipForAlliance(bluePose, this);
    }

    @Override
    public String toString() {
        return "FieldInfo{" + "size=" + size + ", symmetry=" + symmetry + '}';
    }
}
