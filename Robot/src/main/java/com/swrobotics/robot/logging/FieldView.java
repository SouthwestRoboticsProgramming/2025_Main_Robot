package com.swrobotics.robot.logging;

import com.swrobotics.lib.utils.DebugGraphics;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * View of the field and various objects on it for SmartDashboard. Used in both
 * simulation and the real robot.
 */
public final class FieldView {
    private static final Field2d field = new Field2d();
    private static final Field2d left = new Field2d();
    private static final Field2d right = new Field2d();

    public static final FieldObject2d robotPose = field.getRobotObject();
    public static final FieldObject2d visionEstimates = field.getObject("Vision estimates");
    public static final FieldObject2d aprilTagPoses = field.getObject("AprilTag poses");
    public static final FieldObject2d pathPlannerPath = field.getObject("PathPlanner path");
    public static final FieldObject2d pathPlannerSetpoint = field.getObject("PathPlanner setpoint");

    public static final FieldObject2d pathfindingGoal = field.getObject("Pathfinding goal");
    public static final FieldObject2d pathfindingGoal2 = field.getObject("Pathfinding goal 2");
    public static final FieldObject2d pathfindingPath = field.getObject("Pathfinding path");

    public static final DebugGraphics pathfindingDebug = new DebugGraphics(
            "Pathfinding Debug",
            Constants.kField.getWidth(), Constants.kField.getHeight()
    );

    /**
     * Adds the field view to SmartDashboard
     */
    public static void publish() {
        SmartDashboard.putData("Field View", field);
        SmartDashboard.putData("Left", left);
        SmartDashboard.putData("Right", right);
    }
}
