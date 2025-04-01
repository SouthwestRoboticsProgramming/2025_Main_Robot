package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AimArmCommand extends Command {
    private final double STANDARD_DISTANCE = 3.20- 3.186; // Meters

    private final RobotContainer robot;

    public AimArmCommand(RobotContainer robot) {
        this.robot = robot;
    }

    @Override
    public void execute() {
        // Get distance to reef
        Pose2d position = FieldPositions.getClosestSnapTarget(robot.drive.getEstimatedPose());
        double distance = robot.drive.getEstimatedPose().getTranslation().getDistance(position.getTranslation());

        robot.superstructure.setElevatorAim(getElevatorAdjust(distance- STANDARD_DISTANCE));
        robot.superstructure.setPivotAim(getArmAngle(distance - STANDARD_DISTANCE));
    }

    @Override
    public void end(boolean interrupted) {
        robot.superstructure.setElevatorAim(0.0);
        robot.superstructure.setPivotAim(0.0);
    }

    private double getArmAngle(double distance) {
        if (distanceNotStandard(distance))  { return 0; }
        return Constants.kAimArmCoefficient.get() * distance;
    }

    private double getElevatorAdjust(double distance) {
        if (distanceNotStandard(distance))  { return 0; }
        return Constants.kAimElevatorCoefficient.get() * distance;
    }

    private boolean distanceNotStandard(double distance) {
        return distance > 0 && distance < 0.2286; // Two coral thick
    }
}
