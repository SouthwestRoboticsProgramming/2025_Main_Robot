package com.swrobotics.robot.commands;

import com.swrobotics.lib.utils.MathUtil;
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

        distance -= STANDARD_DISTANCE;

        double maxDistance = 0.15;
        distance = MathUtil.clamp(distance, 0, maxDistance);
        double distancePct = distance / maxDistance;

        robot.superstructure.setElevatorAim(getElevatorAdjust(distancePct));
        robot.superstructure.setPivotAim(getArmAngle(distancePct));
    }

    @Override
    public void end(boolean interrupted) {
        robot.superstructure.setElevatorAim(0.0);
        robot.superstructure.setPivotAim(0.0);
    }

    private double getArmAngle(double distancePct) {
        return Constants.kAimArmCoefficient.get() * distancePct;
    }

    private double getElevatorAdjust(double distancePct) {
        return Constants.kAimElevatorCoefficient.get() * distancePct;
    }
}
