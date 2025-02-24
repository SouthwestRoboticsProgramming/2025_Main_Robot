package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ProfileCalcTest {
    public static void doTest() {
        TrapezoidProfile pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Constants.kOuttakePivotMotionMagic.getCruiseVelocity(),
                Constants.kOuttakePivotMotionMagic.getAcceleration()
        ));
        TrapezoidProfile elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Constants.kElevatorMotionMagic.getCruiseVelocity(),
                Constants.kElevatorMotionMagic.getAcceleration()
        ));

        pivotProfile.calculate(
                0.02,
                new TrapezoidProfile.State(Units.degreesToRotations(85), 0),
                new TrapezoidProfile.State( Units.degreesToRotations(60), 0)
        );
        double pivotWaitTime = pivotProfile.timeLeftUntil(Units.degreesToRotations((68.2 + 85) / 2));
        System.out.println("Pivot wait time: " + pivotWaitTime);

        double elevatorCollisionHeight = 3.7993;
        double elevatorWaitAccel = 2 * elevatorCollisionHeight / MathUtil.square(pivotWaitTime);
        System.out.println("Elevator wait accel: " + elevatorWaitAccel);
        if (elevatorWaitAccel > Constants.kElevatorMotionMagic.getAcceleration()) {
            System.out.println("Higher than max, clamping to " + Constants.kElevatorMotionMagic.getAcceleration());
            elevatorWaitAccel = Constants.kElevatorMotionMagic.getAcceleration();
        }
        double elevatorVelocityAfterWait = elevatorWaitAccel * pivotWaitTime;
        System.out.println("Elevator velocity after wait: " + elevatorVelocityAfterWait);

        elevatorProfile.calculate(
                0.02,
                new TrapezoidProfile.State(elevatorCollisionHeight, elevatorVelocityAfterWait),
                new TrapezoidProfile.State(Constants.kElevatorMaxHeightRotations / 20.0 * 16.0, 0.0)
        );
        double elevatorTime = elevatorProfile.totalTime();
        System.out.println("Elevator travel time: " + elevatorTime);

        double totalElevatorTime = pivotWaitTime + elevatorTime;
        System.out.println("Full elevator motion time: " + totalElevatorTime);

        double fullPivotTime = pivotProfile.totalTime();
        System.out.println("Full pivot motion time: " + fullPivotTime);

        System.out.println("Full actuation time: " + Math.max(totalElevatorTime, fullPivotTime));
    }
}
