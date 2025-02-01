package com.swrobotics.robot.logging;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class RobotView {
    private static final Mechanism2d mechanism = new Mechanism2d(2, 3);

    private static final MechanismRoot2d center = mechanism.getRoot("Center", 1, 0.5);
    private static final MechanismLigament2d elevator = center.append(
            new MechanismLigament2d("Elevator", 2, 90, 4, new Color8Bit(Color.kOrange)));

    private static final MechanismRoot2d outtakePivot = mechanism.getRoot("Outtake Pivot", 1, 0.5);
    private static final MechanismLigament2d outtake = outtakePivot.append(
            new MechanismLigament2d("Outtake", 0.7, 0, 3, new Color8Bit(Color.kYellow)));
    
    private static final MechanismRoot2d algaePivot = mechanism.getRoot("Algae Pivot", 0.5, 0.5);
    private static final MechanismLigament2d algaeIntake = algaePivot.append(
            new MechanismLigament2d("Algae Intake", 0.5, 90, 3, new Color8Bit(Color.kWhite)));

    public static void setSuperstructureState(double elevatorHeightMeters, double pivotAngleRot) {
        outtakePivot.setPosition(1, 0.5 + elevatorHeightMeters);
        outtake.setAngle(Units.rotationsToDegrees(pivotAngleRot));
    }

    public static void setAlgaeIntakeState(double intakeAngleRot, double voltageOut) {
        double intakeAngleDeg = Units.rotationsToDegrees(intakeAngleRot);
        algaeIntake.setAngle(180 - intakeAngleDeg);
        double red = 256.0 / 12.0 * voltageOut;
        algaeIntake.setColor(new Color8Bit((int) red, 20, 100));
    }

    public static void publish() {
        SmartDashboard.putData("Robot View", mechanism);
    }
}
