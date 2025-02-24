package com.swrobotics.robot.logging;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public final class RobotView {
    private static final LoggedMechanism2d mechanism = new LoggedMechanism2d(2, 3);

    private static final LoggedMechanismRoot2d center = mechanism.getRoot("Center", 1, 0.5);
    private static final LoggedMechanismLigament2d elevator = center.append(
            new LoggedMechanismLigament2d("Elevator", 2, 90, 4, new Color8Bit(Color.kOrange)));

    private static final LoggedMechanismRoot2d outtakePivot = mechanism.getRoot("Outtake Pivot", 1, 0.5);
    private static final LoggedMechanismLigament2d outtake = outtakePivot.append(
            new LoggedMechanismLigament2d("Outtake", 0.7, 0, 3, new Color8Bit(Color.kYellow)));
    
    private static final LoggedMechanismRoot2d algaePivot = mechanism.getRoot("Algae Pivot", 0.5, 0.5);
    private static final LoggedMechanismLigament2d algaeIntake = algaePivot.append(
            new LoggedMechanismLigament2d("Algae Intake", 0.5, 90, 3, new Color8Bit(Color.kWhite)));

    private static final LoggedMechanismRoot2d outtakeTargetPivot = mechanism.getRoot("Outtake Target Pivot", 1, 0.5);
    private static final LoggedMechanismLigament2d outtakeTarget = outtakeTargetPivot.append(
            new LoggedMechanismLigament2d("Outtake Target", 0.7, 0, 3, new Color8Bit(Color.kLightBlue)));

    private static final LoggedMechanismRoot2d outtakeSetpointPivot = mechanism.getRoot("Outtake Setpoint Pivot", 1, 0.5);
    private static final LoggedMechanismLigament2d outtakeSetpoint = outtakeTargetPivot.append(
            new LoggedMechanismLigament2d("Outtake Setpoint", 0.7, 0, 3, new Color8Bit(Color.kBlue)));

    public static void setSuperstructureState(double elevatorHeightPct, double pivotAngleRot) {
        outtakePivot.setPosition(1, 2 * elevatorHeightPct);
        outtake.setAngle(Units.rotationsToDegrees(pivotAngleRot));
    }

    public static void setTargetSuperstructureState(double elevatorHeightPct, double pivotAngleRot) {
        outtakeTargetPivot.setPosition(1, 2 * elevatorHeightPct);
        outtakeTarget.setAngle(Units.rotationsToDegrees(pivotAngleRot));
    }

    public static void setSuperstructureSetpoint(double elevatorHeightPct, double pivotAngleRot) {
        outtakeSetpointPivot.setPosition(1, 2 * elevatorHeightPct);
        outtakeSetpoint.setAngle(Units.rotationsToDegrees(pivotAngleRot));
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
