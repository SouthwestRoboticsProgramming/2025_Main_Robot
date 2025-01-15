package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

// This is not its own subsystem! It is part of SuperstructureSubsystem.
public final class OuttakePivot {
    private static final NTBoolean CALIBRATE = new NTBoolean("Coral Outtake Pivot/Encoder/Calibrate", false);

    public enum Position {
        IN(Constants.kOuttakePivotInAngle),
        WAIT_FOR_ELEVATOR(Constants.kOuttakePivotMaxAngleWithElevatorUp),
        SCORE_L1(Constants.kOuttakePivotScoreL1Angle),
        SCORE_L2L3(Constants.kOuttakePivotScoreL2L3Angle),
        SCORE_L4(Constants.kOuttakePivotScoreL4Angle);

        private final Supplier<Double> getter;

        Position(Supplier<Double> getter) {
            this.getter = getter;
        }

        public double getAngleDeg() {
            return getter.get();
        }
    }

    private final OuttakePivotIO io;
    private final OuttakePivotIO.Inputs inputs;

    private Position targetPosition;

    public OuttakePivot() {
        if (RobotBase.isReal())
            io = new OuttakePivotIOReal();
        else
            io = new OuttakePivotIOSim();
        inputs = new OuttakePivotIO.Inputs();

        targetPosition = Position.IN;
    }

    public boolean isSafeToMoveElevatorUp() {
        return inputs.currentAngle <= Units.degreesToRotations(Constants.kOuttakePivotMaxAngleWithElevatorUp.get());
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Outtake Pivot", inputs);

        if (CALIBRATE.get()) {
            CALIBRATE.set(false);
            io.calibrateEncoder();
        }
    }

    public void setTargetPosition(Position targetPosition) {
        this.targetPosition = targetPosition;

        double angleDeg = targetPosition.getAngleDeg();
        io.setTargetAngle(Units.degreesToRotations(angleDeg));
    }

    public double getCurrentAngle() {
        return inputs.currentAngle;
    }

    public boolean isInTolerance() {
        double targetAngle = Units.degreesToRotations(targetPosition.getAngleDeg());
        double error = Math.abs(targetAngle - inputs.currentAngle);

        return error < Constants.kOuttakePivotTolerance.get();
    }
}
