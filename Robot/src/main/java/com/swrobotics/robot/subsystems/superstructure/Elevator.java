package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

// This is not its own subsystem! It is part of SuperstructureSubsystem.
public final class Elevator {
    public enum Position {
        DOWN(() -> 0.0),
        WAIT_FOR_PIVOT(Constants.kElevatorMaxHeightWithArmIn),
        SCORE_L1(Constants.kElevatorHeightL1),
        SCORE_L2(Constants.kElevatorHeightL2),
        SCORE_L3(Constants.kElevatorHeightL3),
        SCORE_L4(Constants.kElevatorHeightL4);

        private final Supplier<Double> heightGetter;

        Position(Supplier<Double> heightGetter) {
            this.heightGetter = heightGetter;
        }

        public double getHeight() {
            return heightGetter.get();
        }
    }

    private final ElevatorIO io;
    private final ElevatorIO.Inputs inputs;

    private Position targetPosition;

    public Elevator() {
        if (RobotBase.isReal())
            io = new ElevatorIOReal();
        else
            io = new ElevatorIOSim();
        inputs = new ElevatorIO.Inputs();

        targetPosition = Position.DOWN;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setTargetPosition(Position targetPosition) {
        this.targetPosition = targetPosition;

        double targetHeight = targetPosition.getHeight();
        if (targetHeight == 0.0 && isInTolerance()) {
            // Conserve battery power when we can
            io.setNeutral();
        } else {
            io.setTargetHeight(targetHeight);
        }
    }

    public double getCurrentHeight() {
        return inputs.currentHeight;
    }

    public boolean isSafeToMovePivotIn() {
        return inputs.currentHeight < Constants.kElevatorMaxHeightWithArmIn.get();
    }

    public boolean isInTolerance() {
        return Math.abs(inputs.currentHeight - targetPosition.getHeight()) < Constants.kElevatorTolerance.get();
    }
}
