package com.swrobotics.robot.subsystems.outtake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.swrobotics.robot.config.Constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
    public enum State {
        INTAKE(Constants.kOuttakeRollerIntakeVoltage),
        SCORE(Constants.kOuttakeRollerScoreVoltage),
        HOLD(() -> 0.0);

        private final Supplier<Double> voltageGetter;

        State(Supplier<Double> voltageGetter) {
            this.voltageGetter = voltageGetter;
        }

        public double getVoltage() {
            return voltageGetter.get();
        }
    }

    private final CoralOuttakeIO io;
    private final CoralOuttakeIO.Inputs inputs;

    private State targetState;

    public OuttakeSubsystem() {
        if (RobotBase.isReal()) {
            io = new CoralOuttakeIOReal();
        } else {
            io = new CoralOuttakeIOSim();
        }
        inputs = new CoralOuttakeIO.Inputs();

        targetState = State.HOLD;
    }

    public void setTargetState(State targetState) {
        this.targetState = targetState;
    }

    public Command commandSetState(State targetState) {
        return Commands.run(() -> setTargetState(targetState), this);
    }

    public Command commandSetStateOnce(State targetState) {
        return Commands.runOnce(() -> setTargetState(targetState), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Outtake", inputs);

        if (inputs.hasCoral && targetState != State.SCORE) {
            setTargetState(State.HOLD);
        }

        io.setVoltage(targetState.getVoltage());
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }
}
