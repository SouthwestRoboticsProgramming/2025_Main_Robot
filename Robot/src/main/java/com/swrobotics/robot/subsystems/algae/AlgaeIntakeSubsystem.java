package com.swrobotics.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.RobotView;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {

    public enum State {
        STOW(Constants.kAlgaeStowAngle, () -> 0.0),
        INTAKE(Constants.kAlgaeIntakeAngle, Constants.kAlgaeIntakeVoltage),
        HOLD(Constants.kAlgaeHoldAngle, Constants.kAlgaeHoldVoltage);

        private final Supplier<Double> angleGetter;
        private final Supplier<Double> voltageGetter;

        State(Supplier<Double> angleGetter, Supplier<Double> voltageGetter) {
            this.angleGetter = angleGetter;
            this.voltageGetter = voltageGetter;
        }

        public double getAngle() {
            return angleGetter.get();
        }

        public double getVoltage() {
            return voltageGetter.get();
        }
    }

    private final AlgaeIO algaeIO;
    private final AlgaeIO.Inputs algaeInputs;

    private State targetState;

    public AlgaeIntakeSubsystem() {
        if (RobotBase.isReal()) {
            algaeIO = new AlgaeIOReal();
        } else {
            algaeIO = new AlgaeIOSim();
        }
        algaeInputs = new AlgaeIO.Inputs();

        targetState = State.STOW;
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
        algaeIO.updateInputs(algaeInputs);
        Logger.processInputs("Algae", algaeInputs);

        RobotView.setAlgaeIntakeState(algaeInputs.currentAngleRot / 360.0, algaeInputs.voltageOut);

        algaeIO.setTargetAngle(Degrees.of(targetState.getAngle()));
        algaeIO.setVoltage(targetState.getVoltage());
    }
}
