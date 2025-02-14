package com.swrobotics.robot.subsystems.intake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.RobotView;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class IntakeSubsystem extends SubsystemBase {
    private static final NTBoolean CALIBRATE_PIVOT = new NTBoolean("Intake/Pivot/Calibrate", false);

    public enum State {
        STOW(Constants.kIntakeStowAngle, () -> 0.0),
        INTAKE_CORAL(Constants.kIntakeCoralAngle, Constants.kIntakeCoralVoltage),
        EJECT_CORAL(Constants.kIntakeStowAngle, () -> -Constants.kIntakeCoralEjectVoltage.get()),
        INTAKE_ALGAE(Constants.kIntakeAlgaeAngle, () -> -Constants.kIntakeAlgaeVoltage.get()),
        EJECT_ALGAE(Constants.kIntakeAlgaeAngle, Constants.kIntakeAlgaeEjectVoltage);

        private final Supplier<Double> angleGetter;
        private final Supplier<Double> voltageGetter;

        State(Supplier<Double> angleGetter, Supplier<Double> voltageGetter) {
            this.angleGetter = angleGetter;
            this.voltageGetter = voltageGetter;
        }

        public double getAngle() {
            return Units.degreesToRotations(angleGetter.get());
        }

        public double getVoltage() {
            return voltageGetter.get();
        }
    }

    private final IntakeIO io;
    private final IntakeIO.Inputs inputs;

    private State targetState;

    public IntakeSubsystem() {
        if (RobotBase.isReal()) {
            io = new IntakeIOReal();
        } else {
            io = new IntakeIOSim();
        }
        inputs = new IntakeIO.Inputs();

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
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (CALIBRATE_PIVOT.get()) {
            CALIBRATE_PIVOT.set(false);
            io.calibrateEncoder();
        }

        RobotView.setIntakeState(inputs.currentAngleRot, inputs.voltageOut);

        io.setTargetAngle(targetState.getAngle());
        io.setVoltage(targetState.getVoltage());
    }
}
