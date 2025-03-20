package com.swrobotics.robot.subsystems.algae;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.RobotView;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private static final NTBoolean CALIBRATE_PIVOT = new NTBoolean("Algae/Pivot/Calibrate", false);

    public enum State {
        STOW(() -> 0.0),
        INTAKE(Constants.kAlgaeIntakeVoltage),
        OUTTAKE(() -> -Constants.kAlgaeOuttakeVoltage.get());

        private final Supplier<Double> voltageGetter;

        State(Supplier<Double> voltageGetter) {
            this.voltageGetter = voltageGetter;
        }

        public double getVoltage() {
            return voltageGetter.get();
        }
    }

    private final AlgaeIO algaeIO;
    private final AlgaeIO.Inputs algaeInputs;
    private final Debouncer algaeDetectDebounce;

    private State targetState;

    public AlgaeIntakeSubsystem() {
        if (RobotBase.isReal()) {
            algaeIO = new AlgaeIOReal();
        } else {
            algaeIO = new AlgaeIOSim();
        }
        algaeInputs = new AlgaeIO.Inputs();

        algaeDetectDebounce = new Debouncer(Constants.kAlgaeDetectDebounce.get(), Debouncer.DebounceType.kBoth);
        Constants.kAlgaeDetectDebounce.onChange(
                () -> algaeDetectDebounce.setDebounceTime(Constants.kAlgaeDetectDebounce.get()));

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

        if (CALIBRATE_PIVOT.get()) {
            CALIBRATE_PIVOT.set(false);
            algaeIO.calibrateEncoder();
        }

        RobotView.setAlgaeIntakeState(algaeInputs.currentAngleRot, algaeInputs.voltageOut);

        boolean stalled = algaeInputs.statorCurrent > Constants.kAlgaeDetectCurrentThreshold.get();
        boolean hasAlgae = algaeDetectDebounce.calculate(stalled);
        Logger.recordOutput("Algae/Stalled", stalled);
        Logger.recordOutput("Algae/Has Algae", hasAlgae);

        double angleDeg;
        if (targetState == State.STOW) {
            angleDeg = Constants.kAlgaeStowAngle.get();
        } else {
            if (hasAlgae) {
                angleDeg = Constants.kAlgaeIntakeHoldAngle.get();
            } else {
                angleDeg = Constants.kAlgaeIntakeAngle.get();
            }
        }

        algaeIO.setTargetAngle(Units.degreesToRotations(angleDeg));
        algaeIO.setVoltage(targetState.getVoltage());
    }
}
