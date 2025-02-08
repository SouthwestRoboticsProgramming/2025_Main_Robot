package com.swrobotics.robot.subsystems.outtake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import com.swrobotics.robot.config.Constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralHandlingSubsystem extends SubsystemBase {
    public enum State {
        INTAKE(Constants.kOuttakeRollerIntakeVoltage, Constants.kIndexerIntakeVoltage),
        SCORE(Constants.kOuttakeRollerScoreVoltage, () -> 0.0),
        HOLD(() -> 0.0, () -> 0.0);

        private final Supplier<Double> outtakeVoltage;
        private final Supplier<Double> indexerVoltage;

        State(Supplier<Double> outtakeVoltage, Supplier<Double> indexerVoltage) {
            this.outtakeVoltage = outtakeVoltage;
            this.indexerVoltage = indexerVoltage;
        }

        public double getOuttakeVoltage() {
            return outtakeVoltage.get();
        }

        public double getIndexerVoltage() {
            return indexerVoltage.get();
        }
    }

    private final CoralOuttakeIO outtakeIO;
    private final CoralOuttakeIO.Inputs outtakeInputs;

    private State targetState;
    private double beamBreakDetectTime;

    public CoralHandlingSubsystem() {
        if (RobotBase.isReal()) {
            outtakeIO = new CoralOuttakeIOReal();
        } else {
            outtakeIO = new CoralOuttakeIOSim();
        }

        outtakeInputs = new CoralOuttakeIO.Inputs();

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
        boolean hadPiece = outtakeInputs.hasPiece;
        outtakeIO.updateInputs(outtakeInputs);
        Logger.processInputs("Outtake", outtakeInputs);
        boolean hasPiece = outtakeInputs.hasPiece;

        if (!hadPiece && hasPiece) {
            beamBreakDetectTime = Timer.getTimestamp();
        }

        if (outtakeInputs.hasPiece && targetState != State.SCORE && Timer.getTimestamp() >= beamBreakDetectTime + 0.05) {
            setTargetState(State.HOLD);
        }

        outtakeIO.setVoltage(targetState.getOuttakeVoltage());
    }
}
