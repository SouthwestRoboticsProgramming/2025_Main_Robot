package com.swrobotics.robot.subsystems.outtake;

import java.util.function.Supplier;

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
    private final IndexerIO indexerIO;
    private final CoralOuttakeIO.Inputs outtakeInputs;
    private final IndexerIO.Inputs indexerInputs;

    private State targetState;

    public CoralHandlingSubsystem() {
        if (RobotBase.isReal()) {
            outtakeIO = new CoralOuttakeIOReal();
            indexerIO = new IndexerIOReal();
        } else {
            outtakeIO = new CoralOuttakeIOSim();
            indexerIO = new IndexerIOSim();
        }

        outtakeInputs = new CoralOuttakeIO.Inputs();
        indexerInputs = new IndexerIO.Inputs();

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
        outtakeIO.updateInputs(outtakeInputs);
        indexerIO.updateInputs(indexerInputs);
        Logger.processInputs("Outtake", outtakeInputs);
        Logger.processInputs("Indexer", indexerInputs);

        outtakeIO.setVoltage(targetState.getOuttakeVoltage());
        indexerIO.setVoltage(targetState.getIndexerVoltage());
    }
}
