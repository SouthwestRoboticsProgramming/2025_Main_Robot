package com.swrobotics.robot.subsystems.outtake.algae;

import org.littletonrobotics.junction.Logger;

import com.swrobotics.robot.config.Constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeOuttakeSubsystem extends SubsystemBase {
    public enum State {
        IDLE,
        INTAKE,
        SCORE
    }

    private final AlgaeOuttakeIO outtakeIO;
    private final AlgaeOuttakeIO.Inputs outtakeInputs;

    private State targetState;

    public AlgaeOuttakeSubsystem() {
        if (RobotBase.isReal()) {
            outtakeIO = new AlgaeOuttakeIOReal();
        } else {
            outtakeIO = new AlgaeOuttakeIOSim();
        }

        outtakeInputs = new AlgaeOuttakeIO.Inputs();

        targetState = State.IDLE;
    }

    public void setTargetState(State targetCoralState) {
        this.targetState = targetCoralState;
    }


    public Command commandSetState(State targetCoralState) {
        return Commands.run(() -> setTargetState(targetCoralState), this);
    }

    public Command commandSetStateOnce(State targetCoralState) {
        return Commands.runOnce(() -> setTargetState(targetCoralState), this);
    }

    @Override
    public void periodic() {

        if (targetState == State.INTAKE && outtakeInputs.hasPiece) {
            targetState = State.IDLE;
        }

        Logger.processInputs("Algae Outtake", outtakeInputs);

        switch (targetState) {
            case IDLE -> outtakeIO.setVoltage(outtakeInputs.hasPiece ? Constants.kOuttakeRollerHoldAlgaeVoltage.get() : 0.0);
            case INTAKE -> outtakeIO.setVoltage(Constants.kOuttakeRollerIntakeAlgaeVoltage.get());
            case SCORE -> outtakeIO.setVoltage(Constants.kOuttakeRollerScoreAlgaeVoltage.get());
        }
    }

    public boolean hasPiece() {
        return outtakeInputs.hasPiece;
    }
}
