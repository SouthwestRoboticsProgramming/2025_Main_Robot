package com.swrobotics.robot.subsystems.outtake;

import org.littletonrobotics.junction.Logger;

import com.swrobotics.robot.config.Constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
    public enum GamePiece {
        CORAL,
        ALGAE
    }

    public enum State {
        INTAKE_CORAL,
        INTAKE_ALGAE,
        SCORE,
        HOLD,
        REVERSE
    }

    private final OuttakeIO outtakeIO;
    private final OuttakeIO.Inputs outtakeInputs;

    private GamePiece heldPiece;
    private State targetState;
    private double holdPosition;

    public OuttakeSubsystem() {
        if (RobotBase.isReal()) {
            outtakeIO = new OuttakeIOReal();
        } else {
            outtakeIO = new OuttakeIOSim();
        }

        outtakeInputs = new OuttakeIO.Inputs();

        heldPiece = GamePiece.CORAL;
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

    public Command score(double seconds) {
        return commandSetState(State.SCORE)
                .withTimeout(seconds)
                .finallyDo(() -> setTargetState(State.HOLD));
    }

    @Override
    public void periodic() {
        outtakeIO.setBeamBreakIgnored(targetState == State.SCORE || targetState == State.REVERSE);

        outtakeIO.updateInputs(outtakeInputs);
        Logger.processInputs("Outtake", outtakeInputs);

        if (targetState == State.INTAKE_CORAL)
            heldPiece = GamePiece.CORAL;
        else if (targetState == State.INTAKE_ALGAE)
            heldPiece = GamePiece.ALGAE;

        if (targetState == State.INTAKE_CORAL && outtakeInputs.hasPiece) {
            targetState = State.HOLD;
        }

        switch (targetState) {
            case INTAKE_CORAL -> outtakeIO.setVoltage(Constants.kOuttakeRollerIntakeCoralVoltage.get());
            case INTAKE_ALGAE -> outtakeIO.setVoltage(Constants.kOuttakeRollerIntakeAlgaeVoltage.get());
            case SCORE -> outtakeIO.setVoltage(switch (heldPiece) {
                case CORAL -> Constants.kOuttakeRollerScoreCoralVoltage.get();
                case ALGAE -> Constants.kOuttakeRollerScoreAlgaeVoltage.get();
            });
            case HOLD -> {
                switch (heldPiece) {
                    case CORAL -> outtakeIO.setHoldPosition(outtakeInputs.positionAtPieceDetect + Constants.kOuttakeHoldPositionOffset.get());
                    case ALGAE -> outtakeIO.setVoltage(-Constants.kOuttakeRollerHoldAlgaeVoltage.get());
                }
            }
            case REVERSE -> outtakeIO.setVoltage(-Constants.kOuttakeRollerIntakeCoralVoltage.get());
        }
    }

    public boolean hasPiece() {
        return outtakeInputs.hasPiece;
    }
}
