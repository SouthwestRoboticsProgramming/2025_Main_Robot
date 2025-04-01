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
        SCORE_NOT_L4,
        SCORE_L4,
        HOLD,
        REVERSE
    }

    private final OuttakeIO outtakeIO;
    private final OuttakeIO.Inputs outtakeInputs;

    private GamePiece heldPiece;
    private State targetState;
    private boolean reverseScore;

    public OuttakeSubsystem() {
        if (RobotBase.isReal()) {
            outtakeIO = new OuttakeIOReal();
        } else {
            outtakeIO = new OuttakeIOSim();
        }

        outtakeInputs = new OuttakeIO.Inputs();

        heldPiece = GamePiece.CORAL;
        targetState = State.HOLD;
        reverseScore = false;
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

    public Command score(double seconds, boolean l4) {
        return commandSetState(l4 ? State.SCORE_L4 : State.SCORE_NOT_L4)
                .withTimeout(seconds)
                .finallyDo(() -> setTargetState(State.HOLD));
    }

    @Override
    public void periodic() {
        outtakeIO.setBeamBreakIgnored(targetState == State.SCORE_NOT_L4 || targetState == State.SCORE_L4 || targetState == State.REVERSE);

        outtakeIO.updateInputs(outtakeInputs);
        Logger.processInputs("Outtake", outtakeInputs);

        if (targetState == State.INTAKE_CORAL)
            heldPiece = GamePiece.CORAL;
        else if (targetState == State.INTAKE_ALGAE)
            heldPiece = GamePiece.ALGAE;

        if (targetState == State.INTAKE_CORAL && outtakeInputs.hasPiece) {
            targetState = State.HOLD;
        }

        double flip = reverseScore ? -1 : 1;
        switch (targetState) {
            case INTAKE_CORAL -> outtakeIO.setVoltage(Constants.kOuttakeRollerIntakeCoralVoltage.get());
            case INTAKE_ALGAE -> outtakeIO.setVoltage(Constants.kOuttakeRollerIntakeAlgaeVoltage.get());
            case SCORE_NOT_L4 -> outtakeIO.setVoltage(flip * switch (heldPiece) {
                case CORAL -> Constants.kOuttakeRollerScoreCoralVoltage.get();
                case ALGAE -> Constants.kOuttakeRollerScoreAlgaeVoltage.get();
            });
            case SCORE_L4 -> outtakeIO.setVoltage(flip * Constants.kOuttakeRollerScoreCoralL4Voltage.get());
            case HOLD -> {
                switch (heldPiece) {
                    case CORAL -> outtakeIO.setHoldPosition(outtakeInputs.positionAtPieceDetect + Constants.kOuttakeHoldPositionOffset.get());
                    case ALGAE -> outtakeIO.setVoltage(-Constants.kOuttakeRollerHoldAlgaeVoltage.get());
                }
            }
            case REVERSE -> outtakeIO.setVoltage(-Constants.kOuttakeRollerIntakeCoralVoltage.get());
        }
    }

    public void setReverseScore(boolean reverseScore) {
        this.reverseScore = reverseScore;
    }

    public boolean hasPiece() {
        return outtakeInputs.hasPiece;
    }
}
