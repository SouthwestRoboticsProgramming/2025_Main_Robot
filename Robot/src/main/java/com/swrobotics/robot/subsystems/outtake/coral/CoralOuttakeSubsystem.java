package com.swrobotics.robot.subsystems.outtake.coral;

import org.littletonrobotics.junction.Logger;

import com.swrobotics.robot.config.Constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralOuttakeSubsystem extends SubsystemBase {
    public enum State {
        INTAKE_CORAL,
        SCORE_NOT_L4,
        SCORE_L4,
        HOLD,
        REVERSE
    }

    private final CoralOuttakeIO outtakeIO;
    private final CoralOuttakeIO.Inputs outtakeInputs;

    private State targetState;

    public CoralOuttakeSubsystem() {
        if (RobotBase.isReal()) {
            outtakeIO = new CoralOuttakeIOReal();
        } else {
            outtakeIO = new CoralOuttakeIOSim();
        }

        outtakeInputs = new CoralOuttakeIO.Inputs();

        targetState = State.HOLD;
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

    public Command score(double seconds, boolean l4) {
        return commandSetState(l4 ? State.SCORE_L4 : State.SCORE_NOT_L4)
                .withTimeout(seconds)
                .finallyDo(() -> setTargetState(State.HOLD));
    }

    @Override
    public void periodic() {
        outtakeIO.setBeamBreakIgnored(targetState == State.SCORE_NOT_L4 || targetState == State.SCORE_L4 || targetState == State.REVERSE);

        outtakeIO.updateInputs(outtakeInputs);
        Logger.processInputs("Coral Outtake", outtakeInputs);

        if (targetState == State.INTAKE_CORAL && outtakeInputs.hasPiece) {
            targetState = State.HOLD;
        }

        switch (targetState) {
            case INTAKE_CORAL -> outtakeIO.setVoltage(Constants.kOuttakeRollerIntakeCoralVoltage.get());
            case SCORE_NOT_L4 -> outtakeIO.setVoltage(Constants.kOuttakeRollerScoreCoralVoltage.get());
            case SCORE_L4 -> outtakeIO.setVoltage(Constants.kOuttakeRollerScoreCoralL4Voltage.get());
            case HOLD -> outtakeIO.setHoldPosition(outtakeInputs.positionAtPieceDetect + Constants.kOuttakeHoldPositionOffset.get());
            case REVERSE -> outtakeIO.setVoltage(-Constants.kOuttakeRollerIntakeCoralVoltage.get());
        }
    }

    public boolean hasPiece() {
        return outtakeInputs.hasPiece;
    }
}
