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

    public enum CoralState {
        INTAKE_CORAL,
        SCORE_NOT_L4,
        SCORE_L4,
        HOLD,
        REVERSE
    }

    public enum AlgaeState {
        OFF,
        INTAKE,
        HOLD,
        SCORE
    }

    private final CoralOuttakeIO outtakeIO;
    private final CoralOuttakeIO.Inputs outtakeInputs;

    private CoralState targetCoralState;
    private AlgaeState targetAlgaeState;
    private double holdPosition;

    public OuttakeSubsystem() {
        if (RobotBase.isReal()) {
            outtakeIO = new CoralOuttakeIOReal();
        } else {
            outtakeIO = new CoralOuttakeIOSim();
        }

        outtakeInputs = new CoralOuttakeIO.Inputs();

        targetCoralState = CoralState.HOLD;
        targetAlgaeState = AlgaeState.OFF;
    }

    public void setTargetCoralState(CoralState targetCoralState) {
        this.targetCoralState = targetCoralState;
    }

    public void setTargetAlgaeState(AlgaeState targetCoralState) {
        this.targetAlgaeState = targetCoralState;
    }

    public Command commandSetCoralState(CoralState targetCoralState) {
        return Commands.run(() -> setTargetCoralState(targetCoralState), this);
    }

    public Command commandSetAlgaeState(AlgaeState targetCoralState) {
        return Commands.run(() -> setTargetAlgaeState(targetCoralState), this);
    }

    public Command commandSetCoralStateOnce(CoralState targetCoralState) {
        return Commands.runOnce(() -> setTargetCoralState(targetCoralState), this);
    }

    public Command commandSetAlgaeStateOnce(AlgaeState targetCoralState) {
        return Commands.runOnce(() -> setTargetAlgaeState(targetCoralState), this);
    }

    public Command score(double seconds, boolean l4) {
        return commandSetCoralState(l4 ? CoralState.SCORE_L4 : CoralState.SCORE_NOT_L4)
                .withTimeout(seconds)
                .finallyDo(() -> setTargetCoralState(CoralState.HOLD));
    }

    @Override
    public void periodic() {
        outtakeIO.setBeamBreakIgnored(targetCoralState == CoralState.SCORE_NOT_L4 || targetCoralState == CoralState.SCORE_L4 || targetCoralState == CoralState.REVERSE);

        outtakeIO.updateInputs(outtakeInputs);
        Logger.processInputs("Outtake", outtakeInputs);

        if (targetCoralState == CoralState.INTAKE_CORAL && outtakeInputs.hasPiece) {
            targetCoralState = CoralState.HOLD;
        }

        switch (targetCoralState) {
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
