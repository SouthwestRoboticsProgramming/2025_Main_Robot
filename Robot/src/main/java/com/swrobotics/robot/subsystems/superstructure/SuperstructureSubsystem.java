package com.swrobotics.robot.subsystems.superstructure;

import com.swrobotics.robot.logging.RobotView;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class SuperstructureSubsystem extends SubsystemBase {
    public enum State {
        RECEIVE_CORAL_FROM_INDEXER(Elevator.Position.DOWN, OuttakePivot.Position.IN),
        SCORE_L1(Elevator.Position.SCORE_L1, OuttakePivot.Position.SCORE_L1),
        SCORE_L2(Elevator.Position.SCORE_L2, OuttakePivot.Position.SCORE_L2L3),
        SCORE_L3(Elevator.Position.SCORE_L3, OuttakePivot.Position.SCORE_L2L3),
        SCORE_L4(Elevator.Position.SCORE_L4, OuttakePivot.Position.SCORE_L4);

        public static State forScoring(int level) {
            return switch (level) {
                case 1 -> SCORE_L1;
                case 2 -> SCORE_L2;
                case 3 -> SCORE_L3;
                case 4 -> SCORE_L4;
                default -> throw new IndexOutOfBoundsException(level);
            };
        }

        public final Elevator.Position elevatorPosition;
        public final OuttakePivot.Position pivotPosition;

        State(Elevator.Position elevatorPosition, OuttakePivot.Position pivotPosition) {
            this.elevatorPosition = elevatorPosition;
            this.pivotPosition = pivotPosition;
        }
    }

    private final Elevator elevator;
    private final OuttakePivot pivot;

    private State targetState;
    private boolean waiting;

    public SuperstructureSubsystem() {
        elevator = new Elevator();
        pivot = new OuttakePivot();

        targetState = State.RECEIVE_CORAL_FROM_INDEXER;
        waiting = false;
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
        elevator.updateInputs();
        pivot.updateInputs();
        RobotView.setSuperstructureState(elevator.getCurrentHeight(), pivot.getCurrentAngle());

        Elevator.Position elevatorTarget = targetState.elevatorPosition;
        OuttakePivot.Position pivotTarget = targetState.pivotPosition;

        // Prevent elevator and pivot from colliding with each other
        // Each one waits for the other to get out of the way
        waiting = false;
        if (elevatorTarget != Elevator.Position.DOWN && !pivot.isSafeToMoveElevatorUp()) {
            elevatorTarget = Elevator.Position.WAIT_FOR_PIVOT;
            waiting = true;
        }
        if (pivotTarget == OuttakePivot.Position.IN && !elevator.isSafeToMovePivotIn()) {
            pivotTarget = OuttakePivot.Position.WAIT_FOR_ELEVATOR;
            waiting = true;
        }

        elevator.setTargetPosition(elevatorTarget);
        pivot.setTargetPosition(pivotTarget);
    }

    public boolean isInTolerance() {
        return !waiting && elevator.isInTolerance() && pivot.isInTolerance();
    }
}
