package com.swrobotics.robot.commands;

import com.swrobotics.lib.input.XboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RumblePatternCommands {
    private static final int BPM = 140;
    private static final double eightNoteSeconds = 60.0 / BPM / 2.0; // Assume 4:4

    public static Command endgameAlert(XboxController controller, double power) {
        // 1 and 3 4


        return Commands.sequence(
            rumbleForTimeCommand(controller, RumbleType.kBothRumble, power, eightNoteSeconds * 2),
            new WaitCommand(eightNoteSeconds),
            rumbleForTimeCommand(controller, RumbleType.kLeftRumble, power, eightNoteSeconds),
            rumbleForTimeCommand(controller, RumbleType.kRightRumble, power, eightNoteSeconds),
            new WaitCommand(eightNoteSeconds),
            rumbleForTimeCommand(controller, RumbleType.kBothRumble, power, eightNoteSeconds * 2)
        );
    }

    public static Command rumbleForTimeCommand(XboxController controller, RumbleType type, double amount, double timeSeconds) {
        return new RunCommand(() -> controller.setRumble(type, amount)).withTimeout(timeSeconds).finallyDo(() -> controller.setRumble(0));
    }
}
