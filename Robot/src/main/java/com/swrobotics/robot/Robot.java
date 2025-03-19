package com.swrobotics.robot;

import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.logging.Logging;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/**
 * The main robot class.
 */
public final class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        Logging.initialize(RobotContainer.SIM_MODE);
        if (RobotBase.isSimulation() && RobotContainer.SIM_MODE == Logging.SimMode.REPLAY) {
            setUseTiming(RobotContainer.REPLAY_REAL_TIME);
        }

        // Create a RobotContainer to manage our subsystems and our buttons
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);

        NTEntry.updateAll();
        CommandScheduler.getInstance().run(); // Leave this alone
    }

    @Override
    public void autonomousInit() {
        // If an autonomous command has already be set, reset it
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            System.out.println("Canceled the current auto command");
        }

        autonomousCommand = robotContainer.getAutonomousCommand();

        // Prevent crash if the same auto is run twice
        CommandScheduler.getInstance().removeComposedCommand(autonomousCommand);

        // Add delay if needed
        double delay = robotContainer.getAutoDelay();
        if (delay > 0) {
            autonomousCommand = Commands.sequence(
                    Commands.waitSeconds(delay),
                    autonomousCommand
            );
        }

        // Log whether auto was cancelled
        autonomousCommand = autonomousCommand
                .finallyDo((cancelled) -> {
                    Logger.recordOutput("Auto/Command Cancelled", cancelled);
                    if (cancelled)
                        DriverStation.reportWarning("Auto command ended early", false);
                });

        // For timing tests in simulator
        if (RobotBase.isSimulation()) {
            autonomousCommand = autonomousCommand
                    .withTimeout(15);
        }

        // Measure elapsed time
        double startTimestamp = Timer.getTimestamp();
        autonomousCommand = autonomousCommand
                .finallyDo(() -> {
                    double endTimestamp = Timer.getTimestamp();

                    Logger.recordOutput("Auto/Command Time", endTimestamp - startTimestamp);
                    System.out.println("Auto command took " + (endTimestamp - startTimestamp) + " seconds");
                });

        // Start autonomous command
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void disabledInit() {
        robotContainer.disabledInit();
    }

    @Override
    public void disabledExit() {
        robotContainer.disabledExit();
    }

    // Override these so WPILib doesn't print unhelpful warnings
    @Override public void simulationPeriodic() {}
    @Override public void disabledPeriodic() {}
    @Override public void autonomousPeriodic() {}
    @Override public void teleopPeriodic() {}
    @Override public void testPeriodic() {}
}
