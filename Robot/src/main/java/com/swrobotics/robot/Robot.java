package com.swrobotics.robot;

import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.logging.Logging;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.LoggedRobot;

/**
 * The main robot class.
 */
public final class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        Logging.initialize(RobotContainer.SIM_MODE);

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

        // Start autonomous command
        Command auto = robotContainer.getAutonomousCommand();
        double delay = robotContainer.getAutoDelay();
        if (delay > 0) {
            autonomousCommand = Commands.sequence(
                    Commands.waitSeconds(delay),
                    // Use a proxy here so that running the same auto twice
                    // does not crash the robot code. Directly adding it to the
                    // sequence would mark the auto command as composed,
                    // causing the second time to throw an exception
                    auto.asProxy()
            );
        } else {
            autonomousCommand = auto;
        }
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
