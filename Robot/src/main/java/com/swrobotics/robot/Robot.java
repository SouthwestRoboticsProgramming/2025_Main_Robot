package com.swrobotics.robot;

import com.swrobotics.lib.ThreadUtils;

import com.swrobotics.lib.tunable.LoggedTunableValue;
import com.swrobotics.robot.logging.Logging;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * The main robot class.
 */
// TODO: Merge with RobotContainer??
public final class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private final Timer autonomousTimer = new Timer();
    private double autonomousDelay;
    private boolean hasScheduledAuto;

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

        LoggedTunableValue.updateAll();
        ThreadUtils.runMainThreadOperations();
        CommandScheduler.getInstance().run(); // Leave this alone
    }

    @Override
    public void autonomousInit() {
        // If an autonomous command has already be set, reset it
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            System.out.println("Canceled the current auto command");
        }

        // Get autonomous from selector
        autonomousCommand = robotContainer.getAutonomousCommand();

        autonomousDelay = robotContainer.getAutoDelay();
        autonomousTimer.restart();
        hasScheduledAuto = false;

        robotContainer.drive.setEstimatorIgnoreVision(true);
    }

    @Override
    public void autonomousPeriodic() {
        // Manually time auto delay since using sequential group causes crash
        // when running the same auto twice
        if (!hasScheduledAuto && autonomousCommand != null && autonomousTimer.hasElapsed(autonomousDelay)) {
            autonomousCommand.schedule();
            hasScheduledAuto = true;
        }
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
    @Override public void teleopPeriodic() {}
    @Override public void testPeriodic() {}
}
