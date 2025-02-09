package com.swrobotics.robot.subsystems.motortracker;

import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.robot.config.Constants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem to track the status of all the motors in the robot.
 */
public final class MotorTrackerSubsystem extends SubsystemBase {
    private static MotorTrackerSubsystem instance = null;

    public static MotorTrackerSubsystem getInstance() {
        if (instance == null)
            throw new IllegalStateException("MotorTrackerSubsystem is not yet initialized");
        return instance;
    }

    private final MotorTrackerIO io;
    private final MotorTrackerIO.Inputs inputs;
    private boolean overheating;

    public MotorTrackerSubsystem() {
        if (instance != null)
            throw new IllegalStateException("MotorTrackerSubsystem already initialized");
        instance = this;

        if (RobotBase.isReal())
            io = new RealMotorTrackerIO();
        else
            io = new SimMotorTrackerIO();
        inputs = new MotorTrackerIO.Inputs();

        overheating = false;
    }

    /**
     * Adds a motor to be monitored.
     *
     * @param name name of the motor for logging
     * @param motor motor to add
     */
    public void addMotor(String name, TalonFX motor) {
        io.addMotor(name, motor);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Motor Status", inputs);

        overheating = false;
        for (double temp : inputs.temperatures) {
            if (temp > Constants.kOverheatingThreshold) {
                overheating = true;
                break;
            }
        }

        Logger.recordOutput("Overheating", overheating);
    }

    public boolean isOverheating() {
        return overheating;
    }
}
