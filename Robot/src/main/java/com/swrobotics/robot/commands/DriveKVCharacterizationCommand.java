package com.swrobotics.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class DriveKVCharacterizationCommand extends Command {
    private final double kVolts = 6;
    private final double kAccelTime = 0.3;

    private final SwerveDriveSubsystem drive;
    private final Timer timer;
    private boolean hasStartedMeasuring;
    private SwerveModulePosition[] initialPositions;

    public DriveKVCharacterizationCommand(SwerveDriveSubsystem drive) {
        this.drive = drive;
        timer = new Timer();
        hasStartedMeasuring = false;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        drive.setControl(new SwerveRequest.SysIdSwerveTranslation()
                .withVolts(6));

        if (!hasStartedMeasuring) {
            if (timer.hasElapsed(kAccelTime)) {
                hasStartedMeasuring = true;
                timer.restart();

                initialPositions = drive.getModulePositions().clone();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!hasStartedMeasuring) {
            DriverStation.reportError("Didn't start measuring yet!", false);
            return;
        }

        double measurementTime = timer.get();
        SwerveModulePosition[] finalPositions = drive.getModulePositions();

        double avgDisplacement = 0;
        for (int i = 0; i < 4; i++) {
            avgDisplacement += Math.abs(finalPositions[i].distanceMeters - initialPositions[i].distanceMeters);
        }
        avgDisplacement /= 4;

        double avgVelocity = avgDisplacement / measurementTime;

        double rotorVel = avgVelocity / Constants.kModuleConstantsFactory.WheelRadius / Constants.kModuleConstantsFactory.DriveMotorGearRatio;

        double kV = kVolts / rotorVel;

        // Report result as warning so it shows up in the driver station
        DriverStation.reportWarning("Drive kV: " + kV, false);

        // Also log to AdvantageKit in case we miss the message
        Logger.recordOutput("Drive/Characterized kV", kV);
    }
}
