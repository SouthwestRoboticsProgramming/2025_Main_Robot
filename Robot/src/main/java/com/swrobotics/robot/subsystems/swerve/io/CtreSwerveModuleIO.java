package com.swrobotics.robot.subsystems.swerve.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

/**
 * Swerve module implementation using the CTRE swerve API. This is a swerve
 * module with TalonFX motors and a CANcoder absolute encoder.
 */
public final class CtreSwerveModuleIO extends SwerveModuleIO {
    private final LegacySwerveModule module;
    private final StatusSignal<Angle> canCoderPosition;

    /**
     * @param name name of this module, for logging and debugging
     * @param constants CTRE swerve module constants
     * @param canBus Name of the CAN bus the devices are wired into. All three
     *               devices must be on the same CAN bus.
     */
    public CtreSwerveModuleIO(String name, LegacySwerveModuleConstants constants, String canBus) {
        super(name);
        module = new LegacySwerveModule(constants, canBus);

        // Limit drive current. This is needed because stalling all four drive
        // motors at full power (such as during heavy defense) consumes enough
        // current to trip the main breaker.
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLowerLimit = Constants.kDriveSupplyCurrentLimit;
        limits.SupplyCurrentLowerTime = Constants.kDriveCurrentLimitTime;
        limits.StatorCurrentLimitEnable = true;
        limits.StatorCurrentLimit = Constants.kDriveStatorCurrentLimit;
        module.getDriveMotor().getConfigurator().apply(limits);

        canCoderPosition = module.getCANcoder().getAbsolutePosition();

        MusicSubsystem.getInstance().addInstrument(module.getDriveMotor());
        MusicSubsystem.getInstance().addInstrument(module.getSteerMotor());
        MotorTrackerSubsystem.getInstance().addMotor(name + " Drive", module.getDriveMotor());
        MotorTrackerSubsystem.getInstance().addMotor(name + " Steer", module.getSteerMotor());

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.kPeriodicFreq,
                module.getDriveMotor().getPosition(),
                module.getDriveMotor().getVelocity(),
                module.getSteerMotor().getPosition(),
                module.getSteerMotor().getVelocity(),
                canCoderPosition
        );
        ParentDevice.optimizeBusUtilizationForAll(module.getDriveMotor(), module.getSteerMotor(), module.getCANcoder());
    }

    @Override
    public void updateInputs(Inputs inputs) {
        canCoderPosition.refresh();
        SwerveModulePosition position = module.getPosition(true);
        SwerveModuleState state = module.getCurrentState();

        inputs.drivePosition = position.distanceMeters;
        inputs.driveVelocity = state.speedMetersPerSecond;
        // Position's angle is latency compensated, so use it instead of state's angle
        inputs.angle = position.angle.getRotations();
        inputs.canCoderPos = canCoderPosition.getValueAsDouble();
    }

    @Override
    public void setCANcoderMagnetOffset(double offset) {
        MagnetSensorConfigs configs = new MagnetSensorConfigs();
        configs.MagnetOffset = offset;
        module.getCANcoder().getConfigurator().apply(configs);
    }

    @Override
    public void setTarget(SwerveModuleState state, DriveRequestType driveRequestType) {
        module.apply(state, driveRequestType, SteerRequestType.MotionMagic);
    }
}
