package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;

public class CtreSwerveIO implements SwerveIO {
    private final SwerveDrivetrain drivetrain;
    private final StatusSignal<Angle> rawGyroAngleSignal;

    public CtreSwerveIO() {
        int kModuleCount = Constants.kSwerveModuleInfos.length;
        SwerveModuleConstants[] moduleConstants = new SwerveModuleConstants[kModuleCount];
        for (int i = 0; i < kModuleCount; i++) {
            SwerveModuleInfo info = Constants.kSwerveModuleInfos[i];
            moduleConstants[i] = Constants.kModuleConstantsFactory.createModuleConstants(
                    info.driveId(),
                    info.turnId(),
                    info.encoderId(),
                    info.offset().get(),
                    info.position().getX(),
                    info.position().getY(),
                    false,
                    true,
                    false
            );
        }

        drivetrain = new SwerveDrivetrain(
                Constants.kDrivetrainConstants,
                Constants.kOdometryUpdateFreq,
                Constants.kOdometryStdDevs,
                // These values have no effect, they are overridden by the
                // standard deviations given to drivetrain.addVisionMeasurement()
                VecBuilder.fill(0.6, 0.6, 0.6),
                moduleConstants
        );

        for (int i = 0; i < kModuleCount; i++) {
            String name = Constants.kSwerveModuleInfos[i].name();

            SwerveModule module = drivetrain.getModule(i);
            MotorTrackerSubsystem.getInstance().addMotor(name + " Drive", module.getDriveMotor());
            MotorTrackerSubsystem.getInstance().addMotor(name + " Steer", module.getSteerMotor());
            MusicSubsystem.getInstance().addInstrument(module.getDriveMotor());
            MusicSubsystem.getInstance().addInstrument(module.getSteerMotor());

            CurrentLimitsConfigs driveLimits = new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.kDriveStatorCurrentLimit)
                    .withSupplyCurrentLowerLimit(Constants.kDriveSupplyCurrentLimit)
                    .withSupplyCurrentLowerTime(Constants.kDriveCurrentLimitTime);
            module.getDriveMotor().getConfigurator().apply(driveLimits);
        }

        // Set operator perspective to field +X axis (0 degrees) so coordinate
        // system stays centered on blue alliance origin
        drivetrain.setOperatorPerspectiveForward(new Rotation2d(0));

        rawGyroAngleSignal = drivetrain.getPigeon2().getYaw();
    }

    @Override
    public void updateInputs(Inputs inputs) {
        SwerveDrivetrain.SwerveDriveState state = drivetrain.getState();
        inputs.estPose = state.Pose;
        inputs.robotRelSpeeds = state.Speeds;
        inputs.moduleStates = state.ModuleStates;
        inputs.moduleTargets = state.ModuleTargets;
        inputs.modulePositions = state.ModulePositions;
        inputs.odometryPeriod = state.OdometryPeriod;
        inputs.successfulDaqs = state.SuccessfulDaqs;
        inputs.failedDaqs = state.FailedDaqs;

        inputs.rawGyroRotation = Rotation2d.fromDegrees(rawGyroAngleSignal.refresh().getValueAsDouble());
    }

    @Override
    public void setControl(SwerveRequest request) {
        drivetrain.setControl(request);
    }

    @Override
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        drivetrain.resetRotation(rotation);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        drivetrain.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestamp), stdDevs);
    }

    @Override
    public void calibrateModuleOffsets() {
        SwerveModule[] modules = drivetrain.getModules();
        for (int i = 0; i < modules.length; i++) {
            CANcoder canCoder = modules[i].getCANcoder();
            NTEntry<Double> offset = Constants.kSwerveModuleInfos[i].offset();

            double position = canCoder
                    .getAbsolutePosition(true)
                    .getValueAsDouble();

            offset.set(offset.get() - position);
            canCoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(offset.get()));
        }
    }

    public SwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
