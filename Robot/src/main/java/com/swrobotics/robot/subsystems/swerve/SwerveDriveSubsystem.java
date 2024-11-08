package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.lib.pathfinding.PathPlannerPathfinder;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// TODO: AdvantageKit-ify
public final class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDrivetrain drivetrain;

    public SwerveDriveSubsystem() {
        drivetrain = configureDrivetrain();
        configurePathPlanner();
    }

    private SwerveDrivetrain configureDrivetrain() {
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
                    true
            );
        }

        SwerveDrivetrain drivetrain = new SwerveDrivetrain(
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

        return drivetrain;
    }

    private void configurePathPlanner() {
        final int driveMotorsPerModule = 1;

        ModuleConfig ppModuleConfig = new ModuleConfig(
                Units.inchesToMeters(Constants.kModuleConstantsFactory.WheelRadius),
                Constants.kDriveMaxAchievableSpeed,
                Constants.kDriveWheelCOF,
                DCMotor.getKrakenX60Foc(driveMotorsPerModule)
                        .withReduction(Constants.kModuleConstantsFactory.DriveMotorGearRatio),
                Constants.kDriveStatorCurrentLimit,
                driveMotorsPerModule);

        RobotConfig ppRobotConfig = new RobotConfig(
                Constants.kRobotMass,
                Constants.kRobotMOI,
                ppModuleConfig,
                Constants.kDriveWheelSpacingY,
                Constants.kDriveWheelSpacingX);

        AutoBuilder.configure(
                this::getEstimatedPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> {
                    setControl(new SwerveRequest.ApplyChassisSpeeds()
                            .withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity));
                },
                new PPHolonomicDriveController(
                        new PIDConstants(Constants.kAutoDriveKp, Constants.kAutoDriveKd),
                        new PIDConstants(Constants.kAutoTurnKp.get(), Constants.kAutoTurnKd.get())
                ),
                ppRobotConfig,
                () -> FieldInfo.getAlliance() == DriverStation.Alliance.Red,
                this
        );

        PathPlannerLogging.setLogActivePathCallback((path) -> {
            FieldView.pathPlannerPath.setPoses(path);
            Logger.recordOutput("PathPlanner/Active Path", path.toArray(new Pose2d[0]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((target) -> {
            FieldView.pathPlannerSetpoint.setPose(target);
            Logger.recordOutput("PathPlanner/Target Pose", target);
        });

        Pathfinding.setPathfinder(new PathPlannerPathfinder());
    }

    public void setControl(SwerveRequest request) {
        drivetrain.setControl(request);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getState().Speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        return drivetrain.getState().ModulePositions;
    }

    public Pose2d getEstimatedPose() {
        return drivetrain.getState().Pose;
    }

    public void resetRotation(Rotation2d robotRotation) {
        resetPose(new Pose2d(getEstimatedPose().getTranslation(), robotRotation));
    }

    public void resetPose(Pose2d robotPose) {
        drivetrain.seedFieldRelative(robotPose);
    }

    public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {
        drivetrain.addVisionMeasurement(robotPose, timestamp, stdDevs);
    }

    public Rotation2d getRawGyroRotation() {
        return drivetrain.getPigeon2().getRotation2d();
    }

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

    @Override
    public void periodic() {
        FieldView.robotPose.setPose(getEstimatedPose());
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateSimState(Constants.kPeriodicTime, 12.0);
    }
}
