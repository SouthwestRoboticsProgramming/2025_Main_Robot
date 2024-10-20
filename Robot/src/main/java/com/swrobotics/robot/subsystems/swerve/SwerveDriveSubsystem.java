package com.swrobotics.robot.subsystems.swerve;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.lib.pathfinding.PathPlannerPathfinder;
import com.swrobotics.robot.subsystems.swerve.io.*;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.swrobotics.lib.field.FieldInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// TODO: This whole subsystem needs a rewrite due to CTRE swerve API changes
public final class SwerveDriveSubsystem extends SubsystemBase {
    private static final NTBoolean CALIBRATE_OFFSETS = new NTBoolean("Drive/Modules/Calibrate", false);

    public enum Priority {
        // Priorities defined lower here take precedence
        IDLE,
        DRIVER,
        AUTO;

        public boolean takesPrecedenceOver(Priority other) {
            return ordinal() > other.ordinal();
        }

        public Priority max(Priority other) {
            if (takesPrecedenceOver(other))
                return this;
            return other;
        }
    }

    // Priority should be one of the priority levels above
    public static record DriveRequest(Priority priority, Translation2d robotRelTranslation, DriveRequestType type) {
    }

    public static record TurnRequest(Priority priority, Rotation2d turn) {
    }

    private static final DriveRequest NULL_DRIVE = new DriveRequest(Priority.IDLE, new Translation2d(0, 0), DriveRequestType.OpenLoopVoltage);
    private static final TurnRequest NULL_TURN = new TurnRequest(Priority.IDLE, new Rotation2d(0));

    private final MotorTrackerSubsystem motorTracker;
    private final GyroIO gyroIO;
    private final GyroIO.Inputs gyroInputs;
    private final SwerveModuleIO[] moduleIOs;
    private final SwerveModuleIO.Inputs[] moduleInputs;

    private final SwerveKinematics kinematics;
    private final SwerveEstimator estimator;
    private final SwerveSetpointGenerator setpointGenerator;

    private SwerveModulePosition[] prevPositions;
    private double prevGyroAngle;
    private SwerveSetpoints prevSetpoints;

    private DriveRequest currentDriveRequest;
    private TurnRequest currentTurnRequest;
    private SwerveKinematicLimits limits;
    private Priority lastSelectedPriority;

    public SwerveDriveSubsystem(MotorTrackerSubsystem motorTracker) {
        this.motorTracker = motorTracker;
        if (RobotBase.isReal()) {
            gyroIO = new NavXGyroIO();
        } else {
            gyroIO = new SimGyroIO();
        }
        gyroInputs = new GyroIO.Inputs();

        SwerveModuleInfo[] infos = Constants.kSwerveModuleInfos;

        moduleIOs = new SwerveModuleIO[infos.length];
        moduleInputs = new SwerveModuleIO.Inputs[infos.length];
        Translation2d[] positions = new Translation2d[infos.length];
        for (int i = 0; i < moduleIOs.length; i++) {
            SwerveModuleInfo info = infos[i];

            if (RobotBase.isReal()) {
                LegacySwerveModuleConstants moduleConstants = Constants.kSwerveConstantsFactory.createModuleConstants(
                        info.turnId(), info.driveId(), info.encoderId(),
                        info.offset().get(),
                        info.position().getX(), info.position().getY(),
                        false);

                moduleIOs[i] = new CtreSwerveModuleIO(info.name(), moduleConstants, info.canBus());
            } else {
                moduleIOs[i] = new SimSwerveModuleIO(info.name(), info.offset().get());
            }

            positions[i] = info.position();
            moduleInputs[i] = new SwerveModuleIO.Inputs();
        }

        this.kinematics = new SwerveKinematics(positions);
        this.estimator = new SwerveEstimator();
        this.setpointGenerator = new SwerveSetpointGenerator(positions);

        prevPositions = null;
        prevSetpoints = SwerveSetpoints.createInitial(infos.length);
        currentDriveRequest = NULL_DRIVE;
        currentTurnRequest = NULL_TURN;
        limits = Constants.kDriveLimits;

        final int driveMotorsPerModule = 1;

        ModuleConfig ppModuleConfig = new ModuleConfig(
                Units.inchesToMeters(Constants.kSwerveConstantsFactory.WheelRadius),
                Constants.kDriveMaxAchievableSpeed,
                Constants.kDriveWheelCOF,
                DCMotor.getKrakenX60Foc(driveMotorsPerModule)
                    .withReduction(Constants.kSwerveConstantsFactory.DriveMotorGearRatio),
                Constants.kDriveStatorCurrentLimit,
                driveMotorsPerModule);

        RobotConfig ppRobotConfig = new RobotConfig(
                Constants.kRobotMass,
                Constants.kRobotMOI,
                ppModuleConfig,
                Constants.kDriveWheelSpacingY,
                Constants.kDriveWheelSpacingX);

        // TODO: Check GUI settings and make sure they match ppRobotConfig

        // Configure PathPlanner
        AutoBuilder.configure(
                this::getEstimatedPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> {
                    // TODO: Use the feedforwards somehow
                    driveAndTurn(Priority.AUTO, speeds, DriveRequestType.Velocity);
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

    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[moduleIOs.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = moduleInputs[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getCurrentModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[moduleIOs.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = moduleInputs[i].getPosition();
        }
        return positions;
    }

    public void driveAndTurn(Priority priority, ChassisSpeeds speeds, DriveRequestType type) {
        drive(new DriveRequest(priority, new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), type));
        turn(new TurnRequest(priority, new Rotation2d(speeds.omegaRadiansPerSecond)));
    }

    public void drive(DriveRequest request) {
        if (request.priority.takesPrecedenceOver(currentDriveRequest.priority))
            currentDriveRequest = request;
    }

    public void turn(TurnRequest request) {
        if (request.priority.takesPrecedenceOver(currentTurnRequest.priority))
            currentTurnRequest = request;
    }

    private void calibrate() {
        for (int i = 0; i < moduleIOs.length; i++) {
            SwerveModuleInfo info = Constants.kSwerveModuleInfos[i];

            double position = moduleInputs[i].canCoderPos;
            info.offset().set(info.offset().get() - position);
            moduleIOs[i].setCANcoderMagnetOffset(info.offset().get());
        }
    }

    @Override
    public void periodic() {
        // if (overheating) { dont(); }
        if (motorTracker.isOverheating() && !DriverStation.isFMSAttached()) {
            currentDriveRequest = NULL_DRIVE;
            currentTurnRequest = NULL_TURN;
        }

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);

        for (int i = 0; i < moduleIOs.length; i++) {
            SwerveModuleIO io = moduleIOs[i];
            SwerveModuleIO.Inputs inputs = moduleInputs[i];

            io.updateInputs(inputs);
            Logger.processInputs("Swerve Modules/" + io.getName(), inputs);
        }
        Logger.recordOutput("Drive/Module Positions", getCurrentModulePositions());
        Logger.recordOutput("Drive/Module States", getCurrentModuleStates());

        if (CALIBRATE_OFFSETS.get()) {
            CALIBRATE_OFFSETS.set(false);
            calibrate();
        }

        // Update estimator
        SwerveModulePosition[] positions = getCurrentModulePositions();
        if (prevPositions != null) {
            Twist2d twist = kinematics.getTwistDelta(prevPositions, positions);

            // We trust the gyro more than the kinematics estimate
            if (gyroInputs.connected) {
                twist.dtheta = gyroInputs.yaw - prevGyroAngle;
            }

            estimator.update(twist);
        }
        prevPositions = positions;
        prevGyroAngle = gyroInputs.yaw;

        // Apply drive request when robot enabled
        if (!DriverStation.isDisabled()) {
            ChassisSpeeds requestedSpeeds = new ChassisSpeeds(
                    currentDriveRequest.robotRelTranslation.getX(),
                    currentDriveRequest.robotRelTranslation.getY(),
                    currentTurnRequest.turn.getRadians());
            lastSelectedPriority = currentDriveRequest.priority.max(currentTurnRequest.priority);
            currentDriveRequest = NULL_DRIVE;
            currentTurnRequest = NULL_TURN;

            // Apply the drive request
            requestedSpeeds = ChassisSpeeds.discretize(requestedSpeeds, Constants.kDriveDriftComp);
            SwerveSetpoints setpoints = setpointGenerator.generateSetpoint(
                    limits, prevSetpoints, requestedSpeeds, Constants.kPeriodicTime);
            SwerveModuleState[] moduleSetpoints = setpoints.moduleStates;
            for (int i = 0; i < moduleIOs.length; i++) {
                moduleIOs[i].setTarget(moduleSetpoints[i], currentDriveRequest.type);
            }
            prevSetpoints = setpoints;
            Logger.recordOutput("Drive/Module Desired Setpoints", setpoints.desiredModuleStates);
            Logger.recordOutput("Drive/Module Setpoints", moduleSetpoints);
        }
    }

    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPose();
    }

    public void setPose(Pose2d newPose) {
        estimator.resetPose(newPose);
    }

    public void setRotation(Rotation2d newRotation) {
        estimator.resetPose(new Pose2d(getEstimatedPose().getTranslation(), newRotation));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getCurrentModuleStates());
    }

    public Translation2d toRobotRelativeDrive(Translation2d fieldRelDrive) {
        return fieldRelDrive.rotateBy(getEstimatedPose().getRotation().unaryMinus());
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        ChassisSpeeds robotRel = getRobotRelativeSpeeds();

        Rotation2d facing = getEstimatedPose().getRotation();
        Translation2d tx = new Translation2d(robotRel.vxMetersPerSecond, robotRel.vyMetersPerSecond)
                .rotateBy(facing);

        return new ChassisSpeeds(
                tx.getX(), tx.getY(),
                robotRel.omegaRadiansPerSecond
        );
    }

    public Rotation2d getRawGyroRotation() {
        return Rotation2d.fromRadians(gyroInputs.yaw);
    }

    public Priority getLastSelectedPriority() {
        return lastSelectedPriority;
    }

    public void setEstimatorIgnoreVision(boolean ignoreVision) {
        estimator.setIgnoreVision(ignoreVision);
    }

    public void setKinematicLimits(SwerveKinematicLimits limits) {
        this.limits = limits;
    }
}
