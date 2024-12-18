package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.pathfinding.PathEnvironment;
import com.swrobotics.lib.pathfinding.PathPlannerPathfinder;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveIO io;
    private final SwerveIO.Inputs inputs;

    public SwerveDriveSubsystem() {
        if (RobotBase.isReal())
            io = new CtreSwerveIO();
        else
            io = new SimSwerveIO();
        inputs = new SwerveIO.Inputs();

        final int driveMotorsPerModule = 1;

        ModuleConfig ppModuleConfig = new ModuleConfig(
                Units.inchesToMeters(Constants.kModuleConstantsFactory.WheelRadius),
                Constants.kDriveMaxAchievableSpeed,
                Constants.kDriveWheelCOF,
                DCMotor.getKrakenX60Foc(driveMotorsPerModule)
                        .withReduction(Constants.kModuleConstantsFactory.DriveMotorGearRatio),
                Constants.kDriveStatorCurrentLimit,
                driveMotorsPerModule);

        Translation2d[] positions = new Translation2d[Constants.kSwerveModuleInfos.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = Constants.kSwerveModuleInfos[i].position();
        }

        RobotConfig ppRobotConfig = new RobotConfig(
                Constants.kRobotMass,
                Constants.kRobotMOI,
                ppModuleConfig,
                positions);

        AutoBuilder.configure(
                this::getEstimatedPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> {
                    setControl(new SwerveRequest.ApplyRobotSpeeds()
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
        PathPlannerPathfinder.setEnvironment(PathEnvironment.EMPTY);
        PathfindingCommand.warmupCommand().schedule();
    }

    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return inputs.robotRelSpeeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        return inputs.modulePositions;
    }

    public Pose2d getEstimatedPose() {
        return inputs.estPose;
    }

    public void resetRotation(Rotation2d robotRotation) {
        io.resetRotation(robotRotation);
    }

    public void resetPose(Pose2d robotPose) {
        io.resetPose(robotPose);
    }

    public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {
        io.addVisionMeasurement(robotPose, timestamp, stdDevs);
    }

    public Rotation2d getRawGyroRotation() {
        return inputs.rawGyroRotation;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        FieldView.robotPose.setPose(inputs.estPose);
    }
}
