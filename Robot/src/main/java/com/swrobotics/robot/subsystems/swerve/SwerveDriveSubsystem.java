package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.pathfinding.pathplanner.AutoBuilderExt;
import com.swrobotics.lib.pathfinding.pathplanner.SyncPathfinder;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class SwerveDriveSubsystem extends SubsystemBase {
    private static final NTBoolean CALIBRATE = new NTBoolean("Drive/Modules/Calibrate", false);

    private final SwerveIO io;
    private final SwerveIO.Inputs inputs;

    public SwerveDriveSubsystem() {
        if (RobotBase.isReal())
            io = new CtreSwerveIO();
        else
            io = new SimSwerveIO();
        inputs = new SwerveIO.Inputs();

        AutoBuilderExt.configure(
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
                Constants.kPathPlannerRobotConfig,
                () -> FieldInfo.getAlliance() == DriverStation.Alliance.Red,
                new SyncPathfinder(),
                this
        );

        PathPlannerLogging.setLogActivePathCallback((path) -> {
            if (path != null) {
                FieldView.pathPlannerPath.setPoses(path);
                if (path.isEmpty())
                    FieldView.pathPlannerSetpoint.setPoses();

                Logger.recordOutput("PathPlanner/Active Path", path.toArray(new Pose2d[0]));
            } else {
                FieldView.pathPlannerSetpoint.setPoses();
            }
        });
        PathPlannerLogging.setLogTargetPoseCallback((target) -> {
            if (target != null)
                FieldView.pathPlannerSetpoint.setPose(target);
            Logger.recordOutput("PathPlanner/Target Pose", target);
        });
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

    public boolean isCloseTo(Translation2d position, double tolerance) {
        double distance = getEstimatedPose().getTranslation().getDistance(position);
        return distance <= tolerance;
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

    public double getFFCharacterizationVelocity() {
        double avgVelocity = 0;
        for (SwerveModuleState state : inputs.moduleStates) {
            avgVelocity += Math.abs(state.speedMetersPerSecond);
        }
        avgVelocity /= 4;

        return avgVelocity;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        FieldView.robotPose.setPose(inputs.estPose);

        if (CALIBRATE.get()) {
            CALIBRATE.set(false);
            io.calibrateModuleOffsets();
        }
    }
}
