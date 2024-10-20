package com.swrobotics.robot.config;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.subsystems.swerve.SwerveKinematicLimits;
import com.swrobotics.robot.subsystems.swerve.SwerveModuleInfo;
import com.swrobotics.robot.subsystems.vision.RawAprilTagSource;
import com.swrobotics.robot.subsystems.vision.tagtracker.TagTrackerCaptureProperties;
import edu.wpi.first.math.util.Units;

// Use NTEntry when you want tunable
// Use double when value has been tuned in so it can't accidentally change
public final class Constants {
    public static final int kPeriodicFreq = 50; // Hz
    public static final double kPeriodicTime = 1.0 / kPeriodicFreq;

    // We don't know what the 2025 field is yet :(
    public static final FieldInfo kField = FieldInfo.CRESCENDO_2024;
    public static final int kEndgameAlertTime = 15;

    // Robot dimensions
    public static final double kFrameLength = 0.77; // m
    public static final double kFrameWidth = 0.695; // m
    public static final double kRobotRadius = 0.6202230647076; // m, diagonal including bumpers
    public static final double kRobotMass = Units.lbsToKilograms(122 + 14 + 14);
    // Approximation of robot as uniform cuboid
    // See https://sleipnirgroup.github.io/Choreo/usage/estimating-moi/
    // FIXME: Measure in CAD
    public static final double kRobotMOI = 1.0/12.0 * kRobotMass * (kFrameLength*kFrameLength + kFrameWidth*kFrameWidth);

    // Controls
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDeadband = 0.15;
    public static final double kTriggerThreshold = 0.3;

    public static final double kDriveControlMaxAccel = 5.5; // m/s^2
    public static final double kDriveControlMaxTurnSpeed = 1; // rot/s
    public static final double kDriveControlDrivePower = 2; // Exponent input is raised to
    public static final double kDriveControlTurnPower = 2;

    // Auto (TODO: Tune)
    public static final double kAutoDriveKp = 8;
    public static final double kAutoDriveKd = 0;

    public static final NTEntry<Double> kAutoTurnKp = new NTDouble("Auto/Turn PID/kP", 9).setPersistent();
    public static final NTEntry<Double> kAutoTurnKd = new NTDouble("Auto/Turn PID/kD", 0.5).setPersistent();

    // Drive
    public static final double kDriveWheelSpacingX = 55.3 / 100; // m
    public static final double kDriveWheelSpacingY = 63.0 / 100; // m

    public static final NTEntry<Double> kFrontLeftOffset = new NTDouble("Drive/Modules/Front Left Offset (rot)", 0).setPersistent();
    public static final NTEntry<Double> kFrontRightOffset = new NTDouble("Drive/Modules/Front Right Offset (rot)", 0).setPersistent();
    public static final NTEntry<Double> kBackLeftOffset = new NTDouble("Drive/Modules/Back Left Offset (rot)", 0).setPersistent();
    public static final NTEntry<Double> kBackRightOffset = new NTDouble("Drive/Modules/Back Right Offset (rot)", 0).setPersistent();
    public static final SwerveModuleInfo[] kSwerveModuleInfos = {
            new SwerveModuleInfo(IOAllocation.CAN.SWERVE_FL, kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, Constants.kFrontLeftOffset, "Front Left"),
            new SwerveModuleInfo(IOAllocation.CAN.SWERVE_FR, kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, Constants.kFrontRightOffset, "Front Right"),
            new SwerveModuleInfo(IOAllocation.CAN.SWERVE_BL, -kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, Constants.kBackLeftOffset, "Back Left"),
            new SwerveModuleInfo(IOAllocation.CAN.SWERVE_BR, -kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, Constants.kBackRightOffset, "Back Right")
    };

    public static final double kDriveRadius = Math.hypot(kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2);
    public static final double kDriveWheelCOF = 1.2; // TODO: Measure?
    public static final double kDriveMaxAchievableSpeed = Units.feetToMeters(18.9); // m/s  TODO: Measure

    public static final double kDriveDriftComp = kPeriodicTime * 2; // dt for chassis speeds discretize  TODO: Tune

    public static final double kDriveStatorCurrentLimit = 60; // A
    public static final double kDriveSupplyCurrentLimit = 40; // A
    public static final double kDriveCurrentLimitTime = 0.25; // sec

    public static final SwerveKinematicLimits kDriveLimits = new SwerveKinematicLimits();
    static {
        kDriveLimits.kMaxDriveVelocity = kDriveMaxAchievableSpeed;
        kDriveLimits.kMaxDriveAcceleration = kDriveLimits.kMaxDriveVelocity / 0.1;
        kDriveLimits.kMaxSteeringVelocity = Math.toRadians(1500);
    }

    public static final LegacySwerveModuleConstantsFactory kSwerveConstantsFactory = new LegacySwerveModuleConstantsFactory()
            .withDriveMotorGearRatio((50.0/16) * (16.0/28) * (45.0/15))
            .withSteerMotorGearRatio(150.0 / 7)
            .withWheelRadius(1.9) // Inches
            .withSlipCurrent(300) // A  TODO Tune
            .withSteerMotorGains(new Slot0Configs().withKP(100).withKD(0.05))
            .withDriveMotorGains(new Slot0Configs().withKP(3).withKD(0))
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withSpeedAt12VoltsMps(kDriveMaxAchievableSpeed)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(50.0 / 16)
            .withSteerMotorInverted(true);

    // Pathfinding
    public static final String kPathfindingJson = "crescendo_pathfinding.json";
    public static final double kPathfindingTolerance = 0.2; // m

    // Vision
    public static final String kAprilTagJson = "crescendo_apriltag.json";

    // These need to be tuned during field calibration time at every event.
    // They can be adjusted manually in NetworkTables at TagTracker/<Camera>/Config/...
    // Don't forget to set your adjusted values here! The NetworkTables values
    // do not save.
    public static final TagTrackerCaptureProperties kTagTrackerCaptureProps = new TagTrackerCaptureProperties()
            .setAutoExposure(false)
            .setExposure(20)
            .setGain(1)
            .setTargetFps(50);

    public static final RawAprilTagSource.FilterParameters kTagTrackerFilterParams = new RawAprilTagSource.FilterParameters()
            .setAmbiguityThreshold(0.9)
            .setXYStdDevCoefficient(0.01)
            .setThetaStdDevCoefficient(0.01)
            .setFieldBorderMargin(0.5)
            .setZMargin(0.75)
            .setMaxTrustDistance(Double.POSITIVE_INFINITY);

    public static final double[] kVisionStateStdDevs = {0.005, 0.005, 0.001};
    public static final double kVisionHistoryTime = 0.3; // Secs
    // Time at beginning of teleop where vision angle is trusted more
    // Only applies when NOT on competition field!
    public static final double kVisionInitialTrustTime = 5; // Secs
    public static final double kVisionInitialAngleStdDev = 0.2;

    // Lights
    public static final int kLedStripLength = 22;
    public static final int kLowBatteryThreshold = 10; // Volts
    public static final int kLedCurrentShutoffThreshold = 250; // Amps

    // Motor tracking
    public static final double kMotorTrackInterval = 2; // Seconds
    public static final double kOverheatingThreshold = 75; // Celsius

    // This must be at the bottom of the file so it happens last
    static {
        NTEntry.cleanPersistent();
    }
}
