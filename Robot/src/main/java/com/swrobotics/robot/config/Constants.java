package com.swrobotics.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.subsystems.swerve.SwerveModuleInfo;
import com.swrobotics.robot.subsystems.vision.limelight.LimelightCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

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
    public static final double kDriveMaxAchievableSpeed = Units.feetToMeters(18.9); // m/s  TODO: Measure

    public static final double kOdometryUpdateFreq = 200; // Hz
    public static final Matrix<N3, N1> kOdometryStdDevs = VecBuilder.fill(0.005, 0.005, 0.001);

    public static final double kDriveStatorCurrentLimit = 60; // A
    public static final double kDriveSupplyCurrentLimit = 40; // A
    public static final double kDriveCurrentLimitTime = 0.25; // sec

    public static final double kDriveWheelCOF = 1.2; // TODO: Measure?

    public static final double kDriveWheelSpacingX = 55.3 / 100; // m
    public static final double kDriveWheelSpacingY = 63.0 / 100; // m
    public static final double kDriveRadius = Math.hypot(kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2);

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

    public static final SwerveDrivetrainConstants kDrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(IOAllocation.CAN.SWERVE_BUS)
            .withPigeon2Id(IOAllocation.CAN.PIGEON2.id())
            .withPigeon2Configs(new Pigeon2Configuration());
    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kModuleConstantsFactory =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio((50.0/16) * (16.0/28) * (45.0/15))
                    .withSteerMotorGearRatio(150.0 / 7)
                    .withCouplingGearRatio(50.0 / 16)
                    .withWheelRadius(Inches.of(1.9))
                    // Gains taken from 254 2024 robot code
                    .withSteerMotorGains(new Slot0Configs().withKP(100).withKD(0.2).withKV(1.5))
                    .withDriveMotorGains(new Slot0Configs().withKP(0.35).withKD(0).withKV(12.0 / 88.2142857143))
                    // TODO: Torque current FOC
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withSlipCurrent(Amps.of(80))
                    .withSpeedAt12Volts(MetersPerSecond.of(kDriveMaxAchievableSpeed))
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                    .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                    .withEncoderInitialConfigs(new CANcoderConfiguration());
    // Simulation constants left at defaults for now

    // Pathfinding
    public static final String kPathfindingJson = "crescendo_pathfinding.json";
    public static final double kPathfindingTolerance = 0.2; // m

    // Vision
    public static final double kVisionMT2SpeedThreshold = 0.2; // m/s

    public static final LimelightCamera.MountingLocation kLimelightLocation = new LimelightCamera.MountingLocation(
            0, 0, 0,
            0, 0, 0
    );

    // This will be different for each lens type, cameras with same lens should
    // have the same config
    public static final LimelightCamera.Config kLimelightConfig = new LimelightCamera.Config(
            // These were tuned at MURA using red alliance speaker AprilTags
            4,
            0.00197,
            0.002,
            0.00117
    );

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
