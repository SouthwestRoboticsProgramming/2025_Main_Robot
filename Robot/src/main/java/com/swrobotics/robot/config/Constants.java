package com.swrobotics.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.swrobotics.lib.ctre.NTMotionMagicConfigs;
import com.swrobotics.lib.ctre.NTSlot0Configs;
import com.swrobotics.lib.ctre.NTSlot1Configs;
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

    public static final FieldInfo kField = FieldInfo.REEFSCAPE_2025;
    public static final int kEndgameAlertTime = 20;

    // Robot dimensions
    public static final double kFrameLength = Units.inchesToMeters(30); // m
    public static final double kFrameWidth = Units.inchesToMeters(27); // m

    public static final double kBumperThickness = Units.inchesToMeters(3); // FIXME
    public static final double kRobotLength = kFrameLength + kBumperThickness * 2;
    public static final double kRobotWidth = kFrameWidth + kBumperThickness * 2;
    public static final double kRobotRadius = Math.hypot(kRobotLength /2, kRobotWidth /2);
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
    public static final NTEntry<Double> kAutoTurnKp = new NTDouble("Drive/Auto/Turn PID/kP", 9).setPersistent();
    public static final NTEntry<Double> kAutoTurnKd = new NTDouble("Drive/Auto/Turn PID/kD", 0.5).setPersistent();

    public static final NTEntry<Double> kSnapOffset = new NTDouble("Drive/Snap/Offset (m)", 0.025).setPersistent();
    public static final NTEntry<Double> kSnapMaxSpeed = new NTDouble("Drive/Snap/Max Speed (m/s)", 1.5).setPersistent();
    public static final NTEntry<Double> kSnapMaxTurnSpeed = new NTDouble("Drive/Snap/Max Turn Speed (rot/s)", 1.2).setPersistent();
    public static final NTEntry<Double> kSnapDriveKp = new NTDouble("Drive/Snap/Drive kP", 3).setPersistent();
    public static final NTEntry<Double> kSnapDriveKd = new NTDouble("Drive/Snap/Drive kD", 0).setPersistent();
    public static final NTEntry<Double> kSnapTurnKp = new NTDouble("Drive/Snap/Turn kP", 5).setPersistent();
    public static final NTEntry<Double> kSnapTurnKd = new NTDouble("Drive/Snap/Turn kD", 0).setPersistent();

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

    public static final NTEntry<Double> kFrontLeftOffset = new NTDouble("Drive/Modules/Front Left Offset (rot)", -0.335693).setPersistent();
    public static final NTEntry<Double> kFrontRightOffset = new NTDouble("Drive/Modules/Front Right Offset (rot)", 0.314453).setPersistent();
    public static final NTEntry<Double> kBackLeftOffset = new NTDouble("Drive/Modules/Back Left Offset (rot)", -0.318115).setPersistent();
    public static final NTEntry<Double> kBackRightOffset = new NTDouble("Drive/Modules/Back Right Offset (rot)", -0.360107).setPersistent();
    public static final SwerveModuleInfo[] kSwerveModuleInfos = {
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveFL, kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, Constants.kFrontLeftOffset, "Front Left"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveFR, kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, Constants.kFrontRightOffset, "Front Right"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveBL, -kDriveWheelSpacingX / 2, kDriveWheelSpacingY / 2, Constants.kBackLeftOffset, "Back Left"),
            new SwerveModuleInfo(IOAllocation.CAN.kSwerveBR, -kDriveWheelSpacingX / 2, -kDriveWheelSpacingY / 2, Constants.kBackRightOffset, "Back Right")
    };

    public static final SwerveDrivetrainConstants kDrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(IOAllocation.CAN.kSwerveBus)
            .withPigeon2Id(IOAllocation.CAN.kJosh.id())
            .withPigeon2Configs(new Pigeon2Configuration());
    public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kModuleConstantsFactory =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio((50.0/16) * (16.0/28) * (45.0/15))
                    .withSteerMotorGearRatio(150.0 / 7)
                    .withCouplingGearRatio(50.0 / 16)
                    .withWheelRadius(Inches.of(1.9))
                    // Gains taken from 254 2024 robot code
                    .withSteerMotorGains(new Slot0Configs().withKP(50).withKD(0.01).withKV(0.1))
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
    public static final String kPathfindingJson = "reefscape_pathfinding.json";
    public static final double kPathfindingTolerance = 0.2; // m

    // Vision
    public static final double kVisionMT2SpeedThreshold = 0.2; // m/s

    public static final LimelightCamera.MountingLocation kLimelightFrontLeftLocation = new LimelightCamera.MountingLocation(
            kFrameLength / 2 - Units.inchesToMeters(4.5),
            -kFrameWidth / 2 + Units.inchesToMeters(3.25),
            Units.inchesToMeters(9.059),
            // Degrees CCW
            0, 20.6, -33
    );
    public static final LimelightCamera.MountingLocation kLimelightFrontRightLocation = new LimelightCamera.MountingLocation(
            kFrameLength / 2 - Units.inchesToMeters(4.5),
            kFrameWidth / 2 - Units.inchesToMeters(3.25),
            Units.inchesToMeters(9.059),
            // Degrees CCW
            0, 20.6, 33
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

    // Elevator
    public static final double kElevatorMaxHeightRotations = 100;
    public static final NTSlot0Configs kElevatorPID =
            new NTSlot0Configs("Superstructure/Elevator/PID", 0.3, 0, 0.286, 0, 0.12, 0);
    public static final NTMotionMagicConfigs kElevatorMotionMagic =
            new NTMotionMagicConfigs("Superstructure/Elevator/Motion Magic", 80, 500, 0);
    public static final NTEntry<Double> kElevatorClimbKg = new NTDouble("Superstructure/Elevator/Climb kG", 0).setPersistent();

    public static final NTEntry<Double> kElevatorTolerance = new NTDouble("Superstructure/Elevator/Tolerance", 0.01).setPersistent();
    public static final NTEntry<Double> kElevatorCollisionTolerance = new NTDouble("Superstructure/Elevator/Collision Tolerance", 0.01).setPersistent();
    public static final NTEntry<Double> kElevatorMaxHeightWithArmInBelowBar = new NTDouble("Superstructure/Elevator/Max Height With Arm In Below Bar", 0.047993).setPersistent();
    public static final NTEntry<Double> kElevatorHeightBottom = new NTDouble("Superstructure/Elevator/Bottom Height", 0.002).setPersistent();
    public static final NTEntry<Double> kElevatorHeightL1 = new NTDouble("Superstructure/Elevator/L1 Height", 0.25).setPersistent();
    public static final NTEntry<Double> kElevatorHeightL2 = new NTDouble("Superstructure/Elevator/L2 Height", 0.375).setPersistent();
    public static final NTEntry<Double> kElevatorHeightL3 = new NTDouble("Superstructure/Elevator/L3 Height", 0.55).setPersistent();
    public static final NTEntry<Double> kElevatorHeightL4 = new NTDouble("Superstructure/Elevator/L4 Height", 1).setPersistent();
    public static final NTEntry<Double> kElevatorHeightClimbPrep = new NTDouble("Superstructure/Elevator/Climb Prep Height", 0.75).setPersistent();
    public static final NTEntry<Double> kElevatorHeightClimbEnd = new NTDouble("Superstructure/Elevator/Climb End Height", 0.5).setPersistent();

    // Coral outtake pivot
    // 60:24 CANcoder
    public static final double kOuttakePivotMotorToArmRatio = (60.0 / 27.0) * (58.0 / 18.0) * (60.0 / 8.0);
    public static final double kOuttakePivotCANcoderToArmRatio = 60.0 / 27.0;
    public static final NTEntry<Double> kOuttakePivotEncoderOffset = new NTDouble("Superstructure/Pivot/Encoder/Offset (rot)", 0).setPersistent();
    public static final NTSlot0Configs kOuttakePivotPID =
            new NTSlot0Configs("Superstructure/Pivot/PID", 200, 0, 0.316, 0, 7.021702, 0);
    public static final NTSlot1Configs kOuttakePivotPIDWithCoral =
            new NTSlot1Configs("Superstructure/Pivot/PID With Coral", 200, 0, 0.424, 0.1, 7.356069, 0);
    public static final NTMotionMagicConfigs kOuttakePivotMotionMagic =
            new NTMotionMagicConfigs("Superstructure/Pivot/Motion Magic", 1.2, 0.5, 0);

    public static final NTEntry<Double> kOuttakePivotTolerance = new NTDouble("Superstructure/Pivot/Tolerance (deg)", 3).setPersistent();
    public static final NTEntry<Double> kOuttakePivotCollisionTolerance = new NTDouble("Superstructure/Pivot/Collision Tolerance (deg)", 5).setPersistent();
    public static final NTEntry<Double> kOuttakePivotMaxAngleNearBar = new NTDouble("Superstructure/Pivot/Max Angle Near Bar", 68.203080).setPersistent();
    public static final NTEntry<Double> kOuttakePivotInAngle = new NTDouble("Superstructure/Pivot/In Angle (deg)", 90).setPersistent();
    public static final NTEntry<Double> kOuttakePivotScoreL1Angle = new NTDouble("Superstructure/Pivot/Score L1 Angle (deg)", 70).setPersistent();
    public static final NTEntry<Double> kOuttakePivotScoreL2Angle = new NTDouble("Superstructure/Pivot/Score L2 Angle (deg)", 75).setPersistent();
    public static final NTEntry<Double> kOuttakePivotScoreL3Angle = new NTDouble("Superstructure/Pivot/Score L3 Angle (deg)", 75).setPersistent();
    public static final NTEntry<Double> kOuttakePivotScoreL4Angle = new NTDouble("Superstructure/Pivot/Score L4 Angle (deg)", 60).setPersistent();
    public static final NTEntry<Double> kOuttakePivotClimbAngle = new NTDouble("Superstructure/Pivot/Climb Angle (deg)", 90).setPersistent();

    // Algae floor intake
    public static final double kAlgaePivotMotorToArmRatio = (36.0 / 16.0) * (56.0 / 24.0) * 25;
    public static final double kAlgaePivotCANcoderToArmRatio = (36.0 / 16.0);
    public static final NTEntry<Double> kAlgaeStowAngle = new NTDouble("Algae/Pivot/Stow Angle (deg)", 85).setPersistent();
    public static final NTEntry<Double> kAlgaeIntakeAngle = new NTDouble("Algae/Pivot/Intake Angle (deg)", 35.0).setPersistent();
    public static final NTEntry<Double> kAlgaeIntakeVoltage = new NTDouble("Algae/Roller/Intake Voltage", 8).setPersistent();
    public static final NTEntry<Double> kAlgaeOuttakeVoltage = new NTDouble("Algae/Roller/Outtake Voltage", 8).setPersistent();
    public static final NTEntry<Double> kAlgaePivotEncoderOffset = new NTDouble("Algae/Pivot/Offset (rot)", -0.244141).setPersistent();
    public static final NTEntry<Double> kAlgaeIntakeCurrentLimit = new NTDouble("Algae/Roller/Current Limit (A)", 24).setPersistent();
    public static final NTSlot0Configs kAlgaePivotPID =
            new NTSlot0Configs("Algae/Pivot/PID", 50, 5, 0.122, 0.1, 17, 0.2);
    public static final NTMotionMagicConfigs kAlgaePivotMotionMagic =
            new NTMotionMagicConfigs("Algae/Pivot/Motion Magic", 20, 4, 0);
    
    // Coral outtake
    public static final NTEntry<Double> kOuttakeRollerIntakeVoltage = new NTDouble("Outtake/Intake Voltage", 2).setPersistent();
    public static final NTEntry<Double> kOuttakeRollerScoreVoltage = new NTDouble("Outtake/Score Voltage", 3).setPersistent();

    // Indexer
    public static final NTEntry<Double> kIndexerIntakeVoltage = new NTDouble("Indexer/Intake Voltage", 5).setPersistent();

    // Lights
    public static final int kLedStripLength = 22;
    public static final int kLowBatteryThreshold = 10; // Volts

    // Motor tracking
    public static final double kOverheatingThreshold = 75; // Celsius

    // This must be at the bottom of the file so it happens last
    static {
        NTEntry.cleanPersistent();
    }
}
