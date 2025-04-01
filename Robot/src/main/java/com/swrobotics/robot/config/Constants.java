package com.swrobotics.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.*;

// Use NTEntry when you want tunable
// Use double when value has been tuned in so it can't accidentally change
public final class Constants {
    public static final int kPeriodicFreq = 50; // Hz
    public static final double kPeriodicTime = 1.0 / kPeriodicFreq;

    public static final FieldInfo kField = FieldInfo.REEFSCAPE_2025;
    public static final int kEndgameAlertTime = 20;
    public static final int kEndgameAlert2Time = 5;

    // Robot dimensions
    public static final double kFrameLength = Units.inchesToMeters(30); // m
    public static final double kFrameWidth = Units.inchesToMeters(27); // m

    public static final double kBumperThickness = Units.inchesToMeters(3); // FIXME
    public static final double kRobotLength = kFrameLength + kBumperThickness * 2;
    public static final double kRobotWidth = kFrameWidth + kBumperThickness * 2;
    public static final double kRobotRadius = Math.hypot(kRobotLength / 2, kRobotWidth / 2);
    public static final double kRobotMass = Units.lbsToKilograms(135);
    // Approximation of robot as uniform cuboid
    // See https://sleipnirgroup.github.io/Choreo/usage/estimating-moi/
    // FIXME: Measure in CAD
    public static final double kRobotMOI = 1.0/12.0 * kRobotMass * (kFrameLength*kFrameLength + kFrameWidth*kFrameWidth);
    public static final double kCOGHeightWithElevatorDown = Units.inchesToMeters(10); // TODO: Measure
    public static final double kCOGHeightWithElevatorUp = Units.inchesToMeters(23.126);

    // Controls
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDeadband = 0.15;
    public static final double kTriggerThreshold = 0.3;

    public static final double kDriveControlMaxAccel = 3.5; // m/s^2
    public static final double kDriveControlMaxTurnSpeed = 1; // rot/s
    public static final double kDriveControlDrivePower = 2; // Exponent input is raised to
    public static final double kDriveControlTurnPower = 2;

    // Auto (TODO: Tune)
    public static final double kAutoDriveKp = 4;
    public static final double kAutoDriveKd = 0;
    public static final NTEntry<Double> kAutoTurnKp = new NTDouble("Drive/Auto/Turn PID/kP", 5).setPersistent();
    public static final NTEntry<Double> kAutoTurnKd = new NTDouble("Drive/Auto/Turn PID/kD", 0).setPersistent();

    public static final double kAutoMaxDriveSpeed = 4;//Units.feetToMeters(18);
    public static final double kAutoMaxDriveAccel = 3;//5;
    public static final double kAutoMaxTurnSpeed = 1.25;
    public static final double kAutoMaxTurnAccel = 2;

    public static final double kAutoSwitchToSnapDist = 0.2;
    public static final NTEntry<Double> kAutoScoreXYTolerance = new NTDouble("Drive/Auto/Score XY Tolerance (m)", 0.05).setPersistent();
    public static final NTEntry<Double> kAutoScoreAngleTolerance = new NTDouble("Drive/Auto/Score Angle Tolerance (deg)", 2).setPersistent();
    public static final double kAutoToleranceTimeout = 0.8;
    public static final double kAutoCoralEjectTime = 0.3;
    public static final double kAutoElevatorDownDelay = 0.5;

    public static final NTEntry<Double> kSnapOffset = new NTDouble("Drive/Snap/Offset (m)", 0.0).setPersistent();
    public static final NTEntry<Double> kSnapMaxSpeed = new NTDouble("Drive/Snap/Max Speed (meters per sec)", 1.5).setPersistent();
    public static final NTEntry<Double> kSnapMaxTurnSpeed = new NTDouble("Drive/Snap/Max Turn Speed (rot per sec)", 1.2).setPersistent();
    public static final NTEntry<Double> kSnapDriveKp = new NTDouble("Drive/Snap/Drive kP", 3).setPersistent();
    public static final NTEntry<Double> kSnapDriveKd = new NTDouble("Drive/Snap/Drive kD", 0).setPersistent();
    public static final NTEntry<Double> kSnapTurnKp = new NTDouble("Drive/Snap/Turn kP", 5).setPersistent();
    public static final NTEntry<Double> kSnapTurnKd = new NTDouble("Drive/Snap/Turn kD", 0).setPersistent();
    public static final NTEntry<Double> kSnapXYDeadzone = new NTDouble("Drive/Snap/XY Deadzone (m)", 0.005).setPersistent();
    public static final NTEntry<Double> kSnapThetaDeadzone = new NTDouble("Drive/Snap/Theta Deadzone (deg)", 0.2).setPersistent();

    // Drive
    public static final double kDriveMaxAchievableSpeed = Units.feetToMeters(18.9); // m/s  TODO: Measure

    // Subtracted from calculated max acceleration to get tipping acceleration limit
    public static final double kDriveTippingAccelTolerance = 1; // m/s^2

    public static final double kOdometryUpdateFreq = 200; // Hz
    public static final Matrix<N3, N1> kOdometryStdDevs = VecBuilder.fill(0.005, 0.005, 0.001);

    public static final double kDriveStatorCurrentLimit = 60; // A
    public static final double kDriveSupplyCurrentLimit = 40; // A
    public static final double kDriveCurrentLimitTime = 0.25; // sec

    public static final double kDriveWheelCOF = 1.2; // TODO: Measure?

    public static final double kDriveWheelSpacingX = 63.0 / 100; // m
    public static final double kDriveWheelSpacingY = 55.3 / 100; // m
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
                    .withWheelRadius(Meters.of(0.04905134))//0.048218 ))
                    .withSteerMotorGains(new Slot0Configs().withKP(50).withKD(0.01).withKV(0.1))
                    .withDriveMotorGains(new Slot0Configs().withKP(0.35).withKD(0).withKV(0.012621).withKS(0.22109))
//                    .withDriveMotorGains(new Slot0Configs().withKP(0.4).withKD(0).withKV(0.012621 * 17.675293 / 13.515625 * 17.675293 / 2.136719 * 19.378906 / 17.676758 * 17.675293 / 21.419922).withKS(0.22109))
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withSlipCurrent(Amps.of(80))
                    .withSpeedAt12Volts(MetersPerSecond.of(kDriveMaxAchievableSpeed))
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                    .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                    .withEncoderInitialConfigs(new CANcoderConfiguration());
    static {
        if (RobotBase.isSimulation()) {
            kModuleConstantsFactory.DriveMotorGains
                    .withKV(0.12612)
                    .withKS(0.22510);
        }
    }

    public static final RobotConfig kPathPlannerRobotConfig = new RobotConfig(
            Constants.kRobotMass,
            Constants.kRobotMOI,
            new ModuleConfig(
                    Constants.kModuleConstantsFactory.WheelRadius,
                    Constants.kDriveMaxAchievableSpeed,
                    Constants.kDriveWheelCOF,
                    DCMotor.getKrakenX60Foc(1).withReduction(Constants.kModuleConstantsFactory.DriveMotorGearRatio),
                    Constants.kDriveStatorCurrentLimit,
                    1
            ),
            kSwerveModuleInfos[0].position(),
            kSwerveModuleInfos[1].position(),
            kSwerveModuleInfos[2].position(),
            kSwerveModuleInfos[3].position()
    );

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
    public static final LimelightCamera.MountingLocation kLimelightBackLocation = new LimelightCamera.MountingLocation(
            // TODO: These are guesses, they should be measured in CAD
            0, 0, 0.972,
            0, 35, 180
    );

    // This will be different for each lens type, cameras with same lens should
    // have the same config
    public static final LimelightCamera.Config kLimelightConfig = new LimelightCamera.Config(
            // These were tuned at MURA using red alliance speaker AprilTags
            2,
            0.00197,
            0.002,
            0.00117
    );

    // Elevator
    public static final double kElevatorMaxHeightRotations = 25;
    
    public static final NTSlot0Configs kElevatorPID =
            new NTSlot0Configs("Superstructure/Elevator/PID", 0.3, 0, 0.286, 0, 0.12, 0);
    public static final NTEntry<Double> kElevatorMaxVelocity = new NTDouble("Superstructure/Elevator/Max Velocity", 80).setPersistent();
    public static final NTEntry<Double> kElevatorMaxAccel = new NTDouble("Superstructure/Elevator/Max Acceleration", 500).setPersistent();
    public static final NTEntry<Double> kElevatorClimbHoldVolts = new NTDouble("Superstructure/Elevator/Climb Hold Voltage", 1).setPersistent();
    public static final NTEntry<Double> kElevatorClimbPullVolts = new NTDouble("Superstructure/Elevator/Climb Pull Voltage", 1).setPersistent();

    public static final NTEntry<Double> kElevatorTolerance = new NTDouble("Superstructure/Elevator/Tolerance", 0.04).setPersistent();
    public static final NTEntry<Double> kElevatorCollisionTolerance = new NTDouble("Superstructure/Elevator/Collision Tolerance", 0.005).setPersistent();
    public static final NTEntry<Double> kElevatorDeviationTolerance = new NTDouble("Superstructure/Elevator/Deviation Tolerance", 0.03).setPersistent();
    public static final NTEntry<Double> kElevatorFrameCollisionHeight = new NTDouble("Superstructure/Elevator/Frame Collision Height", 0.060566).setPersistent();
    public static final NTEntry<Double> kElevatorStage2CollisionHeight = new NTDouble("Superstructure/Elevator/Stage 2 Collision Height", 0.324990).setPersistent();
    public static final NTEntry<Double> kElevatorNeutralThreshold = new NTDouble("Superstructure/Elevator/Neutral Threshold", 0.08).setPersistent();
    public static final NTEntry<Double> kElevatorIndexerHeight = new NTDouble("Superstructure/Elevator/Indexer Height", 0.05).setPersistent();
    public static final NTEntry<Double> kElevatorHeightL1 = new NTDouble("Superstructure/Elevator/L1 Height", 0.01).setPersistent();
    public static final NTEntry<Double> kElevatorHeightL2 = new NTDouble("Superstructure/Elevator/L2 Height", 0.27).setPersistent();
    public static final NTEntry<Double> kElevatorHeightL3 = new NTDouble("Superstructure/Elevator/L3 Height", 0.535).setPersistent();
    public static final NTEntry<Double> kElevatorHeightL4 = new NTDouble("Superstructure/Elevator/L4 Height", 1).setPersistent();
    public static final NTEntry<Double> kElevatorHeightClimbPrep = new NTDouble("Superstructure/Elevator/Climb Prep Height", 0.75).setPersistent();
    public static final NTEntry<Double> kElevatorHeightClimbEnd = new NTDouble("Superstructure/Elevator/Climb End Height", 0.5).setPersistent();
    public static final NTEntry<Double> kElevatorHeightLowAlgae = new NTDouble("Superstructure/Elevator/Low Algae Height", 0.5).setPersistent();
    public static final NTEntry<Double> kElevatorHeightHighAlgae = new NTDouble("Superstructure/Elevator/High Algae Height", 0.75).setPersistent();
    public static final NTEntry<Double> kElevatorHeightNet = new NTDouble("Superstructure/Elevator/Net Height", 1).setPersistent();

    // Coral outtake pivot
    // 60:24 CANcoder
    public static final double kOuttakePivotMotorToArmRatio = (60.0 / 26.0) * (58.0 / 18.0) * (60.0 / 8.0);
    public static final double kOuttakePivotCANcoderToArmRatio = 48.0 / 36.0;
    public static final NTEntry<Double> kOuttakePivotEncoderOffset = new NTDouble("Superstructure/Pivot/Encoder/Offset (rot)", 0.415609).setPersistent();
    public static final NTSlot0Configs kOuttakePivotPID =
            new NTSlot0Configs("Superstructure/Pivot/PID", 200, 0, 0.316, 0, 7.021702, 0);
    public static final NTSlot1Configs kOuttakePivotPIDWithCoral =
            new NTSlot1Configs("Superstructure/Pivot/PID With Coral", 200, 0, 0.424, 0.1, 7.356069, 0);
    public static final NTEntry<Double> kOuttakePivotMaxVelocity = new NTDouble("Superstructure/Pivot/Max Velocity", 1.2).setPersistent();
    public static final NTEntry<Double> kOuttakePivotMaxAccel = new NTDouble("Superstructure/Pivot/Max Acceleration", 0.5).setPersistent();

    public static final NTEntry<Double> kOuttakePivotTolerance = new NTDouble("Superstructure/Pivot/Tolerance (deg)", 4).setPersistent();
    public static final NTEntry<Double> kOuttakePivotCollisionTolerance = new NTDouble("Superstructure/Pivot/Collision Tolerance (deg)", 5).setPersistent();
    public static final NTEntry<Double> kOuttakePivotDeviationTolerance = new NTDouble("Superstructure/Pivot/Deviation Tolerance (deg)", 4).setPersistent();
    public static final NTEntry<Double> kOuttakePivotFrameCollisionAngle = new NTDouble("Superstructure/Pivot/Frame Collision Angle (deg)", 80.683560).setPersistent();
    public static final NTEntry<Double> kOuttakePivotStage2CollisionAngle = new NTDouble("Superstructure/Pivot/Stage 2 Collision Angle (deg)", 67.060440).setPersistent();
    public static final NTEntry<Double> kOuttakePivotInAngle = new NTDouble("Superstructure/Pivot/In Angle (deg)", 88).setPersistent();
    public static final NTEntry<Double> kOuttakePivotScoreL1Angle = new NTDouble("Superstructure/Pivot/Score L1 Angle (deg)", 85).setPersistent();
    public static final NTEntry<Double> kOuttakePivotScoreL2Angle = new NTDouble("Superstructure/Pivot/Score L2 Angle (deg)", 75).setPersistent();
    public static final NTEntry<Double> kOuttakePivotScoreL3Angle = new NTDouble("Superstructure/Pivot/Score L3 Angle (deg)", 75).setPersistent();
    public static final NTEntry<Double> kOuttakePivotScoreL4Angle = new NTDouble("Superstructure/Pivot/Score L4 Angle (deg)", 60).setPersistent();
    public static final NTEntry<Double> kOuttakePivotClimbAngle = new NTDouble("Superstructure/Pivot/Climb Angle (deg)", 90).setPersistent();
    public static final NTEntry<Double> kOuttakePivotLowAlgaeAngle = new NTDouble("Superstructure/Pivot/Low Algae Angle (deg)", 75).setPersistent();
    public static final NTEntry<Double> kOuttakePivotHighAlgaeAngle = new NTDouble("Superstructure/Pivot/High Algae Angle (deg)", 75).setPersistent();
    public static final NTEntry<Double> kOuttakePivotNetAngle = new NTDouble("Superstructure/Pivot/Net Angle (deg)", 75).setPersistent();

    // Algae floor intake
    public static final double kAlgaePivotMotorToArmRatio = (36.0 / 16.0) * (56.0 / 24.0) * 20;
    public static final double kAlgaePivotCANcoderToArmRatio = (36.0 / 16.0);
    public static final NTEntry<Double> kAlgaeStowAngle = new NTDouble("Algae/Pivot/Stow Angle (deg)", 85).setPersistent();
    public static final NTEntry<Double> kAlgaeIntakeAngle = new NTDouble("Algae/Pivot/Intake Angle (deg)", 35.0).setPersistent();
    public static final NTEntry<Double> kAlgaeIntakeHoldAngle = new NTDouble("Algae/Hold Angle (deg)", 60).setPersistent();
    public static final NTEntry<Double> kAlgaeIntakeVoltage = new NTDouble("Algae/Roller/Intake Voltage", 8).setPersistent();
    public static final NTEntry<Double> kAlgaeOuttakeVoltage = new NTDouble("Algae/Roller/Outtake Voltage", 8).setPersistent();
    public static final NTEntry<Double> kAlgaeDetectCurrentThreshold = new NTDouble("Algae/Roller/Detect Current Threshold (A)", 15).setPersistent();
    public static final NTEntry<Double> kAlgaeDetectDebounce = new NTDouble("Algae/Roller/Detect Debounce (s)", 0.4).setPersistent();
    public static final NTEntry<Double> kAlgaePivotEncoderOffset = new NTDouble("Algae/Pivot/Offset (rot)", -0.244141).setPersistent();
    public static final NTEntry<Double> kAlgaeIntakeCurrentLimit = new NTDouble("Algae/Roller/Current Limit (A)", 24).setPersistent();
    public static final NTSlot0Configs kAlgaePivotPID =
            new NTSlot0Configs("Algae/Pivot/PID", 50, 5, 0.122, 0.1, 17, 0.2);
    public static final NTMotionMagicConfigs kAlgaePivotMotionMagic =
            new NTMotionMagicConfigs("Algae/Pivot/Motion Magic", 20, 4, 0);

    // Coral outtake
    public static final int kOuttakeRefreshFreq = 100; // Hz
    public static final NTEntry<Double> kOuttakeRollerIntakeCoralVoltage = new NTDouble("Outtake/Intake Coral Voltage", 2).setPersistent();
    public static final NTEntry<Double> kOuttakeRollerScoreCoralVoltage = new NTDouble("Outtake/Score Coral Voltage", 3).setPersistent();
    public static final NTEntry<Double> kOuttakeRollerScoreCoralL4Voltage = new NTDouble("Outtake/Score Coral L4 Voltage", 3).setPersistent();
    public static final NTEntry<Double> kOuttakeRollerIntakeAlgaeVoltage = new NTDouble("Outtake/Intake Algae Voltage", 1).setPersistent();
    public static final NTEntry<Double> kOuttakeRollerScoreAlgaeVoltage = new NTDouble("Outtake/Score Algae Voltage", 1).setPersistent();
    public static final NTEntry<Double> kOuttakeRollerHoldAlgaeVoltage = new NTDouble("Outtake/Hold Algae Voltage", 0).setPersistent();
    public static final NTEntry<Double> kOuttakeHoldPositionOffset = new NTDouble("Outtake/Hold Position Offset", 0.3).setPersistent();
    public static final NTSlot0Configs kOuttakeRollerPID =
            new NTSlot0Configs("Outtake/PID", 5, 0, 0, 0, 0, 0);

    // Indexer
    public static final NTEntry<Double> kIndexerIntakeVoltage = new NTDouble("Indexer/Intake Voltage", 5).setPersistent();

    // Lights
    public static final int kLedStripLength = 51;
    public static final int kLowBatteryThreshold = 10; // Volts

    // Motor tracking
    public static final double kOverheatingThreshold = 75; // Celsius

    // Adjusts
    public static final NTEntry<Double> kPivotAdjustMax = new NTDouble("Superstructure/Pivot/Adjust Max (deg)", 5.0).setPersistent(); // Degrees
    public static final NTEntry<Double> kElevatorAdjustMax = new NTDouble("Superstructure/Elevator/Adjust Max", 0.05).setPersistent(); // Degrees

    // This must be at the bottom of the file so it happens last
    static {
        NTEntry.cleanPersistent();
    }
}
