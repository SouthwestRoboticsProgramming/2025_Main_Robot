package com.swrobotics.robot.config;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Describes the IDs of the various devices within the robot.
 */
public final class IOAllocation {
    public static final class CAN {
        private static final String kRIO = "";
        public static final String kGerald = "Gerald";

        // All on Gerald
        public static final String kSwerveBus = kGerald;
        public static final SwerveIDs kSwerveFL = new SwerveIDs(9, 5, 1);
        public static final SwerveIDs kSwerveFR = new SwerveIDs(10, 6, 2);
        public static final SwerveIDs kSwerveBL = new SwerveIDs(11, 7, 3);
        public static final SwerveIDs kSwerveBR = new SwerveIDs(12, 8, 4);
        public static final CanId kJosh = new CanId(13, kGerald);

        public static final CanId kElevatorMotor1 = new CanId(14, kGerald);
        public static final CanId kElevatorMotor2 = new CanId(15, kGerald);

        public static final CanId kAlgaeIntakeSpinMotor = new CanId(16, kGerald);
        public static final CanId kAlgaeIntakePivotMotor = new CanId(17, kGerald);
        public static final CanId kAlgaeIntakePivotEncoder = new CanId(18, kGerald);

        public static final CanId kOuttakePivotMotor = new CanId(1, kRIO);
        public static final CanId kOuttakePivotEncoder = new CanId(2, kRIO);
        public static final CanId kOuttakeMotor = new CanId(3, kRIO);

        public static final CanId kIndexerMotor = new CanId(4, kRIO);

        public static final CanId kPDP = new CanId(62, kRIO);
    }

    public static final class RIO {
        public static final int kPWM_LEDs = 0;

        public static final int kDIO_OuttakeBeamBreak = 0;
    }

    /** IDs of the devices within one swerve module */
    public static final class SwerveIDs {
        public final CanId drive, turn, encoder;

        public SwerveIDs(int drive, int turn, int encoder) {
            this.drive = new CanId(drive, CAN.kSwerveBus);
            this.turn = new CanId(turn, CAN.kSwerveBus);
            this.encoder = new CanId(encoder, CAN.kSwerveBus);
        }
    }

    /** Location of a CAN device. This includes both the ID and the CAN bus */
    public static final class CanId {
        public static int uniqueifyForSim(int id, String bus) {
            // CTRE sim doesn't support CANivore, so ids need to be globally unique
            if (RobotBase.isSimulation() && bus.equals(CAN.kGerald)) {
                // If we somehow have more than 32 devices on the RoboRIO bus
                // we have other problems
                return id + 32;
            }

            return id;
        }

        private final int id;
        private final String bus;

        public CanId(int id, String bus) {
            this.id = id;
            this.bus = bus;
        }

        public int id() {
            return uniqueifyForSim(id, bus);
        }

        public String bus() {
            if (RobotBase.isSimulation())
                return CAN.kRIO;

            return bus;
        }

        public TalonFX createTalonFX() {
            return new TalonFX(id(), bus());
        }

        public CANcoder createCANcoder() {
            return new CANcoder(id(), bus());
        }
    }
}
