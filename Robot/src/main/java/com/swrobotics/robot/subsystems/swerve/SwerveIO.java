package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveIO {
    final class Inputs implements LoggableInputs {
        public Pose2d estPose;
        public ChassisSpeeds robotRelSpeeds;
        public SwerveModuleState[] moduleStates;
        public SwerveModuleState[] moduleTargets;
        public SwerveModulePosition[] modulePositions;
        public double odometryPeriod;
        public int successfulDaqs;
        public int failedDaqs;

        public Rotation2d rawGyroRotation;

        @Override
        public void toLog(LogTable table) {
            table.put("pose", estPose);
            table.put("robotRelSpeeds", robotRelSpeeds);
            table.put("moduleStates", moduleStates);
            table.put("moduleTargets", moduleTargets);
            table.put("modulePositions", modulePositions);
            table.put("odometryPeriod", odometryPeriod);
            table.put("successfulDaqs", successfulDaqs);
            table.put("failedDaqs", failedDaqs);
            table.put("rawGyroRotation", rawGyroRotation);
        }

        @Override
        public void fromLog(LogTable table) {
            estPose = table.get("pose", estPose);
            robotRelSpeeds = table.get("robotRelSpeeds", robotRelSpeeds);
            moduleStates = table.get("moduleStates", moduleStates);
            moduleTargets = table.get("moduleTargets", moduleTargets);
            modulePositions = table.get("modulePositions", modulePositions);
            odometryPeriod = table.get("odometryPeriod", odometryPeriod);
            successfulDaqs = table.get("successfulDaqs", successfulDaqs);
            failedDaqs = table.get("failedDaqs", failedDaqs);
            rawGyroRotation = table.get("rawGyroRotation", rawGyroRotation);
        }
    }

    void updateInputs(Inputs inputs);

    void setControl(SwerveRequest request);

    void resetPose(Pose2d pose);

    void resetRotation(Rotation2d rotation);

    void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);

    void calibrateModuleOffsets();
}
