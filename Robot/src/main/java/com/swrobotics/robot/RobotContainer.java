package com.swrobotics.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.pathfinding.pathplanner.AutoBuilderExt;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.FieldPositions;
import com.swrobotics.robot.config.PathEnvironments;
import com.swrobotics.robot.logging.RobotView;
import com.swrobotics.robot.subsystems.superstructure.SuperstructureSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDriveSubsystem;
import com.swrobotics.robot.subsystems.PathfindingTest;
import com.swrobotics.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.swrobotics.robot.commands.PlaySongCommand;
import com.swrobotics.robot.control.ControlBoard;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.logging.Logging;
import com.swrobotics.robot.subsystems.lights.LightsSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * The container for all of the robot's subsystems. This is separate from
 * {@link Robot} so that we can use a constructor for initialization instead of
 * {@code robotInit()}, which allows us to have final fields for subsystems.
 */
public class RobotContainer {
    // Whether to simulate the robot or replay a log file
    public static final Logging.SimMode SIM_MODE = Logging.SimMode.SIMULATE;
//    public static final Logging.SimMode SIM_MODE = Logging.SimMode.SIMULATE_AND_LOG;
//    public static final Logging.SimMode SIM_MODE = Logging.SimMode.REPLAY;

    // Create dashboard choosers
    private final LoggedDashboardChooser<Command> autoSelector;
    private final LoggedDashboardChooser<Double> autoDelaySelector;

    public final MotorTrackerSubsystem motorTracker;
    public final SwerveDriveSubsystem drive;
    public final VisionSubsystem vision;
    public final SuperstructureSubsystem superstructure;

    public final LightsSubsystem lights;
    public final MusicSubsystem music;

    public final ControlBoard controlboard;

    private Command musicCommand;

    public RobotContainer() {
        // Turn off joystick warnings in sim
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

        // These must be initialized first
        music = new MusicSubsystem();
        motorTracker = new MotorTrackerSubsystem();

        drive = new SwerveDriveSubsystem();
        vision = new VisionSubsystem(drive);
        superstructure = new SuperstructureSubsystem();

        lights = new LightsSubsystem(this);

        // ControlBoard must be initialized last
        controlboard = new ControlBoard(this);

        // Register Named Commands for Auto
        NamedCommands.registerCommand("Example Named Command", Commands.print("The command ran!"));

        // Create a chooser to select the autonomous
        List<AutoEntry> autos = buildPathPlannerAutos();
        autos.sort(Comparator.comparing(AutoEntry::name, String.CASE_INSENSITIVE_ORDER));
        autoSelector = new LoggedDashboardChooser<>("Auto Selector");
        autoSelector.addDefaultOption("None", Commands.none());
        autoSelector.addOption("Test", testAutoCommand());
        for (AutoEntry auto : autos)
            autoSelector.addOption(auto.name(), auto.cmd());

        // Create a selector to select delay before running auto
        autoDelaySelector = new LoggedDashboardChooser<>("Auto Delay");
        autoDelaySelector.addDefaultOption("None", 0.0);
        for (int i = 0; i < 10; i++) {
            double time = i / 2.0 + 0.5;
            autoDelaySelector.addOption(time + " seconds", time);
        }

        FieldView.publish();
        RobotView.publish();

        // Play startup song
        CommandScheduler.getInstance().schedule(musicCommand = Commands.waitSeconds(5)
                .andThen(new PlaySongCommand(music, "music" + File.separator + "xp.chrp")));
    }

    private record AutoEntry(String name, Command cmd) {}

    private Command testAutoCommand() {
        Pose2d coralStation = new Pose2d(
                new Translation2d(1.775/2, 1.289/2),
                FieldPositions.getBlueCoralStationAngle(FieldPositions.CoralStation.RIGHT)
        );
        Pose2d coralStation2 = new Pose2d(
                new Translation2d(1.775/2, 8.052 - 1.289/2),
                FieldPositions.getBlueCoralStationAngle(FieldPositions.CoralStation.LEFT)
        );

        PathConstraints constraints = new PathConstraints(
                Constants.kDriveMaxAchievableSpeed,
                Constants.kDriveControlMaxAccel,
                Units.rotationsToRadians(Constants.kDriveControlMaxTurnSpeed),
                Units.rotationsToRadians(Constants.kDriveControlMaxTurnSpeed / 0.3));

        List<Pose2d> remainingScoringPositions = new ArrayList<>();

        List<Command> sequence = new ArrayList<>();
        sequence.add(Commands.runOnce(() -> {
            for (int i = 0; i < 12; i++) {
                remainingScoringPositions.add(Constants.kField.flipPoseForAlliance(
                        FieldPositions.getBlueReefScoringTarget(i)));
            }
        }));
        for (int i = 0; i < 12; i++) {
            sequence.add(AutoBuilderExt.pathfindToClosestPoseFlipped(
                    PathEnvironments.kFieldWithAutoGamePieces,
                    List.of(coralStation, coralStation2),
                    constraints,
                    null
            ));
            sequence.add(Commands.defer(
                    () -> AutoBuilderExt.pathfindToClosestPose(
                        PathEnvironments.kFieldWithAutoGamePieces,
                        remainingScoringPositions,
                        constraints,
                        remainingScoringPositions::remove
                    ), Collections.singleton(drive)));
        }

        return Commands.sequence(sequence.toArray(new Command[0]));
    }

    private static List<AutoEntry> buildPathPlannerAutos() {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        List<String> autoNames = AutoBuilder.getAllAutoNames();
        autoNames.sort(String.CASE_INSENSITIVE_ORDER);

        List<PathPlannerAuto> options = new ArrayList<>();
        for (String autoName : autoNames) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);

            options.add(auto);
        }

        List<AutoEntry> entries = new ArrayList<>();
        for (PathPlannerAuto auto : options)
            entries.add(new AutoEntry(auto.getName(), auto));

        return entries;
    }

    public void disabledInit() {
        lights.disabledInit();

        if (DriverStation.isEStopped()) {
            if (musicCommand != null)
                musicCommand.cancel();

            // Play abort sound
            CommandScheduler.getInstance().schedule(musicCommand = new PlaySongCommand(
                    music,
                    "music" + File.separator + "abort.chrp"));
        }
    }

    public void disabledExit() {
        if (musicCommand != null)
            musicCommand.cancel();
    }

    public double getAutoDelay() {
        return autoDelaySelector.get();
    }

    public Command getAutonomousCommand() {
        return autoSelector.get();
    }
}
