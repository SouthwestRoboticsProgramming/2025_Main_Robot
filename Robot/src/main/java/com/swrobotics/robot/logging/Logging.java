package com.swrobotics.robot.logging;

import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Helpers for AdvantageKit logging
 */
public final class Logging {
    /** The logging mode for simulation */
    public enum SimMode {
        /** Normal simulation, for testing */
        SIMULATE,
        /** Normal simulation, and write out a log file */
        SIMULATE_AND_LOG,
        /** Replay a log file. Useful for debugging. */
        REPLAY
    }

    public static void initialize(SimMode simMode) {
        if (RobotBase.isReal()) {
            // TODO: Figure out writing to USB
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else if (simMode == SimMode.SIMULATE) {
            Logger.addDataReceiver(new NT4Publisher());
        } else if (simMode == SimMode.SIMULATE_AND_LOG) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter());
            // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        } else {
            // FIXME: findReplayLog() does not work
            String path = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(path));
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();
    }
}
