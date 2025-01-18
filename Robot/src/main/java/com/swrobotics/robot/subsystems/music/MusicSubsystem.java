package com.swrobotics.robot.subsystems.music;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.lib.ctre.CTREUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Fun subsystem to play music using the robot's motors */
public final class MusicSubsystem extends SubsystemBase {
    private static MusicSubsystem instance = null;

    /**
     * @return the instance of the music subsystem
     */
    public static MusicSubsystem getInstance() {
        if (instance == null)
            throw new IllegalStateException("Music subsystem has not been initialized yet!");
        return instance;
    }

    private static final String songsFolder = "music";

    /**
     * @return the list of available songs in the songs folder
     */
    public static List<String> getAvailableSongs() {
        File dir = new File(Filesystem.getDeployDirectory(), songsFolder);

        File[] files = dir.listFiles();
        if (files == null)
            return Collections.emptyList();

        List<String> out = new ArrayList<>();
        for (File file : files) {
            if (file.getName().endsWith(".chrp")) {
                out.add(file.getAbsolutePath());
            }
        }

        return out;
    }

    private final Orchestra orchestra;

    public MusicSubsystem() {
        orchestra = new Orchestra();

        if (instance != null)
            throw new IllegalStateException("MusicSubsystem already initialized");
        instance = this;
    }

    /**
     * Adds a motor to the orchestra. This should be called once in subsystem
     * constructors. This will reconfigure the motor's AudioConfigs.
     *
     * @param fx motor to add
     */
    public void addInstrument(TalonFX fx) {
        AudioConfigs conf = new AudioConfigs();
        conf.BeepOnBoot = true;
        conf.BeepOnConfig = true;
        conf.AllowMusicDurDisable = true;
        CTREUtil.retryUntilOk(fx, () -> fx.getConfigurator().apply(conf));
        
        orchestra.addInstrument(fx);
    }

    /**
     * Starts playing a song.
     *
     * @param file absolute filesystem path to the CHRP file to play
     */
    public void beginSong(String file) {
        orchestra.loadMusic(file);
        orchestra.play();
    }

    /**
     * @return whether a song is currently playing
     */
    public boolean isSongPlaying() {
        return orchestra.isPlaying();
    }

    /**
     * Stops playing any song that is currently playing. If no song is playing
     * this will do nothing.
     */
    public void endSong() {
        orchestra.stop();
    }
}
