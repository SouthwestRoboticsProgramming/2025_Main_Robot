package com.swrobotics.lib.pathfinding;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

public final class PathfindingJNI {
    // Name of the Rust crate containing the pathfinder
    private static final String LIBRARY_NAME = "pathfinding_jni";

    static {
        RuntimeType type = RuntimeType.getCurrent();
        Path p = type.getLibraryPath();
        System.out.println("Pathfinding JNI path is " + p.toAbsolutePath());

        try {
            System.load(p.toAbsolutePath().toString());
        } catch (UnsatisfiedLinkError e) {
            if (type == RuntimeType.SIMULATION_WINDOWS || type == RuntimeType.SIMULATION_LINUX) {
                // Give a little reminder
                System.err.println("Failed to load Pathfinding JNI library. Make sure the library is built using "
                    + "'cargo build --release'.");
            }
            throw e;
        }
    }

    // These functions are implemented in src/main/rust/lib.rs
    // Do not use these unless you know what you're doing!

    public static native long newObstacleList(); // Returns obstacle list handle
    public static native void addCircle(long obsHandle, double posX, double posY, double radius);
    public static native void addPolygon(long obsHandle, double[] vertices);

    // Takes ownership of the obstacle list - don't use it afterward!
    public static native long buildEnvironment(long obsHandle, double avoidanceRadius); // Returns environment handle

    // Returns null if path not found
    public static native double[] findPath(long envHandle, double startX, double startY, double goalX, double goalY);

    public static native double[] getDebugData(long envHandle);
    public static native double[] debugFindSafe(long envHandle, double startX, double startY);

    // Not perfect platform compatibility, but good enough for our uses
    private enum RuntimeType {
        ROBORIO {
            @Override
            Path getLibraryPath() {
                return Paths.get("/home/lvuser/lib" + LIBRARY_NAME + ".so");
            }
        },
        SIMULATION_WINDOWS {
            @Override
            Path getLibraryPath() {
                return Paths.get("target\\release\\" + LIBRARY_NAME + ".dll");
            }
        },
        SIMULATION_LINUX {
            @Override
            Path getLibraryPath() {
                return Paths.get("target/release/lib" + LIBRARY_NAME + ".so");
            }
        };

        abstract Path getLibraryPath();

        static RuntimeType getCurrent() {
            if (RobotBase.isReal())
                return ROBORIO;
            String os = System.getProperty("os.name");
            if (os == null || os.toLowerCase().startsWith("windows"))
                return SIMULATION_WINDOWS;
            else
                return SIMULATION_LINUX;
        }
    }
}
