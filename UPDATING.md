# Updating project

Steps to update the project. These are more or less equivalent to what the WPILib project importer does but I find it nicer to know exactly what's changing.

1. Download and install WPILib VS Code and tools from https://github.com/wpilibsuite/allwpilib/releases
2. Open WPILib VS Code and create a new command-based template project
3. Copy the GradleRIO plugin version from the template build.gradle to the robot build.gradle
4. Make sure Java version in robot build.gradle is set to at least the version in the template
4. Copy gradlew, gradlew.bat, and the entire gradle folder into the robot project
5. Copy vendordeps/WPILibNewCommands.json into the robot project
6. Update the projectYear in .wpilib/wpilib_preferences.json
7. Fix any compilation errors in the robot code due to library changes
