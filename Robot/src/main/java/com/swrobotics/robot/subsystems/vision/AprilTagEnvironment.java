package com.swrobotics.robot.subsystems.vision;

import com.fasterxml.jackson.annotation.*;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public final class AprilTagEnvironment {
    /**
     * Loads an environment from a JSON file.
     *
     * @param fileName name of the JSON file within the deploy directory
     * @return loaded environment
     * @throws IOException if the environment fails to load from the file
     */
    public static AprilTagEnvironment load(String fileName) throws IOException {
        return new ObjectMapper().readValue(
                new File(Filesystem.getDeployDirectory(), fileName),
                AprilTagEnvironment.class);
    }

    private final double tagSize;
    private final Map<Integer, Pose3d> poses;

    @JsonCreator
    private AprilTagEnvironment(
            @JsonProperty(required = true, value = "tags") List<JsonTag> tags,
            @JsonProperty(required = true, value = "tag_size") double tagSize) {
        this.tagSize = tagSize;
        poses = new HashMap<>();
        for (JsonTag tag : tags) {
            poses.put(tag.ID, tag.pose);
        }
    }

    /**
     * @param tagId ID of the tag to get
     * @return pose if AprilTag exists with the specified ID, else null
     */
    public Pose3d getPose(int tagId) {
        return poses.get(tagId);
    }

    /**
     * @return collection of the poses of all the tags in the environment
     */
    public Collection<Pose3d> getAllPoses() {
        return poses.values();
    }

    /**
     * @return edge length of the tags in meters
     */
    public double getTagSize() {
        return tagSize;
    }

    /**
     * @return map from the tag ID to the tag's pose
     */
    public Map<Integer, Pose3d> getPoseMap() {
        return poses;
    }

    private static final class JsonTag {
        @JsonProperty("ID")
        public final int ID;

        @JsonProperty
        public final Pose3d pose;

        @JsonCreator
        public JsonTag(
                @JsonProperty(required = true, value = "ID") int ID,
                @JsonProperty(required = true, value = "pose") Pose3d pose) {
            this.ID = ID;
            this.pose = pose;
        }
    }
}
