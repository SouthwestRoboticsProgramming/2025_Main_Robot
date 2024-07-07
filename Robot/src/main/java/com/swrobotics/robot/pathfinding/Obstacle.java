package com.swrobotics.robot.pathfinding;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public abstract class Obstacle {
    abstract void addToJNIObstacleList(long obsHandle);

    public static List<Obstacle> loadFromJson(String fileName) throws IOException {
        ObjectMapper mapper = new ObjectMapper();

        File file = new File(Filesystem.getDeployDirectory(), fileName);
        List<Obstacle> obstacles = new ArrayList<>();

        JsonNode rootNode = mapper.readTree(file);
        for (Iterator<JsonNode> iter = rootNode.elements(); iter.hasNext();) {
            JsonNode node = iter.next();
            String type = node.get("type").asText();

            Obstacle obs = switch (type) {
                case "Circle" -> mapper.treeToValue(node, Circle.class);
                case "Rectangle" -> mapper.treeToValue(node, Rectangle.class);
                case "Polygon" -> mapper.treeToValue(node, Polygon.class);
                default -> throw new IOException("Unknown obstacle type: " + type);
            };
            obstacles.add(obs);
        }

        return obstacles;
    }
}
