package com.swrobotics.lib.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.List;

/**
 * VERY inefficient way to draw arbitrary lines to a Mechanism2d. Should only
 * be used in simulation as it causes issues with most dashboards.
 */
public final class DebugGraphics {
    private final Mechanism2d mechanism;
    private int rootIdx;

    /**
     * Creates a new DebugGraphics and adds it to SmartDashboard.
     *
     * @param name name of the item in SmartDashboard
     * @param w width of the canvas
     * @param h height of the canvas
     */
    public DebugGraphics(String name, double w, double h) {
        mechanism = new Mechanism2d(w, h);
        SmartDashboard.putData(name, mechanism);
        rootIdx = 0;
    }

    // Formats the number such that sorting alphabetically gives the right
    // order. Glass draws the Mechanism2d elements in alphabetical order by
    // their name.
    private String formatAlpha(int i) {
        // This will work up until 10000 elements have been added, which
        // hopefully won't happen
        return String.format("%04d", i);
    }

    /**
     * Plots a sequence of lines onto the canvas.
     *
     * @param positions Points in the line strip. Lines are drawn connecting
     *                  consecutive points in the list. The first and last
     *                  points are not connected, so you need to add the first
     *                  point again if you want a polygon.
     * @param color color to draw the lines
     */
    public void plotLines(List<Translation2d> positions, Color color) {
        Translation2d first = positions.get(0);
        MechanismRoot2d root = mechanism.getRoot("root" + formatAlpha(rootIdx++), first.getX(), first.getY());

        Color8Bit color8Bit = new Color8Bit(color);

        Translation2d prev = first;
        MechanismLigament2d prevLigament = null;
        double prevAngle = 0;
        for (int i = 1; i < positions.size(); i++) {
            Translation2d pos = positions.get(i);

            double dx = pos.getX() - prev.getX();
            double dy = pos.getY() - prev.getY();
            double angle = Math.toDegrees(Math.atan2(dy, dx));
            double length = Math.sqrt(dx * dx + dy * dy);

            MechanismLigament2d ligament2d = new MechanismLigament2d(
                    "line" + i, length, angle - prevAngle,
                    2,
                    color8Bit);
            if (prevLigament == null)
                root.append(ligament2d);
            else
                prevLigament.append(ligament2d);

            prevLigament = ligament2d;
            prev = pos;
            prevAngle = angle;
        }
    }
}
