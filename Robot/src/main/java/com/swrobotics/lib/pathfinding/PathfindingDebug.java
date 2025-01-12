package com.swrobotics.lib.pathfinding;

import com.swrobotics.lib.utils.DoubleInput;
import com.swrobotics.lib.utils.MathUtil;
import com.swrobotics.lib.utils.DebugGraphics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class PathfindingDebug {
    public static final class Arc {
        public final double centerX, centerY;
        public final double radius;
        public final double minAngle, maxAngle;

        public Arc(DoubleInput in) {
            centerX = in.next();
            centerY = in.next();
            radius = in.next();
            minAngle = in.next();
            maxAngle = in.next();
        }
    }

    public static final class Segment {
        public final double x1, y1;
        public final double x2, y2;

        public Segment(DoubleInput in) {
            x1 = in.next();
            y1 = in.next();
            x2 = in.next();
            y2 = in.next();
        }
    }

    public static final class Triangle {
        public final Translation2d v1, v2, v3;

        public Triangle(DoubleInput in) {
            v1 = new Translation2d(in.next(), in.next());
            v2 = new Translation2d(in.next(), in.next());
            v3 = new Translation2d(in.next(), in.next());
        }
    }

    public static final class EnvPolygon {
        public Triangle[] internalRegion;

        public EnvPolygon(DoubleInput in) {
            internalRegion = new Triangle[(int) in.next()];
            for (int i = 0; i < internalRegion.length; i++) {
                internalRegion[i] = new Triangle(in);
            }
        }
    }

    public final List<Obstacle> obstacles;
    public final Arc[] arcs;
    public final Segment[] segments;
    public final EnvPolygon[] envPolygons;
    public final Segment[] visibility;

    /**
     * Decodes the pathfinding debug data array provided.
     *
     * @param obstacles obstacles in the environment
     * @param data debug data
     * @throws ArrayIndexOutOfBoundsException if the debug data is incomplete
     */
    public PathfindingDebug(List<Obstacle> obstacles, double[] data) {
        this.obstacles = obstacles;

        DoubleInput in = new DoubleInput(data);

        arcs = new Arc[(int) in.next()];
        for (int i = 0; i < arcs.length; i++) {
            arcs[i] = new Arc(in);
        }

        segments = new Segment[(int) in.next()];
        for (int i = 0; i < segments.length; i++) {
            segments[i] = new Segment(in);
        }

        envPolygons = new EnvPolygon[(int) in.next()];
        for (int i = 0; i < envPolygons.length; i++) {
            envPolygons[i] = new EnvPolygon(in);
        }

        visibility = new Segment[(int) in.next()];
        for (int i = 0; i < visibility.length; i++) {
            visibility[i] = new Segment(in);
        }
    }

    /**
     * Draws the debug data into the debug graphics provided
     * @param g graphics to draw into
     */
    public void plot(DebugGraphics g) {
        for (Obstacle obs : obstacles) {
            if (obs instanceof Circle circle) {
                List<Translation2d> points = new ArrayList<>();
                for (int i = 0; i <= 8; i++) {
                    double angle = i / 8.0 * Math.PI * 2;

                    points.add(new Translation2d(
                            circle.getCenter().getX() + circle.getRadius() * Math.cos(angle),
                            circle.getCenter().getY() + circle.getRadius() * Math.sin(angle)));
                }
                g.plotLines(points, Color.kOrange);
            } else if (obs instanceof Polygon poly) {
                List<Translation2d> points = new ArrayList<>(Arrays.asList(poly.getVertices()));
                points.add(points.get(0));
                g.plotLines(points, Color.kOrange);
            } else if (obs instanceof Rectangle rect) {
                List<Translation2d> points = new ArrayList<>(Arrays.asList(rect.asPolygon().getVertices()));
                points.add(points.get(0));
                g.plotLines(points, Color.kOrange);
            }
        }

        // for (EnvPolygon poly : envPolygons) {
        //     for (Triangle tri : poly.internalRegion) {
        //         g.plotLines(List.of(tri.v1, tri.v2, tri.v3, tri.v1), Color.kBlue);
        //     }
        // }

        for (Segment seg : segments) {
            g.plotLines(List.of(
                    new Translation2d(seg.x1, seg.y1),
                    new Translation2d(seg.x2, seg.y2)), Color.kYellow);

            Translation2d center = new Translation2d((seg.x1 + seg.x2) / 2, (seg.y1 + seg.y2) / 2);
            Translation2d perp = new Translation2d(seg.y1 - seg.y2, seg.x2 - seg.x1);
            perp = perp.div(perp.getNorm()).times(0.2);

            g.plotLines(List.of(
                    center,
                    center.plus(perp)), Color.kRed);
        }

        // Arc boundaries
        // for (Arc arc : arcs) {
        //     List<Translation2d> points = new ArrayList<>();
        //     for (int i = 0; i <= 8; i++) {
        //         double angle = (i / 8.0) * MathUtil.TAU;

        //         points.add(new Translation2d(
        //                 arc.centerX + arc.radius * Math.cos(angle),
        //                 arc.centerY + arc.radius * Math.sin(angle)));
        //     }
        //     g.plotLines(points, Color.kDimGray);
        // }

        // Actual arcs
        for (Arc arc : arcs) {
            double max = arc.maxAngle;
            if (max <= arc.minAngle)
                max += Math.PI * 2;

            List<Translation2d> points = new ArrayList<>();
            for (int i = 0; i <= 8; i++) {
                double f = i / 8.0f;
                double angle = MathUtil.lerp(arc.minAngle, max, f);

                points.add(new Translation2d(
                        arc.centerX + arc.radius * Math.cos(angle),
                        arc.centerY + arc.radius * Math.sin(angle)));
            }
            g.plotLines(points, Color.kYellow);
        }

        for (Segment seg : visibility) {
            g.plotLines(List.of(
                    new Translation2d(seg.x1, seg.y1),
                    new Translation2d(seg.x2, seg.y2)), Color.kBlue);
        }
    }
}
