use std::f64::consts::PI;

use lerp::Lerp;

use super::{
    geom::{PathResult, WindingDir},
    math::{self, Vec2f},
};

pub fn to_bezier(result: &PathResult, start: Vec2f, goal: Vec2f) -> Vec<Vec2f> {
    let PathResult {
        path,
        moved_start,
        moved_goal,
    } = result;

    let mut bezier_pts = Vec::new();
    let mut last_pt = start;

    if let Some(moved_start) = moved_start {
        let moved_start = *moved_start;

        // Segment from original start to moved start
        bezier_pts.push(start);
        bezier_pts.push(start.lerp(moved_start, 1.0 / 3.0));
        bezier_pts.push(start.lerp(moved_start, 2.0 / 3.0));

        last_pt = moved_start;
    }

    for arc in path {
        let angle1 = math::wrap_angle(arc.incoming_angle);
        let angle2 = math::wrap_angle(arc.outgoing_angle);
        let diff = math::floor_mod(
            match arc.direction {
                WindingDir::Counterclockwise => angle2 - angle1,
                WindingDir::Clockwise => angle1 - angle2,
            },
            PI * 2.0,
        );

        let in_pt = arc.center + Vec2f::new_angle(arc.radius, angle1);
        let out_pt = arc.center + Vec2f::new_angle(arc.radius, angle2);

        // Segment from last point to this arc
        bezier_pts.push(last_pt);
        bezier_pts.push(last_pt.lerp(in_pt, 1.0 / 3.0));
        bezier_pts.push(last_pt.lerp(in_pt, 2.0 / 3.0));

        if diff > PI {
            // Need to split the arc since the approximation becomes significantly worse there

            let between_angle = match arc.direction {
                WindingDir::Counterclockwise => angle1 + diff / 2.0,
                WindingDir::Clockwise => angle2 + diff / 2.0,
            };
            let between_pt = arc.center + Vec2f::new_angle(arc.radius, between_angle);

            let l = 4.0 / 3.0 * (diff / 8.0).tan() * arc.radius;
            bezier_pts.push(in_pt);
            bezier_pts.push(in_pt + (in_pt - last_pt).norm() * l);
            bezier_pts.push(
                between_pt
                    + Vec2f::new_angle(
                        l,
                        angle2
                            + match arc.direction {
                                WindingDir::Clockwise => PI / 2.0,
                                WindingDir::Counterclockwise => -PI / 2.0,
                            },
                    ),
            );
            bezier_pts.push(between_pt);
            bezier_pts.push(
                between_pt
                    + Vec2f::new_angle(
                        l,
                        angle2
                            + match arc.direction {
                                WindingDir::Clockwise => -PI / 2.0,
                                WindingDir::Counterclockwise => PI / 2.0,
                            },
                    ),
            );
            bezier_pts.push(
                out_pt
                    + Vec2f::new_angle(
                        l,
                        angle2
                            + match arc.direction {
                                WindingDir::Clockwise => PI / 2.0,
                                WindingDir::Counterclockwise => -PI / 2.0,
                            },
                    ),
            );
        } else {
            let l = 4.0 / 3.0 * (diff / 4.0).tan() * arc.radius;

            bezier_pts.push(in_pt);
            bezier_pts.push(in_pt + (in_pt - last_pt).norm() * l);
            bezier_pts.push(
                out_pt
                    + Vec2f::new_angle(
                        l,
                        angle2
                            + match arc.direction {
                                WindingDir::Clockwise => PI / 2.0,
                                WindingDir::Counterclockwise => -PI / 2.0,
                            },
                    ),
            );
        }

        last_pt = out_pt;
    }

    if let Some(moved_goal) = moved_goal {
        bezier_pts.push(last_pt);
        bezier_pts.push(last_pt.lerp(*moved_goal, 1.0 / 3.0));
        bezier_pts.push(last_pt.lerp(*moved_goal, 2.0 / 3.0));
        last_pt = *moved_goal;
    }

    bezier_pts.push(last_pt);
    bezier_pts.push(last_pt.lerp(goal, 1.0 / 3.0));
    bezier_pts.push(last_pt.lerp(goal, 2.0 / 3.0));
    bezier_pts.push(goal);

    bezier_pts
}
