use std::{cmp::Ordering, collections::BinaryHeap, f64::consts::PI, rc::Rc};

// BUG: I think visibility is not taking arc limits into account

use arrayvec::ArrayVec;
use itertools::Itertools;
use lerp::Lerp;
use ordered_float::OrderedFloat;

use super::{
    math::{self, wrap_angle, Vec2f},
    obstacle::Obstacle,
    triangulate::Triangle,
};

// Maximum number of attempts made to get back into safe area if inside an obstacle
const MAX_UNSTUCK_ATTEMPTS: usize = 8;

#[derive(Clone)]
pub struct Arc {
    pub center: Vec2f,
    pub radius: f64,
    pub min_angle: f64,
    pub max_angle: f64,
}

impl Arc {
    pub fn new(center: Vec2f, radius: f64, min_angle: f64, max_angle: f64) -> Self {
        Self {
            center,
            radius,
            min_angle: math::wrap_angle(min_angle),
            max_angle: math::wrap_angle(max_angle),
        }
    }

    pub fn contains_angle(&self, angle: f64) -> bool {
        if self.min_angle == self.max_angle {
            return true;
        }

        let mut angle = math::wrap_angle(angle);

        let mut rel_max = self.max_angle;
        if rel_max < self.min_angle {
            rel_max += PI * 2.0;
        }
        if angle < self.min_angle {
            angle += PI * 2.0;
        }

        angle >= self.min_angle && angle <= rel_max
    }
}

#[derive(Clone, Debug)]
pub struct Segment {
    pub from: Vec2f,
    pub to: Vec2f,
}

impl Segment {
    pub fn length(&self) -> f64 {
        (self.to - self.from).length()
    }
}

// -------------------------

pub struct EnvPolygon {
    pub arcs: Vec<Arc>,
    pub segments: Vec<Segment>,
    pub inner_region: Vec<Triangle>,
    pub inverted: bool,
}

impl EnvPolygon {
    fn is_inside(&self, point: Vec2f) -> bool {
        let mut inside = false;
        for tri in &self.inner_region {
            if tri.contains(point) {
                inside = true;
                break;
            }
        }

        // The triangulation is slightly conservative around arcs, but if it gives a result we can
        // return early
        if self.inverted {
            if !inside {
                return true;
            }
        } else {
            if inside {
                return true;
            }
        }

        // If triangulation did not give a result we need to check each arc. These will account for
        // the regions the triangulation does not cover
        for arc in &self.arcs {
            if (point - arc.center).length_sq() < arc.radius * arc.radius {
                return true;
            }
        }

        return false;
    }
}

fn angle_to_arc(arc: &Arc, point: Vec2f) -> f64 {
    (point - arc.center).angle()
}

struct ArcSegmentIntersection {
    s: f64,     // S value along the segment
    angle: f64, // Angle to the arc
}

fn find_arc_segment_intersection_common(
    seg: &Segment,
    arc: &Arc,
) -> ArrayVec<ArcSegmentIntersection, 2> {
    let mut out = ArrayVec::new();

    let rel = seg.from - arc.center;
    let d = seg.to - seg.from;

    let a = d.length_sq();
    let b = 2.0 * (rel.x * d.x + rel.y * d.y);
    let c = rel.length_sq() - arc.radius * arc.radius;

    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        // No solutions
        return out; // out is empty at this point
    }

    let sqrt_disc = disc.sqrt();
    let s1 = (-b + sqrt_disc) / (2.0 * a);
    let s2 = (-b - sqrt_disc) / (2.0 * a);

    if s1 >= 0.0 && s1 <= 1.0 {
        let p = seg.from.lerp(seg.to, s1);
        let angle = angle_to_arc(arc, p);
        if arc.contains_angle(angle) {
            out.push(ArcSegmentIntersection { s: s1, angle });
        }
    }
    if s2 != s1 && s2 >= 0.0 && s2 <= 1.0 {
        let p = seg.from.lerp(seg.to, s2);
        let angle = angle_to_arc(arc, p);
        if arc.contains_angle(angle) {
            out.push(ArcSegmentIntersection { s: s2, angle });
        }
    }

    out
}

fn find_arc_segment_intersections(seg: &Segment, arc: &Arc, angles_out: &mut Vec<f64>) {
    for int in find_arc_segment_intersection_common(seg, arc) {
        angles_out.push(int.angle);
    }
}

// Puts resulting S values along seg into s_out
fn find_segment_arc_intersections(seg: &Segment, arc: &Arc, s_out: &mut Vec<f64>) {
    for int in find_arc_segment_intersection_common(seg, arc) {
        s_out.push(int.s);
    }
}

// Based on https://gist.github.com/jupdike/bfe5eb23d1c395d8a0a1a4ddd94882ac
// Returns angles relative to arc a
fn find_arc_intersections(arc_a: &Arc, arc_b: &Arc, angles_out: &mut Vec<f64>) {
    let d = arc_a.center - arc_b.center;
    let r = d.length();
    if (arc_a.radius - arc_b.radius).abs() > r || r > arc_a.radius + arc_b.radius {
        return;
    }

    let r2 = r * r;
    let r4 = r2 * r2;
    let a = (arc_a.radius * arc_a.radius - arc_b.radius * arc_b.radius) / (2.0 * r2);
    let r2r2 = arc_a.radius * arc_a.radius - arc_b.radius * arc_b.radius;
    let c = (2.0 * (arc_a.radius * arc_a.radius + arc_b.radius * arc_b.radius) / r2
        - (r2r2 * r2r2) / r4
        - 1.0)
        .sqrt();

    let fx = (arc_a.center.x + arc_b.center.x) / 2.0 + a * (arc_b.center.x - arc_a.center.x);
    let gx = c * (arc_b.center.y - arc_a.center.y) / 2.0;
    let ix1 = fx + gx;
    let ix2 = fx - gx;

    let fy = (arc_a.center.y + arc_b.center.y) / 2.0 + a * (arc_b.center.y - arc_a.center.y);
    let gy = c * (arc_a.center.x - arc_b.center.x) / 2.0;
    let iy1 = fy + gy;
    let iy2 = fy - gy;

    let i1 = Vec2f::new(ix1, iy1);
    let i2 = Vec2f::new(ix2, iy2);

    let angle1 = angle_to_arc(arc_a, i1);
    if arc_a.contains_angle(angle1) && arc_b.contains_angle(angle_to_arc(arc_b, i2)) {
        angles_out.push(angle1);
    }

    let angle2 = angle_to_arc(arc_a, i2);
    if (ix1 != ix2 || iy1 != iy2)
        && arc_a.contains_angle(angle2)
        && arc_b.contains_angle(angle_to_arc(arc_b, i2))
    {
        angles_out.push(angle2);
    }
}

// Returns the S value of the intersection along segment A
fn find_segment_intersection(a: &Segment, b: &Segment) -> Option<f64> {
    // From https://stackoverflow.com/a/18929324

    let d1 = a.to - a.from;
    let d2 = b.to - b.from;

    let vp = d1.x * d2.y - d2.x * d1.y;
    if vp.abs() < 0.001 {
        // Ignore collision if the segments are collinear
        // This is so a segment between two vertices of a polygon cannot
        // collide with its own edge
        return None;
    }

    let v = b.from - a.from;

    let k1 = (v.x * d2.y - v.y * d2.x) / vp;
    if 0.0 <= k1 && k1 <= 1.0 {
        let k2 = (v.x * d1.y - v.y * d1.x) / vp;
        if 0.0 <= k2 && k2 <= 1.0 {
            Some(k1)
        } else {
            None
        }
    } else {
        None
    }
}

fn clip_arc(to_clip: Arc, poly: &EnvPolygon, out: &mut Vec<Arc>) {
    let mut raw_angles = Vec::new();
    for segment in &poly.segments {
        find_arc_segment_intersections(segment, &to_clip, &mut raw_angles);
    }
    for arc in &poly.arcs {
        find_arc_intersections(&to_clip, arc, &mut raw_angles);
    }

    let mut intersect_angles = Vec::new();
    for angle in raw_angles {
        let mut angle = wrap_angle(angle);
        if angle < to_clip.min_angle {
            angle += PI * 2.0;
        }

        let mut add = true;
        for existing in &intersect_angles {
            let diff: f64 = existing - angle;
            if diff.abs() < 0.01 {
                add = false;
                break;
            }
        }
        if add {
            intersect_angles.push(angle);
        }
    }
    intersect_angles.sort_by(|a, b| a.total_cmp(b));

    let mut to_clip_max_angle = to_clip.max_angle;
    if to_clip_max_angle <= to_clip.min_angle {
        to_clip_max_angle += PI * 2.0;
    }

    for i in 0..(intersect_angles.len() + 1) {
        let min_angle = if i == 0 {
            to_clip.min_angle
        } else {
            intersect_angles[i - 1]
        };
        let mut max_angle = if i == intersect_angles.len() {
            to_clip_max_angle
        } else {
            intersect_angles[i]
        };

        if max_angle < to_clip.min_angle {
            max_angle += PI * 2.0;
        }
        if min_angle == max_angle && intersect_angles.len() != 0 {
            continue;
        }

        let halfway = (max_angle + min_angle) / 2.0;
        let test_point = to_clip.center + Vec2f::new_angle(to_clip.radius, halfway);

        if !poly.is_inside(test_point) {
            // Outside the polygon, keep this piece of the arc
            // FIXME: If the previous arc was also kept, they must be merged
            out.push(Arc {
                center: to_clip.center,
                radius: to_clip.radius,
                min_angle: math::wrap_angle(min_angle),
                max_angle: math::wrap_angle(max_angle),
            });
        }
    }
}

fn clip_segment(to_clip: Segment, poly: &EnvPolygon, out: &mut Vec<Segment>) {
    let mut raw_s = Vec::new();
    for segment in &poly.segments {
        if let Some(s) = find_segment_intersection(&to_clip, segment) {
            raw_s.push(s);
        }
    }
    for arc in &poly.arcs {
        find_segment_arc_intersections(&to_clip, arc, &mut raw_s);
    }

    let mut intersect_s = Vec::new();
    for s in raw_s {
        let mut add = true;
        for existing in &intersect_s {
            let diff: f64 = existing - s;
            if diff.abs() < 0.01 {
                add = false;
                break;
            }
        }
        if add {
            intersect_s.push(s);
        }
    }
    intersect_s.sort_by(|a, b| a.total_cmp(b));

    for i in 0..(intersect_s.len() + 1) {
        let min_s = if i == 0 { 0.0 } else { intersect_s[i - 1] };
        let max_s = if i == intersect_s.len() {
            1.0
        } else {
            intersect_s[i]
        };
        if min_s == max_s {
            continue;
        }

        let halfway = (min_s + max_s) / 2.0;
        let test_point = to_clip.from.lerp(to_clip.to, halfway);

        if !poly.is_inside(test_point) {
            out.push(Segment {
                from: to_clip.from.lerp(to_clip.to, min_s),
                to: to_clip.from.lerp(to_clip.to, max_s),
            });
        }
    }
}

// -------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WindingDir {
    Clockwise,
    Counterclockwise,
}

impl WindingDir {
    pub fn opposite(self) -> Self {
        match self {
            Self::Clockwise => Self::Counterclockwise,
            Self::Counterclockwise => Self::Clockwise,
        }
    }
}

#[derive(Debug)]
pub struct PointToArcTangent {
    pub segment: Segment,
    pub arc_angle: f64,
    pub arc_dir: WindingDir,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct ArcContext(pub usize);

impl ArcContext {
    pub fn new(id: usize, dir: WindingDir) -> Self {
        Self(
            id << 1
                | match dir {
                    WindingDir::Clockwise => 0,
                    WindingDir::Counterclockwise => 1,
                },
        )
    }

    pub fn arc_id(self) -> usize {
        self.0 >> 1
    }

    pub fn dir(self) -> WindingDir {
        match self.0 & 1 {
            0 => WindingDir::Clockwise,
            1 => WindingDir::Counterclockwise,
            _ => unreachable!(),
        }
    }
}

pub struct VisibilityEdge {
    pub segment: Segment,
    pub from_angle: f64,
    pub to_angle: f64,
    pub dest: ArcContext,
}

#[derive(Clone, Debug)]
pub struct PathArc {
    pub center: Vec2f,
    pub radius: f64,
    pub incoming_angle: f64,
    pub outgoing_angle: f64,
    pub direction: WindingDir,
}

struct SearchNode {
    pub context: ArcContext,
    pub is_goal: bool,

    // TODO: Should probably not allocate every node on heap
    pub came_from: Option<Rc<SearchNode>>,
    pub incoming_angle: f64,
    pub parent_outgoing_angle: f64,

    pub cost_so_far: f64,
}

impl SearchNode {
    pub fn has_visited(&self, context: ArcContext) -> bool {
        if context == self.context {
            return true;
        }

        match &self.came_from {
            Some(node) => node.has_visited(context),
            None => false,
        }
    }
}

impl Ord for SearchNode {
    fn cmp(&self, other: &Self) -> Ordering {
        OrderedFloat(self.cost_so_far)
            .cmp(&OrderedFloat(other.cost_so_far))
            .reverse() // Turns the heap into a minheap
    }
}

impl PartialOrd for SearchNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for SearchNode {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for SearchNode {}

pub struct Environment {
    pub arcs: Vec<Arc>,
    pub segments: Vec<Segment>,

    pub visibility: Vec<Vec<VisibilityEdge>>,

    gen_debug: Vec<f64>,
}

fn calc_turn_cost(incoming_angle: f64, outgoing_angle: f64, radius: f64, dir: WindingDir) -> f64 {
    let diff = math::floor_mod(
        match dir {
            WindingDir::Counterclockwise => outgoing_angle - incoming_angle,
            WindingDir::Clockwise => incoming_angle - outgoing_angle,
        },
        PI * 2.0,
    );

    diff * radius
}

pub struct PathResult {
    pub moved_start: Option<Vec2f>,
    pub moved_goal: Option<Vec2f>,
    pub path: Vec<PathArc>,
}

impl Environment {
    pub fn generate(obstacles: &Vec<Obstacle>, inflate: f64) -> Self {
        let polygons = obstacles
            .iter()
            .map(|obs| obs.convert_into_env(inflate))
            .collect_vec();

        let mut gen_debug = Vec::new();
        gen_debug.push(polygons.len() as f64);
        for poly in &polygons {
            gen_debug.push(poly.inner_region.len() as f64);
            for tri in &poly.inner_region {
                gen_debug.extend([tri.0.x, tri.0.y, tri.1.x, tri.1.y, tri.2.x, tri.2.y].iter());
            }
        }

        let mut all_arcs = Vec::new();
        let mut all_segments = Vec::new();
        for i1 in 0..polygons.len() {
            let to_clip = &polygons[i1];

            let mut arcs = to_clip.arcs.clone();
            let mut segments = to_clip.segments.clone();
            for i2 in 0..polygons.len() {
                if i1 == i2 {
                    continue;
                }
                let against = &polygons[i2];

                let mut clipped_arcs = Vec::new();
                for arc in arcs {
                    clip_arc(arc, &against, &mut clipped_arcs);
                }
                arcs = clipped_arcs;

                let mut clipped_segs = Vec::new();
                for seg in segments {
                    clip_segment(seg, &against, &mut clipped_segs);
                }
                segments = clipped_segs;
            }

            all_arcs.append(&mut arcs);
            all_segments.append(&mut segments);
        }

        let mut field = Self {
            segments: all_segments,
            visibility: Vec::with_capacity(all_arcs.len() * 2),
            arcs: all_arcs,
            gen_debug,
        };
        field.calc_visibility();

        field
    }

    pub fn get_debug_data(&self) -> Vec<f64> {
        let mut data = Vec::new();

        data.push(self.arcs.len() as f64);
        for arc in &self.arcs {
            data.push(arc.center.x);
            data.push(arc.center.y);
            data.push(arc.radius);
            data.push(arc.min_angle);
            data.push(arc.max_angle);
        }

        data.push(self.segments.len() as f64);
        for seg in &self.segments {
            data.push(seg.from.x);
            data.push(seg.from.y);
            data.push(seg.to.x);
            data.push(seg.to.y);
        }

        data.extend(self.gen_debug.iter());

        let mut vis_data: Vec<f64> = Vec::new();
        for vis_set in &self.visibility {
            for vis_edge in vis_set {
                let seg = &vis_edge.segment;

                vis_data.extend([seg.from.x, seg.from.y, seg.to.x, seg.to.y].iter());
            }
        }
        data.push((vis_data.len() / 4) as f64);
        data.extend(vis_data);

        data
    }

    // Moves an invalid/unreachable point towards the nearest reachable area.
    // May take multiple iterations to actually become reachable.
    fn move_towards_safety(&self, point: Vec2f) -> Option<Vec2f> {
        let mut nearest = None;
        for segment in &self.segments {
            // Check if the start is on the "inside" of this edge (i.e. to
            // the left of the edge)
            let delta = segment.to - segment.from;
            let perp = Vec2f::new(delta.y, -delta.x);
            let rel = point - segment.from;
            if rel.dot(perp) < 0.0 {
                continue;
            }

            let p1 = segment.from;
            let p2 = segment.to;
            let l2 = p2.distance_sq(p1);

            // Project the current position onto the segment
            let proj_point = if l2 == 0.0 {
                p1
            } else {
                let t = ((point.x - p1.x) * (p2.x - p1.x) + (point.y - p1.y) * (p2.y - p1.y)) / l2;
                // TODO: These bounds should be determined by the actual corner angles
                let t = t.clamp(0.1, 0.9); // Not 0 to 1 to prevent getting trapped in acute concave corners
                p1.lerp(p2, t)
            };
            let dist = point.distance_sq(proj_point);

            if match nearest {
                Some((d, _)) => dist < d,
                None => true,
            } {
                nearest = Some((dist, proj_point));
            }
        }
        for arc in &self.arcs {
            let rel = point - arc.center;
            let angle = angle_to_arc(arc, point);

            if rel.length_sq() < arc.radius * arc.radius && arc.contains_angle(angle) {
                // If the point is inside the sector spanned by the arc, project to be along the arc
                let proj_point = arc.center + (rel.norm() * arc.radius);
                let dist = point.distance_sq(proj_point);

                if match nearest {
                    Some((d, _)) => dist < d,
                    None => true,
                } {
                    nearest = Some((dist, proj_point));
                }
            } else {
                // Not inside the sector, but bounds may still be closest point so check both
                let proj_min = arc.center + Vec2f::new_angle(arc.radius, arc.min_angle);
                let proj_max = arc.center + Vec2f::new_angle(arc.radius, arc.max_angle);
                let dist_min = point.distance_sq(proj_min);
                let dist_max = point.distance_sq(proj_max);

                if match nearest {
                    Some((d, _)) => dist_min < d,
                    None => true,
                } {
                    nearest = Some((dist_min, proj_min));
                }
                if match nearest {
                    Some((d, _)) => dist_max < d,
                    None => true,
                } {
                    nearest = Some((dist_max, proj_max));
                }
            }
        }

        match nearest {
            Some((_, proj_point)) => Some(proj_point + (proj_point - point).norm() * 0.05),
            None => return None,
        }
    }

    pub fn debug_find_safe(&self, mut start: Vec2f) -> Vec<f64> {
        let mut out = Vec::new();

        let mut safe_data: Vec<f64> = Vec::new();
        let mut tangents: ArrayVec<_, 2> = ArrayVec::new();
        'outer: for _ in 0..MAX_UNSTUCK_ATTEMPTS {
            // Check if we are inside the field bounds
            for (arc_id, arc) in self.arcs.iter().enumerate() {
                self.find_point_to_arc_tangents(start, &arc, arc_id, &mut tangents);
                if !tangents.is_empty() {
                    break 'outer;
                }
            }

            match self.move_towards_safety(start) {
                Some(new_start) => {
                    start = new_start;
                    safe_data.push(new_start.x);
                    safe_data.push(new_start.y);
                    println!("New start: {new_start:?}");
                }
                None => break 'outer,
            }
        }

        let mut vis_data: Vec<f64> = Vec::new();
        for (arc_id, arc) in self.arcs.iter().enumerate() {
            self.find_point_to_arc_tangents(start, &arc, arc_id, &mut tangents);
            for tangent in &tangents {
                let seg = &tangent.segment;
                vis_data.extend([seg.from.x, seg.from.y, seg.to.x, seg.to.y].iter());
            }
        }

        out.push((safe_data.len() / 2) as f64);
        out.append(&mut safe_data);
        out.push((vis_data.len() / 4) as f64);
        out.append(&mut vis_data);
        out
    }

    pub fn find_path(&self, mut start: Vec2f, mut goal: Vec2f) -> Option<PathResult> {
        // Check if the goal is unreachable, since if we try to search for unreachable
        // goal it will loop forever
        let mut tangents: ArrayVec<_, 2> = ArrayVec::new();
        let mut goal_changed = false;
        let mut goal_reachable = false;
        'goal_loop: for _ in 0..MAX_UNSTUCK_ATTEMPTS {
            for (arc_id, arc) in self.arcs.iter().enumerate() {
                self.find_point_to_arc_tangents(goal, arc, arc_id, &mut tangents);
                if !tangents.is_empty() {
                    goal_reachable = true;
                    break 'goal_loop;
                }
            }

            match self.move_towards_safety(goal) {
                Some(new_goal) => {
                    goal = new_goal;
                    goal_changed = true;
                }
                None => return None,
            }
        }
        if !goal_reachable {
            return None;
        }

        // Check straight-line case first, very likely to be possible
        if self.is_segment_passable(
            &Segment {
                from: start,
                to: goal,
            },
            None,
            None,
        ) {
            // Straight line from start to goal
            return Some(PathResult {
                moved_start: None,
                moved_goal: if goal_changed { Some(goal) } else { None },
                path: Vec::new(),
            });
        }

        let mut frontier = BinaryHeap::new();
        let mut start_changed = false;
        for _ in 0..MAX_UNSTUCK_ATTEMPTS {
            // Check if we are inside the field bounds
            for (arc_id, arc) in self.arcs.iter().enumerate() {
                self.find_point_to_arc_tangents(start, &arc, arc_id, &mut tangents);
                for tangent in &tangents {
                    let cost = tangent.segment.length();
                    frontier.push(Rc::new(SearchNode {
                        context: ArcContext::new(arc_id, tangent.arc_dir),
                        is_goal: false,
                        came_from: None,
                        incoming_angle: tangent.arc_angle,
                        parent_outgoing_angle: 0.0,
                        cost_so_far: cost,
                    }));
                }
            }
            if !frontier.is_empty() {
                break;
            }

            match self.move_towards_safety(start) {
                Some(new_start) => {
                    start = new_start;
                    start_changed = true;
                }
                None => return None,
            }
        }

        if frontier.is_empty() {
            // Couldn't find a way back to safe area
            return None;
        }

        // Try straight line again if it changed
        if start_changed {
            if self.is_segment_passable(
                &Segment {
                    from: start,
                    to: goal,
                },
                None,
                None,
            ) {
                // Straight line from start to goal
                return Some(PathResult {
                    moved_start: if start_changed { Some(start) } else { None },
                    moved_goal: if goal_changed { Some(goal) } else { None },
                    path: Vec::new(),
                });
            }
        }

        println!("Searching from {start:?} to {goal:?}");

        while let Some(current) = frontier.pop() {
            if current.is_goal {
                let mut node = current;
                let mut out = Vec::new();
                println!("Path cost progress (REVERSED):");
                loop {
                    println!("Cost = {}", node.cost_so_far);

                    let outgoing = node.parent_outgoing_angle;
                    match &node.came_from {
                        None => break,
                        Some(parent) => node = parent.clone(),
                    }

                    let incoming = node.incoming_angle;
                    let arc = &self.arcs[node.context.arc_id()];
                    out.push(PathArc {
                        center: arc.center,
                        radius: arc.radius,
                        incoming_angle: incoming,
                        outgoing_angle: outgoing,
                        direction: node.context.dir(),
                    });
                }

                out.reverse();

                println!("Path arcs: {out:?}");

                return Some(PathResult {
                    moved_start: if start_changed { Some(start) } else { None },
                    moved_goal: if goal_changed { Some(goal) } else { None },
                    path: out,
                });
            }

            let current_arc_id = current.context.arc_id();
            let current_dir = current.context.dir();
            let current_arc = &self.arcs[current_arc_id];

            for edge in &self.visibility[current.context.0] {
                if !current.has_visited(edge.dest) {
                    let distance_cost = edge.segment.length();
                    let turn_cost = calc_turn_cost(
                        current.incoming_angle,
                        edge.from_angle,
                        current_arc.radius,
                        current.context.dir(),
                    );

                    frontier.push(Rc::new(SearchNode {
                        context: edge.dest,
                        is_goal: false,
                        came_from: Some(current.clone()),
                        incoming_angle: edge.to_angle,
                        parent_outgoing_angle: edge.from_angle,
                        cost_so_far: current.cost_so_far + distance_cost + turn_cost,
                    }));
                }
            }

            self.find_point_to_arc_tangents(goal, current_arc, current_arc_id, &mut tangents);
            for tangent in &tangents {
                if current_dir == tangent.arc_dir {
                    continue;
                }

                let distance_cost = tangent.segment.length();
                let turn_cost = calc_turn_cost(
                    current.incoming_angle,
                    tangent.arc_angle,
                    current_arc.radius,
                    current_dir,
                );

                frontier.push(Rc::new(SearchNode {
                    context: ArcContext(0), // FIXME: Restructure type so this can be omitted
                    is_goal: true,
                    came_from: Some(current.clone()),
                    incoming_angle: 0.0,
                    parent_outgoing_angle: tangent.arc_angle,
                    cost_so_far: current.cost_so_far + distance_cost + turn_cost,
                }));
            }
        }

        // Didn't find a path :(
        None
    }

    fn find_point_to_arc_tangents(
        &self,
        point: Vec2f,
        arc: &Arc,
        arc_id: usize,
        out: &mut ArrayVec<PointToArcTangent, 2>,
    ) {
        out.clear();

        let d = point - arc.center;
        let distance = d.length();
        if distance < arc.radius {
            // Point is inside the arc, no tangents
            return;
        }

        let angle_offset = (arc.radius / distance).acos();
        let base_angle = d.angle();

        let cw_angle = base_angle - angle_offset;
        let ccw_angle = base_angle + angle_offset;

        if arc.contains_angle(cw_angle) {
            let cw = Segment {
                from: point,
                to: arc.center + Vec2f::new_angle(arc.radius, cw_angle),
            };

            if self.is_segment_passable(&cw, Some(arc_id), None) {
                out.push(PointToArcTangent {
                    segment: cw,
                    arc_angle: cw_angle,
                    arc_dir: WindingDir::Clockwise,
                });
            }
        }

        if arc.contains_angle(ccw_angle) {
            let ccw = Segment {
                from: point,
                to: arc.center + Vec2f::new_angle(arc.radius, ccw_angle),
            };

            if self.is_segment_passable(&ccw, Some(arc_id), None) {
                out.push(PointToArcTangent {
                    segment: ccw,
                    arc_angle: ccw_angle,
                    arc_dir: WindingDir::Counterclockwise,
                });
            }
        }
    }

    fn calc_visibility(&mut self) {
        // TODO: This can be optimized by only iterating half, since
        // visibility is bidirectional
        for (from_id, from_arc) in self.arcs.iter().enumerate() {
            let mut visible_cw = Vec::new();
            let mut visible_ccw = Vec::new();

            for (to_id, to_arc) in self.arcs.iter().enumerate() {
                if from_id == to_id {
                    continue;
                }

                self.find_visible_edges(
                    from_arc,
                    from_id,
                    to_arc,
                    to_id,
                    WindingDir::Clockwise,
                    &mut visible_cw,
                );
                self.find_visible_edges(
                    from_arc,
                    from_id,
                    to_arc,
                    to_id,
                    WindingDir::Counterclockwise,
                    &mut visible_ccw,
                );
            }

            self.visibility.push(visible_cw);
            self.visibility.push(visible_ccw);
        }
    }

    fn find_visible_edges(
        &self,
        from: &Arc,
        from_id: usize,
        to: &Arc,
        to_id: usize,
        from_dir: WindingDir,
        out: &mut Vec<VisibilityEdge>,
    ) {
        let d = to.center - from.center;
        let distance = d.length();
        let intersect_dist = distance * from.radius / (from.radius + to.radius);

        let base_angle = d.angle();
        let angle_flip = match from_dir {
            WindingDir::Clockwise => 1.0,
            WindingDir::Counterclockwise => -1.0,
        };
        let same_dir =
            base_angle + angle_flip * (PI / 2.0 + ((to.radius - from.radius) / distance).asin());
        let cross_dir = base_angle + angle_flip * (from.radius / intersect_dist).acos();

        if from.contains_angle(same_dir) && to.contains_angle(same_dir) {
            let same = Segment {
                from: from.center + Vec2f::new_angle(from.radius, same_dir),
                to: to.center + Vec2f::new_angle(to.radius, same_dir),
            };

            if self.is_segment_passable(&same, Some(from_id), Some(to_id)) {
                out.push(VisibilityEdge {
                    segment: same,
                    from_angle: same_dir,
                    to_angle: same_dir,
                    dest: ArcContext::new(to_id, from_dir),
                });
            }
        }

        if from.contains_angle(cross_dir) && to.contains_angle(cross_dir + PI) {
            let cross = Segment {
                from: from.center + Vec2f::new_angle(from.radius, cross_dir),
                to: to.center - Vec2f::new_angle(to.radius, cross_dir),
            };

            if self.is_segment_passable(&cross, Some(from_id), Some(to_id)) {
                out.push(VisibilityEdge {
                    segment: cross,
                    from_angle: cross_dir,
                    to_angle: cross_dir + PI,
                    dest: ArcContext::new(to_id, from_dir.opposite()),
                })
            }
        }
    }

    fn is_segment_passable(
        &self,
        seg: &Segment,
        ignore_a: Option<usize>,
        ignore_b: Option<usize>,
    ) -> bool {
        for (i, arc) in self.arcs.iter().enumerate() {
            if let Some(a) = ignore_a {
                if a == i {
                    continue;
                }
            }
            if let Some(b) = ignore_b {
                if b == i {
                    continue;
                }
            }

            let dist = arc.center.segment_dist_sq(seg.from, seg.to);
            // Subtract 0.001 to ignore collision if the segment is tangent or nearly tangent to the arc
            if dist < arc.radius * arc.radius - 0.001 {
                return false;
            }

            // Due to allowing nearly tangent segments, cases where an endpoint
            // is slightly inside an arc are missed, so we need to check those
            if arc.center.distance_sq(seg.from) < arc.radius * arc.radius {
                return false;
            }
            if arc.center.distance_sq(seg.to) < arc.radius * arc.radius {
                return false;
            }
        }

        for segment in &self.segments {
            if let Some(_) = find_segment_intersection(segment, seg) {
                return false;
            }
        }

        return true;
    }
}
