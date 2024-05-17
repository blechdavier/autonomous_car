use crate::pose_graph::PositionDiff;

const MM_PER_CLICK: f32 = 0.195364;

/// Returns the distance and angle traveled by the robot given a
pub fn odometry_diff(servo_us: u16, clicks: i32) {
    let dist = clicks as f32 * MM_PER_CLICK;
    let radians_per_5000_clicks = 0.2974285849 * servo_us as f32 - 434.3099174;
    let angle = radians_per_5000_clicks * clicks as f32 / 5000.0;
}
