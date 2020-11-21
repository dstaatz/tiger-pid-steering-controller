/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


use pid::{Pid, ControlOutput};

use rosrust_msg::std_msgs::{Float64, Header};
use rosrust_msg::nav_msgs::Path;
use rosrust_msg::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
use rustros_tf::msg::geometry_msgs::TransformStamped;


#[derive(Debug, Clone, PartialEq)]
pub struct PidConstants {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub p_limit: f64,
    pub i_limit: f64,
    pub d_limit: f64,
}

pub struct SteeringPidController {
    path: Path,
    pid: Pid<f64>,
}

impl SteeringPidController {

    /// Create a new controller
    pub fn new(path: Path, c: PidConstants) -> Self {
        let pid = Pid::new(c.kp, c.ki, c.kd, c.p_limit, c.i_limit, c.d_limit, 0.0);
        Self { path, pid}
    }

    /// Determine the Steering output based on a map->baselink tf
    pub fn update_tf(&mut self, tf: TransformStamped) -> Float64 {
        let pose = Self::convert_tf_to_pose(tf);
        self.update_pose(pose)
    }

    /// Does what it says
    fn convert_tf_to_pose(tf: TransformStamped) -> PoseStamped {
        PoseStamped {
            header: Header {
                seq: tf.header.seq,
                stamp: tf.header.stamp,
                frame_id: tf.header.frame_id,
            },
            pose: Pose {
                position: Point {
                    x: tf.transform.translation.x,
                    y: tf.transform.translation.y,
                    z: tf.transform.translation.z,
                },
                orientation: Quaternion {
                    x: tf.transform.rotation.x,
                    y: tf.transform.rotation.y,
                    z: tf.transform.rotation.z,
                    w: tf.transform.rotation.w,
                },
            }
        }
    }

    /// Determine the Steering output based on a world pose
    pub fn update_pose(&mut self, pose: PoseStamped) -> Float64 {

        let error = self.calc_error(&pose).unwrap_or(0.0);
        let output = self.pid.next_control_output(error);

        rosrust::ros_info!("(err, out, o.p, o.i, o.d): ({:.4}, {:.4}, {:.4}, {:.4}, {:.4})", error, output.output, output.p, output.i, output.d);

        Float64 { data: output.output }
    }

    /// Calulate the error of the robots location in relation to the path
    fn calc_error(&self, pose: &PoseStamped) -> Option<f64> {
        let pair = self.find_closest_pair(pose)?;
        Some(Self::calc_xte(pair, pose)?)
    }

    /// Find the consecutive pair of points with the lowest combined distance
    fn find_closest_pair(&self, pose: &PoseStamped) -> Option<(&PoseStamped, &PoseStamped)> {

        if self.path.poses.len() < 2 {
            None
        } else {

            let mut best = None;
            let count = self.path.poses.len()-1;

            // Check all consecutive pairs of point
            for i in 0..count {
                let pair = (&self.path.poses[i], &self.path.poses[i+1]);
                let distance = Self::calc_combined_distance(pair, pose);
                match best {
                    None => best = Some((pair.0, pair.1, distance)),
                    Some(x) => {
                        if distance < x.2 {
                            best = Some((pair.0, pair.1, distance));
                        }
                    }
                }
            }
            best.map(|(a,b,_)| (a,b))
        }
    }

    /// A sum of the distances in the pair to the pose
    fn calc_combined_distance(pair: (&PoseStamped, &PoseStamped), pose: &PoseStamped) -> f64 {
        Self::calc_distance_pose(pair.0, pose) + Self::calc_distance_pose(pair.1, pose)
    }

    /// Calculate the euclidean distance between two arbitrary points
    fn calc_distance_pose(pose_1: &PoseStamped, pose_2: &PoseStamped) -> f64 {
        let point_1 = (pose_1.pose.position.x, pose_1.pose.position.y);
        let point_2 = (pose_2.pose.position.x, pose_2.pose.position.y);

        Self::calc_distance_point(point_1, point_2)
    }

    /// Calculate the euclidean distance between two arbitrary points
    fn calc_distance_point(p1: (f64, f64), p2: (f64, f64)) -> f64 {
        f64::sqrt((p2.0 - p1.0).powi(2) + (p2.1 - p1.1).powi(2))
    }

    /// Calulate the cross track between of one point relative to a pair
    /// Uses model where:
    /// - point A and B of a triangle are pair.0 and pair.1 respectively
    /// - point C of a triangle is the pose
    /// - side_a, side_b, and side_c are the lengths of the side accross from their respective points
    /// - angle_a, angle_b, and angle_c are the angle of the triangle formed at their respective points
    /// 
    /// The objective is to find the height of this triangle with points A and B as the base
    fn calc_xte(pair: (&PoseStamped, &PoseStamped), pose: &PoseStamped) -> Option<f64> {
        
        // Solve for the side lengths
        let side_a = Self::calc_distance_pose(pair.1, pose);
        let side_b = Self::calc_distance_pose(pose, pair.0);
        let side_c = Self::calc_distance_pose(pair.0, pair.1);

        // Solve for the angles
        let (angle_a, _, _) = Self::solve_sss_triangle(side_a, side_b, side_c)?;

        // Determine the magnitude of the height with points A and B as the base
        let h = side_b * f64::sin(angle_a);

        let dir = if Self::direction_pose(pair, pose) { -1.0 } else { 1.0 };

        Some(dir*h)
    }

    /// Using RHR, if the point are in the order A, B, C, then the output shold be true
    /// (i.e. point C is left of point B from the perspective of point A)
    fn direction_pose(pair: (&PoseStamped, &PoseStamped), pose: &PoseStamped) -> bool {

        // Define points
        let a = (pair.0.pose.position.x, pair.0.pose.position.y);
        let b = (pair.1.pose.position.x, pair.1.pose.position.y);
        let c = (pose.pose.position.x, pose.pose.position.y);

        // Center point a at the origin
        let b = (b.0 - a.0, b.1 - a.1);
        let c = (c.0 - a.0, c.1 - a.1);

        // Find the angles that b and c make with the x-axis
        let theta_b = f64::atan2(b.1, b.0);
        let theta_c = f64::atan2(c.1, c.0);

        // Compare angles to determine the order
        // Add PI/2 to theta_b so we can reuse code from speed controller
        use std::f64::consts::{PI, FRAC_PI_2};
        const TWO_PI: f64 = 2.0 * PI;
        if theta_b + FRAC_PI_2 < PI {
            Self::direction(theta_b + FRAC_PI_2, theta_c)
        } else {
            // Overflow case
            Self::direction(theta_b + FRAC_PI_2 - TWO_PI, theta_c)
        }
    }

    /// Copied from tiger-pid-speed-controller/src/speed.rs
    fn direction(t_theta: f64, r_theta: f64) -> bool {
    
        use std::f64::consts::{PI, FRAC_PI_2};
        const TWO_PI: f64 = 2.0 * PI;
        
        if t_theta <= PI && t_theta >= -PI {
            if t_theta <= FRAC_PI_2 && t_theta >= -FRAC_PI_2 {
                (t_theta - FRAC_PI_2) <= r_theta && r_theta <= (t_theta + FRAC_PI_2)
            } else if t_theta <= FRAC_PI_2 {
                if r_theta > 0.0 {
                    r_theta >= (t_theta - FRAC_PI_2 + TWO_PI)
                } else {
                    r_theta <= (t_theta + FRAC_PI_2)
                }
            } else {
                if r_theta > 0.0 {
                    r_theta >= (t_theta - FRAC_PI_2)
                } else {
                    r_theta <= (t_theta + FRAC_PI_2 - TWO_PI)
                }
            }
        } else {
            // Error kinda
            true
        }
    }

    /// Solve the SSS triangle formed by the length of sides a, b, and c
    /// returns the angles of A, B, and C respectively in radians
    fn solve_sss_triangle(a: f64, b: f64, c: f64) -> Option<(f64, f64, f64)> {

        const MIN_SIDE_LENGTH: f64 = 1e-3;
        if a < MIN_SIDE_LENGTH || b < MIN_SIDE_LENGTH || c  < MIN_SIDE_LENGTH {
            None
        } else {

            // Solve for the angle of A using law of cosines: cos(A) = (b^2 + c^2 - a^2) / 2bc
            let cos_a = (b.powi(2) + c.powi(2) - a.powi(2)) / (2.0*b*c);
            let angle_a = f64::acos(cos_a);
    
            // Solve for the angle of B using law of cosines: cos(B) = (c^2 + a^2 - b^2) / 2ca
            let cos_b = (c.powi(2) + a.powi(2) - b.powi(2)) / (2.0*c*a);
            let angle_b = f64::acos(cos_b);
    
            // Solve for the angle of C using law of cosines: cos(C) = (a^2 + b^2 - c^2) / 2ab
            let cos_c = (a.powi(2) + b.powi(2) - c.powi(2)) / (2.0*a*b);
            let angle_c = f64::acos(cos_c);
    
            // Return
            Some((angle_a, angle_b, angle_c))
        }
    }
}


#[cfg(test)]
mod tests {

    use super::*;
    use super::SteeringPidController as crtl;

    #[test]
    fn test_direction_pose() {
        
        let v = vec![
            //a_x, a_y    b_x, b_y    c_x, c_y       ans
            ((0.0, 0.0), (0.0, 0.0), (1.0, 0.0),    true),
            ((0.0, 0.0), (0.0, 0.0), (1.0, 1.0),    true),
            ((0.0, 0.0), (0.0, 0.0), (0.0, 1.0),    true),
            ((0.0, 0.0), (0.0, 0.0), (-1.0, 1.0),   true),
            ((0.0, 0.0), (0.0, 0.0), (-1.0, 0.0),   true),
            ((0.0, 0.0), (0.0, 0.0), (-1.0, -1.0),  true),
            ((0.0, 0.0), (0.0, 0.0), (0.0, -1.0),   true),
            ((0.0, 0.0), (0.0, 0.0), (1.0, -1.0),   true),

            ((0.0, 0.0), (1.0, 0.0), (1.0, 0.0),    true),
            ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0),    true),
            ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0),    true),
            ((0.0, 0.0), (1.0, 0.0), (-1.0, 1.0),   true),
            ((0.0, 0.0), (1.0, 0.0), (-1.0, 0.0),   true),
            ((0.0, 0.0), (1.0, 0.0), (-1.0, -1.0),  false),
            ((0.0, 0.0), (1.0, 0.0), (0.0, -1.0),   false),
            ((0.0, 0.0), (1.0, 0.0), (1.0, -1.0),   false),

            ((0.0, 0.0), (1.0, 1.0), (1.0, 0.0),    false),
            ((0.0, 0.0), (1.0, 1.0), (1.0, 1.0),    true),
            ((0.0, 0.0), (1.0, 1.0), (0.0, 1.0),    true),
            ((0.0, 0.0), (1.0, 1.0), (-1.0, 1.0),   true),
            ((0.0, 0.0), (1.0, 1.0), (-1.0, 0.0),   true),
            ((0.0, 0.0), (1.0, 1.0), (-1.0, -1.0),  true),
            ((0.0, 0.0), (1.0, 1.0), (0.0, -1.0),   false),
            ((0.0, 0.0), (1.0, 1.0), (1.0, -1.0),   false),

            ((0.0, 0.0), (-1.0, 1.0), (1.0, 0.0),    false),
            ((0.0, 0.0), (-1.0, 1.0), (1.0, 1.0),    false),
            ((0.0, 0.0), (-1.0, 1.0), (0.0, 1.0),    false),
            ((0.0, 0.0), (-1.0, 1.0), (-1.0, 1.0),   true),
            ((0.0, 0.0), (-1.0, 1.0), (-1.0, 0.0),   true),
            ((0.0, 0.0), (-1.0, 1.0), (-1.0, -1.0),  true),
            ((0.0, 0.0), (-1.0, 1.0), (0.0, -1.0),   true),
            ((0.0, 0.0), (-1.0, 1.0), (1.0, -1.0),   true),

            ((0.0, 0.0), (-1.0, -1.0), (1.0, 0.0),    true),
            ((0.0, 0.0), (-1.0, -1.0), (1.0, 1.0),    true),
            ((0.0, 0.0), (-1.0, -1.0), (0.0, 1.0),    false),
            ((0.0, 0.0), (-1.0, -1.0), (-1.0, 1.0),   false),
            ((0.0, 0.0), (-1.0, -1.0), (-1.0, 0.0),   false),
            ((0.0, 0.0), (-1.0, -1.0), (-1.0, -1.0),  true),
            ((0.0, 0.0), (-1.0, -1.0), (0.0, -1.0),   true),
            ((0.0, 0.0), (-1.0, -1.0), (1.0, -1.0),   true),

            ((0.0, 0.0), (1.0, -1.0), (1.0, 0.0),    true),
            ((0.0, 0.0), (1.0, -1.0), (1.0, 1.0),    true),
            ((0.0, 0.0), (1.0, -1.0), (0.0, 1.0),    true),
            ((0.0, 0.0), (1.0, -1.0), (-1.0, 1.0),   true),
            ((0.0, 0.0), (1.0, -1.0), (-1.0, 0.0),   false),
            ((0.0, 0.0), (1.0, -1.0), (-1.0, -1.0),  false),
            ((0.0, 0.0), (1.0, -1.0), (0.0, -1.0),   false),
            ((0.0, 0.0), (1.0, -1.0), (1.0, -1.0),   true),
        ];

        let v: Vec<(PoseStamped, PoseStamped, PoseStamped, bool)> = v
            .into_iter()
            .map(|((a_x, a_y), (b_x, b_y), (c_x, c_y), ans)| {
                let mut a = PoseStamped::default();
                a.pose.position.x = a_x;
                a.pose.position.y = a_y;
                let mut b = PoseStamped::default();
                b.pose.position.x = b_x;
                b.pose.position.y = b_y;
                let mut c = PoseStamped::default();
                c.pose.position.x = c_x;
                c.pose.position.y = c_y;
                (a, b, c, ans)
            }).collect();

        for t in v {
            assert_eq!(crtl::direction_pose((&t.0, &t.1), &t.2), t.3);
        }
    }
}


