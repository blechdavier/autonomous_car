use std::{hint::black_box, time::Instant};

use crate::lidar::LidarScan;

pub struct PoseGraph {
    pub nodes: Vec<Node>,
}

impl PoseGraph {
    pub fn new() -> Self {
        PoseGraph { nodes: Vec::new() }
    }
    /// Add a node from the scan and odometry, and do some processing
    pub fn add_node(&mut self, scan: LidarScan) {
        if let Some(first_node) = self.nodes.first() {
            let start = Instant::now();
            // println!(
            //     "Transform: {:?} found in {:#?}",
            //     icp_least_squares(
            //         &scan.to_cartesian_points(),
            //         &first_node.scan.to_cartesian_points(),
            //         50
            //     ),
            //     start.elapsed()
            // );
        }
        self.nodes.push(Node { scan });
    }
}

pub struct Node {
    pub scan: LidarScan,
}

pub struct PositionDiff {
    pub translation: Vector2<f32>,
    pub rotation: f32,
}

pub struct HomogeneousCoordinate {
    pub x: f64,
    pub y: f64,
    pub w: f64,
}

use kd_tree::KdPoint;
use lstsq::lstsq;
use nalgebra::{Matrix2, Matrix2x3, Matrix3, Rotation2, Vector2, Vector3};
use std::f64::consts::PI;

pub fn icp_least_squares(
    p: &[Vector2<f64>],
    q: &[Vector2<f64>],
    iterations: usize,
) -> Result<Vector3<f64>, IcpError> {
    let mut x = Vector3::zeros();
    let mut p_copy = p.to_vec();
    for _ in 0..iterations {
        let correspondences = get_correspondence_indices(&p_copy, q);
        let (h, g, chi) = prepare_system(&x, p, q, &correspondences);

        let epsilon = 1e-6;
        let dx = lstsq(&h, &-g, epsilon).unwrap().solution;
        if dx.norm() < 1e-6 {
            println!("chi: {}", chi);
            return Ok(x);
        }
        x += dx;
        x[2] = x[2].rem_euclid(2.0 * PI);
        let rotation = Rotation2::new(x[2]);
        let translation = Vector2::new(x[0], x[1]);
        p_copy = p
            .iter()
            .map(|point| (rotation * point) + translation)
            .collect();
    }

    Err(IcpError::FailedToConverge)
}

#[derive(Debug)]
enum IcpError {
    FailedToConverge,
    ErrorTooHigh,
}

fn get_correspondence_indices(p: &[Vector2<f64>], q: &[Vector2<f64>]) -> Vec<usize> {
    struct CorrespondencePoint<'a> {
        point: &'a Vector2<f64>,
        index: usize,
    }
    impl KdPoint for CorrespondencePoint<'_> {
        type Scalar = f64;
        type Dim = typenum::U2;
        fn at(&self, i: usize) -> Self::Scalar {
            *self.point.get(i).unwrap()
        }
    }
    let kdtree = kd_tree::KdTree::build_by_ordered_float(
        q.iter()
            .enumerate()
            .map(|(index, point)| CorrespondencePoint { point, index })
            .collect(),
    );

    let mut correspondences = Vec::with_capacity(p.len());
    // closest point
    for i in 0..p.len() {
        // let mut min_dist = f64::MAX;
        // let mut min_index = 0;
        // for j in 0..q.len() {
        //     let dist = (p[i] - q[j]).norm_squared();
        //     if dist < min_dist {
        //         min_dist = dist;
        //         min_index = j;
        //     }
        // }
        // correspondences.push(min_index);
        let found = kdtree.nearest(&[p[i].x, p[i].y]);
        correspondences.push(found.unwrap().item.index);
    }
    correspondences
}

fn error(x: &Vector3<f64>, p_point: &Vector2<f64>, q_point: &Vector2<f64>) -> Vector2<f64> {
    let rotation = Rotation2::new(x[2]);
    let translation = Vector2::new(x[0], x[1]);
    let prediction = (rotation * p_point) + translation;
    prediction - q_point
}

fn prepare_system(
    x: &Vector3<f64>,
    p: &[Vector2<f64>],
    q: &[Vector2<f64>],
    correspondences: &Vec<usize>,
) -> (Matrix3<f64>, Vector3<f64>, f64) {
    let mut h = Matrix3::zeros();
    let mut g = Vector3::zeros();
    let mut chi = 0.0;
    for (i, j) in correspondences.iter().enumerate() {
        let p_point = p[i];
        let q_point = q[*j];
        let e = error(x, &p_point, &q_point);
        let jacobian = jacobian(&p_point);
        let transposed_jacobian = jacobian.transpose();
        h += transposed_jacobian * jacobian;
        g += transposed_jacobian * e;
        chi += (e.transpose() * e)[0];
    }
    (h, g, chi)
}

/// Returns the derivative of a rotation matrix with a given angle.
fn d_r(theta: f64) -> Matrix2<f64> {
    let mut d_r = Matrix2::zeros();
    d_r[(0, 0)] = -theta.sin();
    d_r[(0, 1)] = -theta.cos();
    d_r[(1, 0)] = theta.cos();
    d_r[(1, 1)] = -theta.sin();
    d_r
}

fn jacobian(p: &Vector2<f64>) -> Matrix2x3<f64> {
    let mut j = Matrix2x3::zeros();
    j[(0, 0)] = 1.0;
    j[(1, 1)] = 1.0;
    let dr_0 = d_r(0.0); // Not sure why this doesn't use theta but the python code doesn't either
    let dot = dr_0 * p;
    j[(0, 2)] = dot[0];
    j[(1, 2)] = dot[1];
    j
}
