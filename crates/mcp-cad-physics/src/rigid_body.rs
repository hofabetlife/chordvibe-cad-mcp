//! Rigid body and collider generation from CAD parts.

use mcp_cad_core::Part;
use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector3};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};

use crate::error::PhysicsError;

/// Supported collider approximation strategies for CAD parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ColliderType {
    /// Convex hull computed from the part's surface mesh vertices (best for dynamic bodies).
    ConvexHull,
    /// Exact triangular boundary mesh (best for static/fixed bodies and ground).
    TriMesh,
    /// Approximate compound shape composed of multiple convex hulls via convex decomposition.
    Compound,
    /// Axis-aligned bounding box cuboid primitive.
    Cube,
    /// Bounding sphere primitive enclosing the part.
    Sphere,
    /// Bounding cylinder primitive aligned along the part's major axis.
    Cylinder,
}

/// Constructs a Rapier [`RigidBodyBuilder`] for a given CAD [`Part`].
pub fn create_rigid_body_builder(
    _part: &Part,
    is_dynamic: bool,
    initial_position: Option<[f64; 3]>,
    initial_rotation: Option<[f64; 4]>,
) -> RigidBodyBuilder {
    let mut builder = if is_dynamic {
        RigidBodyBuilder::dynamic()
    } else {
        RigidBodyBuilder::fixed()
    };

    if let Some([x, y, z]) = initial_position {
        builder = builder.translation(vector![x as Real, y as Real, z as Real]);
    }

    if let Some([x, y, z, w]) = initial_rotation {
        let quat = Quaternion::new(w as Real, x as Real, y as Real, z as Real);
        let unit_quat = UnitQuaternion::from_quaternion(quat);
        builder = builder.rotation(unit_quat.scaled_axis());
    }

    builder
}

/// Constructs a Rapier [`ColliderBuilder`] for a given CAD [`Part`] and collider type.
///
/// Uses the physical mass, volume, and material density calculated from the CAD model.
pub fn create_collider_builder(
    part: &Part,
    collider_type: ColliderType,
) -> Result<ColliderBuilder, PhysicsError> {
    if part.mesh.vertices.is_empty() {
        return Err(PhysicsError::InvalidGeometry(
            part.id.clone(),
            "Part mesh is empty. Rebuild the part DAG first.".into(),
        ));
    }

    let mut builder = match collider_type {
        ColliderType::ConvexHull => {
            let points: Vec<Point<Real>> = part
                .mesh
                .vertices
                .iter()
                .map(|v| point![v.position[0] as Real, v.position[1] as Real, v.position[2] as Real])
                .collect();

            if points.len() < 4 {
                return Err(PhysicsError::ColliderCreationFailed(
                    part.id.clone(),
                    "At least 4 vertices required for convex hull collider".into(),
                ));
            }

            ColliderBuilder::convex_hull(&points).ok_or_else(|| {
                PhysicsError::ColliderCreationFailed(
                    part.id.clone(),
                    "Convex hull computation failed: points may be coplanar or collinear".into(),
                )
            })?
        }
        ColliderType::TriMesh => {
            let points: Vec<Point<Real>> = part
                .mesh
                .vertices
                .iter()
                .map(|v| point![v.position[0] as Real, v.position[1] as Real, v.position[2] as Real])
                .collect();
            let indices: Vec<[u32; 3]> = part.mesh.indices.clone();

            if indices.is_empty() {
                return Err(PhysicsError::ColliderCreationFailed(
                    part.id.clone(),
                    "TriMesh collider requires at least one triangle index face".into(),
                ));
            }

            ColliderBuilder::trimesh(points, indices)
        }
        ColliderType::Compound => {
            let points: Vec<Point<Real>> = part
                .mesh
                .vertices
                .iter()
                .map(|v| point![v.position[0] as Real, v.position[1] as Real, v.position[2] as Real])
                .collect();
            let indices: Vec<[u32; 3]> = part.mesh.indices.clone();

            if points.len() < 4 || indices.is_empty() {
                return Err(PhysicsError::ColliderCreationFailed(
                    part.id.clone(),
                    "Compound decomposition requires vertices and faces".into(),
                ));
            }

            let params = rapier3d::parry::transformation::vhacd::VHACDParameters {
                resolution: 10,
                max_convex_hulls: 4,
                concavity: 0.05,
                plane_downsampling: 4,
                convex_hull_downsampling: 4,
                ..Default::default()
            };
            ColliderBuilder::convex_decomposition_with_params(&points, &indices, &params)
        }
        ColliderType::Cube => {
            let aabb = part.bounding_box();
            let dims = aabb.dimensions();
            let hx = (dims[0] * 0.5).max(1e-4) as Real;
            let hy = (dims[1] * 0.5).max(1e-4) as Real;
            let hz = (dims[2] * 0.5).max(1e-4) as Real;
            let center = aabb.center();

            ColliderBuilder::cuboid(hx, hy, hz).translation(vector![
                center[0] as Real,
                center[1] as Real,
                center[2] as Real
            ])
        }
        ColliderType::Sphere => {
            let aabb = part.bounding_box();
            let dims = aabb.dimensions();
            let radius = (dims[0].max(dims[1]).max(dims[2]) * 0.5).max(1e-4) as Real;
            let center = aabb.center();

            ColliderBuilder::ball(radius).translation(vector![
                center[0] as Real,
                center[1] as Real,
                center[2] as Real
            ])
        }
        ColliderType::Cylinder => {
            let aabb = part.bounding_box();
            let dims = aabb.dimensions();
            let center = aabb.center();

            if dims[2] > dims[0] && dims[2] > dims[1] {
                // Aligned along Z-axis (default for mcp-cad-core cylinders)
                let radius = (dims[0].max(dims[1]) * 0.5).max(1e-4) as Real;
                let half_height = (dims[2] * 0.5).max(1e-4) as Real;
                // Rotate by 90 degrees around X to align Rapier's Y-cylinder with Z
                let rot = Rotation3::from_axis_angle(
                    &Vector3::x_axis(),
                    std::f32::consts::FRAC_PI_2,
                );
                ColliderBuilder::cylinder(half_height, radius)
                    .rotation(rot.scaled_axis())
                    .translation(vector![
                        center[0] as Real,
                        center[1] as Real,
                        center[2] as Real
                    ])
            } else if dims[0] > dims[1] && dims[0] > dims[2] {
                // Aligned along X-axis
                let radius = (dims[1].max(dims[2]) * 0.5).max(1e-4) as Real;
                let half_height = (dims[0] * 0.5).max(1e-4) as Real;
                let rot = Rotation3::from_axis_angle(
                    &Vector3::z_axis(),
                    std::f32::consts::FRAC_PI_2,
                );
                ColliderBuilder::cylinder(half_height, radius)
                    .rotation(rot.scaled_axis())
                    .translation(vector![
                        center[0] as Real,
                        center[1] as Real,
                        center[2] as Real
                    ])
            } else {
                // Aligned along Y-axis (Rapier default)
                let radius = (dims[0].max(dims[2]) * 0.5).max(1e-4) as Real;
                let half_height = (dims[1] * 0.5).max(1e-4) as Real;
                ColliderBuilder::cylinder(half_height, radius).translation(vector![
                    center[0] as Real,
                    center[1] as Real,
                    center[2] as Real
                ])
            }
        }
    };

    // Apply material density from Part
    let density = part.material.density_kg_m3;
    if density > 0.0 {
        builder = builder.density(density as Real);
    }

    // Apply exact computed mass from Part if available
    let mass = part.mass();
    if mass > 0.0 {
        builder = builder.mass(mass as Real);
    }

    // Activate collision and contact force events for telemetry logging
    builder = builder.active_events(
        ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS,
    );

    Ok(builder)
}
