//! Parametric Feature Tree (DAG) for solid modeling history and re-evaluation.

use std::time::{SystemTime, UNIX_EPOCH};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::csg::csg_union;
use crate::error::CadError;
use crate::primitives::{create_box, create_cylinder, create_sphere, cut_hole};
use crate::topology::TriangleMesh;

/// Individual modeling feature operation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FeatureOp {
    /// Add a box centered at the origin.
    AddBox {
        width: f64,
        height: f64,
        depth: f64,
    },
    /// Add a cylinder centered along Z.
    AddCylinder {
        radius: f64,
        height: f64,
        #[serde(default = "default_segments")]
        segments: usize,
    },
    /// Add a UV sphere centered at the origin.
    AddSphere {
        radius: f64,
        #[serde(default = "default_sectors")]
        sectors: usize,
        #[serde(default = "default_stacks")]
        stacks: usize,
    },
    /// Cut a cylindrical hole from the current solid.
    CutHole {
        center: [f64; 3],
        normal: [f64; 3],
        radius: f64,
        depth: f64,
    },
    /// Apply spatial translation, rotation (RPY degrees), and scale factors.
    Transform {
        #[serde(default)]
        translation: [f64; 3],
        #[serde(default)]
        rotation_deg: [f64; 3],
        #[serde(default = "default_scale")]
        scale: [f64; 3],
    },
}

fn default_segments() -> usize {
    32
}

fn default_sectors() -> usize {
    32
}

fn default_stacks() -> usize {
    16
}

fn default_scale() -> [f64; 3] {
    [1.0, 1.0, 1.0]
}

/// A node in the parametric history tree.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeatureNode {
    /// Unique identifier for this feature node.
    pub id: Uuid,
    /// User-friendly label or name of the operation.
    pub name: String,
    /// The operation and its geometric parameters.
    pub op: FeatureOp,
    /// Creation timestamp (Unix epoch seconds).
    pub created_at: u64,
    /// Whether this feature is currently active during DAG evaluation.
    pub enabled: bool,
}

impl FeatureNode {
    /// Creates a new feature node with an auto-generated UUID v4.
    pub fn new(name: impl Into<String>, op: FeatureOp) -> Self {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);

        Self {
            id: Uuid::new_v4(),
            name: name.into(),
            op,
            created_at: now,
            enabled: true,
        }
    }
}

/// Parametric history DAG (Feature Tree) of a CAD part.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FeatureTree {
    /// Sequential list of parametric operations.
    pub nodes: Vec<FeatureNode>,
}

impl FeatureTree {
    /// Creates a new empty feature tree.
    pub fn new() -> Self {
        Self { nodes: Vec::new() }
    }

    /// Appends a new feature operation to the history tree, returning its unique ID.
    pub fn add_feature(&mut self, op: FeatureOp, name: Option<String>) -> Uuid {
        let default_name = match &op {
            FeatureOp::AddBox { .. } => "Box".to_string(),
            FeatureOp::AddCylinder { .. } => "Cylinder".to_string(),
            FeatureOp::AddSphere { .. } => "Sphere".to_string(),
            FeatureOp::CutHole { .. } => "Cut Hole".to_string(),
            FeatureOp::Transform { .. } => "Transform".to_string(),
        };
        let node_name = name.unwrap_or(default_name);
        let node = FeatureNode::new(node_name, op);
        let id = node.id;
        self.nodes.push(node);
        id
    }

    /// Modifies an existing feature operation by ID without altering other history steps.
    pub fn modify_feature(&mut self, id: Uuid, new_op: FeatureOp) -> Result<(), CadError> {
        let node = self
            .nodes
            .iter_mut()
            .find(|n| n.id == id)
            .ok_or(CadError::FeatureNotFound(id))?;
        node.op = new_op;
        Ok(())
    }

    /// Removes a feature node from the tree by ID.
    pub fn remove_feature(&mut self, id: Uuid) -> Result<FeatureNode, CadError> {
        let index = self
            .nodes
            .iter()
            .position(|n| n.id == id)
            .ok_or(CadError::FeatureNotFound(id))?;
        Ok(self.nodes.remove(index))
    }

    /// Finds a feature node by ID.
    pub fn get_feature(&self, id: Uuid) -> Option<&FeatureNode> {
        self.nodes.iter().find(|n| n.id == id)
    }

    /// Finds a feature node by ID (mutable).
    pub fn get_feature_mut(&mut self, id: Uuid) -> Option<&mut FeatureNode> {
        self.nodes.iter_mut().find(|n| n.id == id)
    }

    /// Enables or disables a feature step.
    pub fn set_feature_enabled(&mut self, id: Uuid, enabled: bool) -> Result<(), CadError> {
        let node = self.get_feature_mut(id).ok_or(CadError::FeatureNotFound(id))?;
        node.enabled = enabled;
        Ok(())
    }

    /// Evaluates the complete feature history tree and returns the resulting 3D boundary mesh.
    pub fn evaluate(&self) -> Result<TriangleMesh, CadError> {
        let mut current_mesh: Option<TriangleMesh> = None;

        for node in &self.nodes {
            if !node.enabled {
                continue;
            }

            match &node.op {
                FeatureOp::AddBox { width, height, depth } => {
                    if *width <= 0.0 || *height <= 0.0 || *depth <= 0.0 {
                        return Err(CadError::InvalidParameter(format!(
                            "Box dimensions must be positive, got: {}x{}x{}",
                            width, height, depth
                        )));
                    }
                    let box_mesh = create_box(*width, *height, *depth);
                    current_mesh = match current_mesh {
                        Some(m) => Some(csg_union(&m, &box_mesh)),
                        None => Some(box_mesh),
                    };
                }
                FeatureOp::AddCylinder { radius, height, segments } => {
                    if *radius <= 0.0 || *height <= 0.0 {
                        return Err(CadError::InvalidParameter(format!(
                            "Cylinder radius and height must be positive, got: r={}, h={}",
                            radius, height
                        )));
                    }
                    let cyl_mesh = create_cylinder(*radius, *height, *segments);
                    current_mesh = match current_mesh {
                        Some(m) => Some(csg_union(&m, &cyl_mesh)),
                        None => Some(cyl_mesh),
                    };
                }
                FeatureOp::AddSphere { radius, sectors, stacks } => {
                    if *radius <= 0.0 {
                        return Err(CadError::InvalidParameter(format!(
                            "Sphere radius must be positive, got: r={}",
                            radius
                        )));
                    }
                    let sph_mesh = create_sphere(*radius, *sectors, *stacks);
                    current_mesh = match current_mesh {
                        Some(m) => Some(csg_union(&m, &sph_mesh)),
                        None => Some(sph_mesh),
                    };
                }
                FeatureOp::CutHole { center, normal, radius, depth } => {
                    if *radius <= 0.0 || *depth <= 0.0 {
                        return Err(CadError::InvalidParameter(format!(
                            "Cut hole radius and depth must be positive, got: r={}, d={}",
                            radius, depth
                        )));
                    }
                    let mesh = current_mesh.ok_or_else(|| {
                        CadError::EvaluationError("Cannot cut hole in empty solid".into())
                    })?;
                    current_mesh = Some(cut_hole(&mesh, *center, *normal, *radius, *depth));
                }
                FeatureOp::Transform { translation, rotation_deg, scale } => {
                    let mut mesh = current_mesh.ok_or_else(|| {
                        CadError::EvaluationError("Cannot transform empty solid".into())
                    })?;
                    if scale[0] == 0.0 || scale[1] == 0.0 || scale[2] == 0.0 {
                        return Err(CadError::InvalidParameter("Scale components cannot be zero".into()));
                    }
                    mesh.scale(*scale);
                    mesh.rotate_rpy_deg(rotation_deg[0], rotation_deg[1], rotation_deg[2]);
                    mesh.translate(*translation);
                    current_mesh = Some(mesh);
                }
            }
        }

        Ok(current_mesh.unwrap_or_else(TriangleMesh::empty))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tree_add_modify_evaluate() {
        let mut tree = FeatureTree::new();
        let box_id = tree.add_feature(
            FeatureOp::AddBox {
                width: 10.0,
                height: 10.0,
                depth: 10.0,
            },
            Some("Base Cube".into()),
        );

        let mesh1 = tree.evaluate().unwrap();
        assert!((mesh1.volume() - 1000.0).abs() < 1e-4);

        // Modify height to 20.0
        tree.modify_feature(
            box_id,
            FeatureOp::AddBox {
                width: 10.0,
                height: 20.0,
                depth: 10.0,
            },
        ).unwrap();

        let mesh2 = tree.evaluate().unwrap();
        assert!((mesh2.volume() - 2000.0).abs() < 1e-4);
    }
}
