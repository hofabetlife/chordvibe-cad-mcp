//! Core geometric types and engine functionality for MCP-CAD.
#![deny(unsafe_code)]

pub mod error;
pub mod material;
pub mod topology;
pub mod primitives;
pub mod csg;
pub mod dag;
pub mod part;
pub mod export_stl;

pub use error::CadError;
pub use material::{Material, MaterialRegistry};
pub use topology::{BoundingBox, TriangleMesh, Vertex};
pub use primitives::{create_box, create_cone, create_cylinder, create_helical_cutter, create_sphere, cut_hole};
pub use dag::{FeatureNode, FeatureOp, FeatureTree};
pub use part::Part;
pub use export_stl::{export_stl_ascii, export_stl_binary, write_stl_ascii_file, write_stl_binary_file};
