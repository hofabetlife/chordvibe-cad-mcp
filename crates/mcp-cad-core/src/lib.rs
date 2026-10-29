//! Core geometric types and engine functionality for MCP-CAD.
#![deny(unsafe_code)]

pub mod error;
pub mod material;
pub mod tessellation;
pub mod topology;
pub mod primitives;
pub mod csg;
pub mod dag;
pub mod part;
pub mod export_stl;
pub mod import_stl;

pub use error::CadError;
pub use material::{Material, MaterialRegistry};
pub use tessellation::{TessellationQuality, TessellationSettings};
pub use topology::{BoundingBox, GeometryInspection, PlanarFaceInfo, TriangleMesh, Vertex};
pub use primitives::{
    create_box, create_cone, create_cone_with_settings, create_cylinder,
    create_cylinder_oriented, create_cylinder_oriented_with_settings,
    create_cylinder_with_settings, create_helical_cutter,
    create_helical_cutter_with_settings, create_polygon_prism, create_revolve,
    create_revolve_with_settings, create_sphere, create_sphere_with_settings,
    create_spur_gear, create_spur_gear_with_settings, create_wheel_rim,
    create_wheel_rim_with_settings, cut_hole, cut_hole_with_settings,
};
pub use dag::{FeatureNode, FeatureOp, FeatureTree};
pub use part::Part;
pub use export_stl::{
    export_part_stl_with_quality, export_stl_ascii, export_stl_binary,
    write_part_stl_with_quality, write_stl_ascii_file, write_stl_binary_file,
};
pub use import_stl::{import_stl, import_stl_ascii, import_stl_binary, import_stl_file, weld_vertices};
