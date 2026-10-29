//! Integration tests for mcp-cad-viewer: camera projections, headless WGPU offscreen rendering,
//! CAD solid shaders, and FEA stress colormaps.

use base64::Engine;
use nalgebra::Matrix3;

use mcp_cad_core::primitives::create_box;
use mcp_cad_viewer::camera::{CameraPreset, OrbitCamera};
use mcp_cad_viewer::offscreen::OffscreenRenderer;

const PNG_MAGIC: [u8; 8] = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];

#[test]
fn test_ortho_matrix() {
    let o = nalgebra::Orthographic3::new(-2.0, 2.0, -1.5, 1.5, -500.0, 500.0);
    let mat = mcp_cad_viewer::camera::OPENGL_TO_WGPU_MATRIX * o.to_homogeneous();
    let p_neg = mat * nalgebra::Vector4::new(0.0, 0.0, 500.0, 1.0);
    let p_pos = mat * nalgebra::Vector4::new(0.0, 0.0, -500.0, 1.0);
    println!("p_neg: {:?}, p_pos: {:?}", p_neg, p_pos);
    assert!((p_neg[2] - 0.0).abs() < 1e-4);
    assert!((p_pos[2] - 1.0).abs() < 1e-4);
}

#[test]
fn test_camera_preset_matrices() {
    let mut camera = OrbitCamera::new(
        [0.0, 0.0, 0.0],
        5.0,
        0.0,
        0.0,
        45.0f32.to_radians(),
        4.0 / 3.0,
        0.1,
        100.0,
    );

    let presets = [
        CameraPreset::Isometric,
        CameraPreset::Front,
        CameraPreset::Top,
        CameraPreset::Right,
    ];

    for preset in presets {
        camera.apply_preset(preset);

        let view = camera.view_matrix();
        let rot: Matrix3<f32> = view.fixed_view::<3, 3>(0, 0).into();

        // 1. Verify rotation submatrix orthogonality: R * R^T = I
        let identity = Matrix3::identity();
        let diff = (rot * rot.transpose() - identity).abs();
        let max_err = diff.max();
        assert!(
            max_err < 1e-4,
            "View rotation matrix for {:?} must be orthogonal, max diff = {}",
            preset,
            max_err
        );

        // 2. Verify determinant is +1.0 (pure proper rotation, right-handed)
        let det = rot.determinant();
        assert!(
            (det - 1.0).abs() < 1e-4,
            "View rotation determinant for {:?} must be 1.0, got {}",
            preset,
            det
        );

        // 3. Verify projection matrix properties
        let proj = camera.proj_matrix();
        assert!(
            proj[(3, 2)].abs() > 0.0,
            "Perspective projection must have non-zero perspective divide element"
        );
    }

    // Specific preset verification: Isometric view eye symmetry
    camera.apply_preset(CameraPreset::Isometric);
    let iso_eye = camera.eye_position();
    assert!(
        (iso_eye[0] - iso_eye[1]).abs() < 1e-4,
        "Isometric X and Y eye coordinates should match: {:?}",
        iso_eye
    );
    assert!(
        (iso_eye[1] - iso_eye[2]).abs() < 1e-4,
        "Isometric Y and Z eye coordinates should match: {:?}",
        iso_eye
    );

    // Specific preset verification: Front view eye on +Z
    camera.apply_preset(CameraPreset::Front);
    let front_eye = camera.eye_position();
    assert!(front_eye[0].abs() < 1e-5);
    assert!(front_eye[1].abs() < 1e-5);
    assert!((front_eye[2] - 5.0).abs() < 1e-5);

    // Specific preset verification: Top view eye on +Y
    camera.apply_preset(CameraPreset::Top);
    let top_eye = camera.eye_position();
    assert!(top_eye[0].abs() < 1e-5);
    assert!((top_eye[1] - 5.0).abs() < 1e-4);
    assert!(top_eye[2].abs() < 1e-4);
}

#[test]
fn test_offscreen_render_mesh_png() {
    let cube = create_box(1.0, 1.0, 1.0);
    assert!(!cube.vertices.is_empty());
    assert!(!cube.indices.is_empty());

    let renderer = match OffscreenRenderer::new() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Skipping offscreen test due to unavailable GPU adapter in environment: {e}");
            return;
        }
    };

    let width = 640;
    let height = 480;
    let png_bytes = renderer
        .render_mesh_png(&cube, CameraPreset::Isometric, width, height)
        .expect("CAD mesh PNG rendering must succeed");

    // 1. Verify buffer length and magic bytes
    assert!(!png_bytes.is_empty(), "Rendered PNG bytes must not be empty");
    assert!(png_bytes.len() > 100, "Rendered PNG file size must be non-trivial");
    assert_eq!(
        &png_bytes[0..8],
        &PNG_MAGIC,
        "First 8 bytes must match standard PNG signature"
    );

    // 2. Verify decoded image dimensions
    let img = image::load_from_memory(&png_bytes).expect("Output must be valid PNG image");
    assert_eq!(img.width(), width);
    assert_eq!(img.height(), height);

    // 3. Verify Base64 conversion
    let b64 = renderer
        .render_mesh_base64(&cube, CameraPreset::Isometric, width, height)
        .expect("Base64 mesh export must succeed");
    assert!(!b64.is_empty());
    assert!(
        b64.starts_with("iVBORw0KGgo"),
        "Base64 PNG string must start with standard PNG header encoding"
    );
}

#[test]
fn test_offscreen_fea_heatmap_png() {
    let beam = create_box(2.0, 0.4, 0.4);
    let vert_count = beam.vertices.len();

    // Generate normalized von Mises stress gradient from 0.0 (blue) to 1.0 (red)
    let stress_scalars: Vec<f32> = (0..vert_count)
        .map(|i| (i as f32) / ((vert_count - 1).max(1) as f32))
        .collect();

    let renderer = match OffscreenRenderer::new() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Skipping FEA heatmap test due to unavailable GPU adapter in environment: {e}");
            return;
        }
    };

    let width = 800;
    let height = 600;
    let png_bytes = renderer
        .render_fea_heatmap_png(&beam, &stress_scalars, CameraPreset::Front, width, height)
        .expect("FEA heatmap PNG rendering must succeed");

    assert!(!png_bytes.is_empty());
    assert!(png_bytes.len() > 100);
    assert_eq!(&png_bytes[0..8], &PNG_MAGIC);

    // Test Base64 generation and decoding roundtrip
    let b64 = renderer
        .render_fea_heatmap_base64(&beam, &stress_scalars, CameraPreset::Front, width, height)
        .expect("Base64 FEA heatmap export must succeed");

    let decoded = base64::prelude::BASE64_STANDARD
        .decode(&b64)
        .expect("Base64 string must decode successfully");

    assert_eq!(decoded, png_bytes);
}

#[test]
fn test_wireframe_index_extraction() {
    use mcp_cad_viewer::extract_wireframe_indices;

    // Two adjacent triangles sharing edge (1, 2)
    let indices = vec![[0, 1, 2], [1, 3, 2]];
    let lines = extract_wireframe_indices(&indices);

    // Edges should be: (0, 1), (1, 2), (0, 2), (1, 3), (2, 3) -> 5 unique edges, 10 indices
    assert_eq!(lines.len(), 10);

    // Box has 6 faces with duplicated vertices for flat normals (24 vertices, 12 triangles)
    // Each face has 4 outer edges + 1 diagonal = 5 edges per face * 6 faces = 30 edges (60 indices)
    let cube = create_box(1.0, 1.0, 1.0);
    let cube_lines = extract_wireframe_indices(&cube.indices);
    assert_eq!(cube_lines.len(), 60);
}

#[test]
fn test_camera_zoom_and_clipping() {
    use mcp_cad_viewer::camera::ProjectionMode;

    let mut camera = OrbitCamera::default();
    assert_eq!(camera.projection, ProjectionMode::Perspective);

    // Initial distance = 5.0
    let initial_dist = camera.distance;
    camera.zoom_by(0.85); // Zoom in
    assert!(camera.distance < initial_dist);
    assert!(camera.z_near < camera.distance);
    assert!(camera.z_far > 1000.0);

    camera.zoom_by(1.5); // Zoom out
    assert!(camera.distance > initial_dist);
    assert!(camera.z_near < camera.distance);
    assert!(camera.z_far >= 10000.0);

    // Orthographic projection matrix computation
    camera.projection = ProjectionMode::Orthographic;
    let ortho_proj = camera.proj_matrix();
    assert_eq!(ortho_proj.nrows(), 4);
    assert_eq!(ortho_proj.ncols(), 4);
}

#[test]
fn test_default_demo_mesh_creation() {
    let demo = mcp_cad_viewer::create_default_demo_mesh();
    assert!(!demo.vertices.is_empty(), "Demo mesh must contain vertices");
    assert!(!demo.indices.is_empty(), "Demo mesh must contain triangles");
    let aabb = demo.bounding_box();
    assert!(aabb.max[0] > aabb.min[0]);
    assert!(aabb.max[1] > aabb.min[1]);
    assert!(aabb.max[2] > aabb.min[2]);
    let vol = demo.volume();
    assert!(vol > 0.0, "Demo mesh must have positive volume");
}
