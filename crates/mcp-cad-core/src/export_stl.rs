//! Standard STL (Stereolithography) format exporter: ASCII and Binary.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use crate::error::CadError;
use crate::topology::TriangleMesh;

/// Exports a TriangleMesh to an ASCII STL formatted stream.
pub fn export_stl_ascii<W: Write>(
    mesh: &TriangleMesh,
    solid_name: &str,
    writer: &mut W,
) -> Result<(), CadError> {
    let name = if solid_name.trim().is_empty() {
        "mcp_cad_solid"
    } else {
        solid_name.trim()
    };

    writeln!(writer, "solid {}", name)?;

    for tri in &mesh.indices {
        let v0 = &mesh.vertices[tri[0] as usize];
        let v1 = &mesh.vertices[tri[1] as usize];
        let v2 = &mesh.vertices[tri[2] as usize];

        // Compute face normal from triangle vertices
        let p0 = v0.pos_vec();
        let p1 = v1.pos_vec();
        let p2 = v2.pos_vec();
        let norm = (p1 - p0).cross(&(p2 - p0));
        let len = norm.norm();
        let [nx, ny, nz] = if len > 1e-12 {
            let u = norm / len;
            [u.x, u.y, u.z]
        } else {
            v0.normal
        };

        writeln!(writer, "  facet normal {:e} {:e} {:e}", nx, ny, nz)?;
        writeln!(writer, "    outer loop")?;
        writeln!(writer, "      vertex {:e} {:e} {:e}", v0.position[0], v0.position[1], v0.position[2])?;
        writeln!(writer, "      vertex {:e} {:e} {:e}", v1.position[0], v1.position[1], v1.position[2])?;
        writeln!(writer, "      vertex {:e} {:e} {:e}", v2.position[0], v2.position[1], v2.position[2])?;
        writeln!(writer, "    endloop")?;
        writeln!(writer, "  endfacet")?;
    }

    writeln!(writer, "endsolid {}", name)?;
    writer.flush()?;
    Ok(())
}

/// Exports a TriangleMesh to a compact Binary STL formatted stream.
///
/// Binary STL format:
/// - 80 bytes: Header string/bytes
/// - 4 bytes: Little-endian u32 triangle count
/// - For each triangle (50 bytes):
///   - 12 bytes: Normal vector (3x f32 little-endian)
///   - 12 bytes: Vertex 1 (3x f32 little-endian)
///   - 12 bytes: Vertex 2 (3x f32 little-endian)
///   - 12 bytes: Vertex 3 (3x f32 little-endian)
///   - 2 bytes: Attribute byte count (u16 little-endian = 0)
pub fn export_stl_binary<W: Write>(
    mesh: &TriangleMesh,
    writer: &mut W,
) -> Result<(), CadError> {
    // 80-byte header
    let mut header = [0u8; 80];
    let title = b"MCP-CAD Engine Binary STL Model";
    let len = title.len().min(80);
    header[..len].copy_from_slice(&title[..len]);
    writer.write_all(&header)?;

    // 4-byte triangle count
    let tri_count = mesh.indices.len() as u32;
    writer.write_all(&tri_count.to_le_bytes())?;

    for tri in &mesh.indices {
        let v0 = &mesh.vertices[tri[0] as usize];
        let v1 = &mesh.vertices[tri[1] as usize];
        let v2 = &mesh.vertices[tri[2] as usize];

        let p0 = v0.pos_vec();
        let p1 = v1.pos_vec();
        let p2 = v2.pos_vec();
        let norm = (p1 - p0).cross(&(p2 - p0));
        let norm_len = norm.norm();
        let [nx, ny, nz] = if norm_len > 1e-12 {
            let u = norm / norm_len;
            [u.x as f32, u.y as f32, u.z as f32]
        } else {
            [v0.normal[0] as f32, v0.normal[1] as f32, v0.normal[2] as f32]
        };

        // Normal
        writer.write_all(&nx.to_le_bytes())?;
        writer.write_all(&ny.to_le_bytes())?;
        writer.write_all(&nz.to_le_bytes())?;

        // Vertex 1
        writer.write_all(&(v0.position[0] as f32).to_le_bytes())?;
        writer.write_all(&(v0.position[1] as f32).to_le_bytes())?;
        writer.write_all(&(v0.position[2] as f32).to_le_bytes())?;

        // Vertex 2
        writer.write_all(&(v1.position[0] as f32).to_le_bytes())?;
        writer.write_all(&(v1.position[1] as f32).to_le_bytes())?;
        writer.write_all(&(v1.position[2] as f32).to_le_bytes())?;

        // Vertex 3
        writer.write_all(&(v2.position[0] as f32).to_le_bytes())?;
        writer.write_all(&(v2.position[1] as f32).to_le_bytes())?;
        writer.write_all(&(v2.position[2] as f32).to_le_bytes())?;

        // Attribute byte count (u16 = 0)
        writer.write_all(&0u16.to_le_bytes())?;
    }

    writer.flush()?;
    Ok(())
}

/// Convenience function to write an ASCII STL file directly to disk.
pub fn write_stl_ascii_file(
    mesh: &TriangleMesh,
    solid_name: &str,
    path: impl AsRef<Path>,
) -> Result<(), CadError> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    export_stl_ascii(mesh, solid_name, &mut writer)
}

/// Convenience function to write a Binary STL file directly to disk.
pub fn write_stl_binary_file(
    mesh: &TriangleMesh,
    path: impl AsRef<Path>,
) -> Result<(), CadError> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    export_stl_binary(mesh, &mut writer)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::create_box;

    #[test]
    fn test_export_stl_ascii() {
        let b = create_box(1.0, 1.0, 1.0);
        let mut buffer = Vec::new();
        export_stl_ascii(&b, "test_cube", &mut buffer).unwrap();

        let text = String::from_utf8(buffer).unwrap();
        assert!(text.starts_with("solid test_cube"));
        assert!(text.trim_end().ends_with("endsolid test_cube"));
        assert!(text.contains("facet normal"));
        assert!(text.contains("outer loop"));
        assert!(text.contains("vertex"));
        assert!(text.contains("endloop"));
        assert!(text.contains("endfacet"));
    }

    #[test]
    fn test_export_stl_binary() {
        let b = create_box(2.0, 3.0, 4.0);
        let mut buffer = Vec::new();
        export_stl_binary(&b, &mut buffer).unwrap();

        // 80 bytes header + 4 bytes tri count + 12 triangles * 50 bytes = 684 bytes
        let expected_size = 80 + 4 + 12 * 50;
        assert_eq!(buffer.len(), expected_size);

        // Header check
        assert!(&buffer[..31] == b"MCP-CAD Engine Binary STL Model");

        // Triangle count check
        let tri_count = u32::from_le_bytes([buffer[80], buffer[81], buffer[82], buffer[83]]);
        assert_eq!(tri_count, 12);
    }
}
