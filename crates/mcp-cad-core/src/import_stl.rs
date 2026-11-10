//! Standard STL (Stereolithography) format importer: ASCII and Binary auto-detection.

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader, Read, Seek, SeekFrom};
use std::path::Path;
use crate::error::CadError;
use crate::topology::{TriangleMesh, Vertex};

/// Automatically detects format (Binary vs ASCII) and imports a `TriangleMesh` from a seekable stream.
///
/// Format detection rule:
/// - If stream length < 84 bytes: parsed as ASCII STL.
/// - If stream length matches $84 + N \times 50$ (where $N$ is the 32-bit triangle count at byte offset 80):
///   - If file starts with `solid` (case-insensitive) and contains no null bytes (`0x00`), it is treated as ASCII STL.
///   - Otherwise, it is parsed as Binary STL.
/// - If stream length does not match the binary formula, it is parsed as ASCII STL.
pub fn import_stl<R: Read + Seek>(reader: &mut R) -> Result<TriangleMesh, CadError> {
    let stream_len = reader.seek(SeekFrom::End(0))?;
    reader.seek(SeekFrom::Start(0))?;

    if stream_len < 84 {
        return import_stl_ascii(reader);
    }

    // Read 80-byte header and 4-byte triangle count
    let mut header = [0u8; 80];
    reader.read_exact(&mut header)?;

    let mut count_buf = [0u8; 4];
    reader.read_exact(&mut count_buf)?;
    let num_triangles = u32::from_le_bytes(count_buf) as u64;

    let expected_binary_size = 84u64.saturating_add(num_triangles.saturating_mul(50));
    let matches_binary_size = stream_len == expected_binary_size;

    let starts_with_solid = header[0..5].eq_ignore_ascii_case(b"solid");

    // Inspect first sample (up to 8192 bytes) for null bytes
    reader.seek(SeekFrom::Start(0))?;
    let sample_len = (stream_len.min(8192)) as usize;
    let mut sample_buf = vec![0u8; sample_len];
    reader.read_exact(&mut sample_buf)?;

    let has_null_bytes = sample_buf.contains(&0u8);

    // Seek back to start before delegating to format parser
    reader.seek(SeekFrom::Start(0))?;

    let is_ascii = !matches_binary_size || (starts_with_solid && !has_null_bytes);

    if is_ascii {
        import_stl_ascii(reader)
    } else {
        import_stl_binary(reader)
    }
}

/// Convenience function to import an STL file (ASCII or Binary auto-detected) directly from disk.
pub fn import_stl_file(path: impl AsRef<Path>) -> Result<TriangleMesh, CadError> {
    let mut file = File::open(path)?;
    import_stl(&mut file)
}

/// Imports a TriangleMesh from a binary STL formatted stream.
///
/// Binary STL format:
/// - 80 bytes: Arbitrary header
/// - 4 bytes: Little-endian u32 triangle count $N$
/// - For each triangle (50 bytes):
///   - 12 bytes: Normal vector (3x f32 little-endian)
///   - 12 bytes: Vertex 1 (3x f32 little-endian)
///   - 12 bytes: Vertex 2 (3x f32 little-endian)
///   - 12 bytes: Vertex 3 (3x f32 little-endian)
///   - 2 bytes: Attribute byte count (u16 little-endian)
pub fn import_stl_binary<R: Read>(reader: &mut R) -> Result<TriangleMesh, CadError> {
    let mut header = [0u8; 80];
    reader.read_exact(&mut header)?;

    let mut count_buf = [0u8; 4];
    reader.read_exact(&mut count_buf)?;
    let tri_count = u32::from_le_bytes(count_buf) as usize;

    let mut vertices = Vec::with_capacity(tri_count.saturating_mul(3));
    let mut indices = Vec::with_capacity(tri_count);

    let mut record = [0u8; 50];

    for i in 0..tri_count {
        reader.read_exact(&mut record).map_err(|e| {
            CadError::InvalidParameter(format!(
                "Unexpected end of binary STL stream at triangle {}/{}: {}",
                i, tri_count, e
            ))
        })?;

        let nx = f32::from_le_bytes([record[0], record[1], record[2], record[3]]) as f64;
        let ny = f32::from_le_bytes([record[4], record[5], record[6], record[7]]) as f64;
        let nz = f32::from_le_bytes([record[8], record[9], record[10], record[11]]) as f64;

        let v0 = [
            f32::from_le_bytes([record[12], record[13], record[14], record[15]]) as f64,
            f32::from_le_bytes([record[16], record[17], record[18], record[19]]) as f64,
            f32::from_le_bytes([record[20], record[21], record[22], record[23]]) as f64,
        ];
        let v1 = [
            f32::from_le_bytes([record[24], record[25], record[26], record[27]]) as f64,
            f32::from_le_bytes([record[28], record[29], record[30], record[31]]) as f64,
            f32::from_le_bytes([record[32], record[33], record[34], record[35]]) as f64,
        ];
        let v2 = [
            f32::from_le_bytes([record[36], record[37], record[38], record[39]]) as f64,
            f32::from_le_bytes([record[40], record[41], record[42], record[43]]) as f64,
            f32::from_le_bytes([record[44], record[45], record[46], record[47]]) as f64,
        ];

        let normal = compute_or_validate_normal(nx, ny, nz, v0, v1, v2);

        let base = vertices.len() as u32;
        vertices.push(Vertex::new(v0, normal));
        vertices.push(Vertex::new(v1, normal));
        vertices.push(Vertex::new(v2, normal));
        indices.push([base, base + 1, base + 2]);
    }

    Ok(TriangleMesh::new(vertices, indices))
}

/// Imports a TriangleMesh from an ASCII STL formatted stream.
///
/// Parses standard blocks:
/// - `solid <name>`
/// - `facet normal nx ny nz`
/// - `outer loop`
/// - `vertex x y z` (3 times)
/// - `endloop`
/// - `endfacet`
/// - `endsolid <name>`
pub fn import_stl_ascii<R: Read>(reader: &mut R) -> Result<TriangleMesh, CadError> {
    let buf_reader = BufReader::new(reader);
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    let mut current_normal: Option<[f64; 3]> = None;
    let mut current_verts: Vec<[f64; 3]> = Vec::with_capacity(3);

    for line_res in buf_reader.lines() {
        let line = line_res?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        let mut tokens = trimmed.split_ascii_whitespace();
        let first_token = match tokens.next() {
            Some(t) => t,
            None => continue,
        };

        if first_token.eq_ignore_ascii_case("facet") {
            // Flush any unfinalized facet if malformed
            flush_facet(&mut vertices, &mut indices, &mut current_verts, current_normal);

            let next_tok = tokens.next();
            if let Some(t) = next_tok {
                if t.eq_ignore_ascii_case("normal") {
                    let nx = tokens.next().and_then(|s| s.parse::<f64>().ok());
                    let ny = tokens.next().and_then(|s| s.parse::<f64>().ok());
                    let nz = tokens.next().and_then(|s| s.parse::<f64>().ok());
                    if let (Some(x), Some(y), Some(z)) = (nx, ny, nz) {
                        current_normal = Some([x, y, z]);
                    }
                } else if let Ok(x) = t.parse::<f64>() {
                    let ny = tokens.next().and_then(|s| s.parse::<f64>().ok());
                    let nz = tokens.next().and_then(|s| s.parse::<f64>().ok());
                    if let (Some(y), Some(z)) = (ny, nz) {
                        current_normal = Some([x, y, z]);
                    }
                }
            }
        } else if first_token.eq_ignore_ascii_case("vertex") {
            let x = tokens.next().and_then(|s| s.parse::<f64>().ok());
            let y = tokens.next().and_then(|s| s.parse::<f64>().ok());
            let z = tokens.next().and_then(|s| s.parse::<f64>().ok());
            if let (Some(x), Some(y), Some(z)) = (x, y, z) {
                current_verts.push([x, y, z]);
            } else {
                return Err(CadError::InvalidParameter(format!(
                    "Invalid vertex line in ASCII STL: {}",
                    trimmed
                )));
            }
        } else if first_token.eq_ignore_ascii_case("endfacet") || first_token.eq_ignore_ascii_case("endsolid") {
            flush_facet(&mut vertices, &mut indices, &mut current_verts, current_normal);
            current_normal = None;
        }
    }

    // Flush any pending trailing facet
    flush_facet(&mut vertices, &mut indices, &mut current_verts, current_normal);

    Ok(TriangleMesh::new(vertices, indices))
}

#[inline]
fn flush_facet(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<[u32; 3]>,
    current_verts: &mut Vec<[f64; 3]>,
    current_normal: Option<[f64; 3]>,
) {
    if current_verts.len() >= 3 {
        let (nx, ny, nz) = match current_normal {
            Some([x, y, z]) => (x, y, z),
            None => (0.0, 0.0, 0.0),
        };
        let v0 = current_verts[0];
        for i in 1..current_verts.len() - 1 {
            let v1 = current_verts[i];
            let v2 = current_verts[i + 1];
            let normal = compute_or_validate_normal(nx, ny, nz, v0, v1, v2);

            let base = vertices.len() as u32;
            vertices.push(Vertex::new(v0, normal));
            vertices.push(Vertex::new(v1, normal));
            vertices.push(Vertex::new(v2, normal));
            indices.push([base, base + 1, base + 2]);
        }
    }
    current_verts.clear();
}

/// Validates or computes a triangle face normal using the vector cross product rule:
/// n = (v1 - v0) x (v2 - v0).
#[inline]
pub fn compute_or_validate_normal(
    nx: f64,
    ny: f64,
    nz: f64,
    v0: [f64; 3],
    v1: [f64; 3],
    v2: [f64; 3],
) -> [f64; 3] {
    let norm_sq = nx * nx + ny * ny + nz * nz;
    if nx.is_finite() && ny.is_finite() && nz.is_finite() && norm_sq > 1e-12 {
        let len = norm_sq.sqrt();
        [nx / len, ny / len, nz / len]
    } else {
        // Face normal via (v1 - v0) x (v2 - v0)
        let d1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
        let d2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
        let cx = d1[1] * d2[2] - d1[2] * d2[1];
        let cy = d1[2] * d2[0] - d1[0] * d2[2];
        let cz = d1[0] * d2[1] - d1[1] * d2[0];
        let c_len = (cx * cx + cy * cy + cz * cz).sqrt();
        if c_len > 1e-12 {
            [cx / c_len, cy / c_len, cz / c_len]
        } else {
            [0.0, 0.0, 1.0]
        }
    }
}

/// Welds coincident vertices within a spatial tolerance.
///
/// Coincident vertex coordinates are indexed into a 3D spatial grid of cell size `tolerance`
/// and searched across all 27 neighboring cells `(-1..=1)^3` using exact Euclidean distance
/// (`||p - q|| <= tolerance`), preventing false boundary splits when vertices straddle a grid cell
/// boundary. Degenerate triangles are removed and crease-balanced vertex normals are computed.
pub fn weld_vertices(mesh: &TriangleMesh, tolerance: f64) -> TriangleMesh {
    if mesh.vertices.is_empty() {
        return TriangleMesh::empty();
    }

    let tol = if tolerance.is_finite() && tolerance > 1e-12 {
        tolerance
    } else {
        1e-6
    };
    let inv_tol = 1.0 / tol;
    let tol_sq = tol * tol;

    let mut welded_vertices: Vec<Vertex> = Vec::new();
    let mut cell_buckets: HashMap<(i64, i64, i64), Vec<u32>> =
        HashMap::with_capacity(mesh.vertices.len());
    let mut index_remap: Vec<u32> = Vec::with_capacity(mesh.vertices.len());

    for v in &mesh.vertices {
        let p = v.position;
        let cx = (p[0] * inv_tol).floor() as i64;
        let cy = (p[1] * inv_tol).floor() as i64;
        let cz = (p[2] * inv_tol).floor() as i64;

        let mut found_idx: Option<u32> = None;
        let mut best_dist_sq = tol_sq;

        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(bucket) = cell_buckets.get(&(cx + dx, cy + dy, cz + dz)) {
                        for &cand_idx in bucket {
                            let cp = welded_vertices[cand_idx as usize].position;
                            let d0 = p[0] - cp[0];
                            let d1 = p[1] - cp[1];
                            let d2 = p[2] - cp[2];
                            let dist_sq = d0 * d0 + d1 * d1 + d2 * d2;
                            if dist_sq <= best_dist_sq {
                                best_dist_sq = dist_sq;
                                found_idx = Some(cand_idx);
                            }
                        }
                    }
                }
            }
        }

        let new_idx = match found_idx {
            Some(idx) => idx,
            None => {
                let idx = welded_vertices.len() as u32;
                cell_buckets.entry((cx, cy, cz)).or_default().push(idx);
                welded_vertices.push(*v);
                idx
            }
        };
        index_remap.push(new_idx);
    }

    let has_face_ids = mesh.face_ids.len() == mesh.indices.len();
    let mut welded_indices = Vec::with_capacity(mesh.indices.len());
    let mut welded_face_ids = if has_face_ids {
        Vec::with_capacity(mesh.indices.len())
    } else {
        Vec::new()
    };

    for (t_idx, tri) in mesh.indices.iter().enumerate() {
        let i0 = index_remap[tri[0] as usize];
        let i1 = index_remap[tri[1] as usize];
        let i2 = index_remap[tri[2] as usize];
        if i0 != i1 && i1 != i2 && i0 != i2 {
            welded_indices.push([i0, i1, i2]);
            if has_face_ids {
                welded_face_ids.push(mesh.face_ids[t_idx]);
            }
        }
    }

    let mut new_mesh = if has_face_ids {
        TriangleMesh::with_faces(welded_vertices, welded_indices, welded_face_ids, mesh.faces.clone())
    } else {
        TriangleMesh::new(welded_vertices, welded_indices)
    };
    new_mesh.recompute_vertex_normals();
    new_mesh
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    use crate::diagnostics::validate_mesh;
    use crate::export_stl::{export_stl_ascii, export_stl_binary};
    use crate::primitives::{create_box, create_cylinder, create_polygon_prism};

    #[test]
    fn test_import_stl_binary_roundtrip() {
        let original = create_box(2.0, 3.0, 4.0);
        let mut buf = Vec::new();
        export_stl_binary(&original, &mut buf).unwrap();

        let mut cursor = Cursor::new(buf);
        let imported = import_stl(&mut cursor).unwrap();

        assert_eq!(imported.triangle_count(), original.triangle_count());
        let diff = (imported.volume() - original.volume()).abs();
        assert!(diff < 1e-4, "Binary STL volume mismatch: {}", diff);
    }

    #[test]
    fn test_import_stl_ascii_roundtrip() {
        let original = create_cylinder(1.5, 3.0, 32);
        let mut buf = Vec::new();
        export_stl_ascii(&original, "test_cyl", &mut buf).unwrap();

        let mut cursor = Cursor::new(buf);
        let imported = import_stl(&mut cursor).unwrap();

        assert_eq!(imported.triangle_count(), original.triangle_count());
        let ratio = (imported.volume() - original.volume()).abs() / original.volume();
        assert!(ratio < 0.001, "ASCII STL volume error ratio too high: {}", ratio);
    }

    #[test]
    fn test_import_stl_recomputes_zero_normals() {
        let ascii_data = r#"solid zero_normals
  facet normal 0.0 0.0 0.0
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 1 0
    endloop
  endfacet
endsolid zero_normals
"#;
        let mut cursor = Cursor::new(ascii_data.as_bytes());
        let mesh = import_stl(&mut cursor).unwrap();

        assert_eq!(mesh.triangle_count(), 1);
        let norm = mesh.vertices[0].normal;
        // CCW in XY plane -> normal should be +Z [0, 0, 1]
        assert!((norm[0] - 0.0).abs() < 1e-6);
        assert!((norm[1] - 0.0).abs() < 1e-6);
        assert!((norm[2] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_binary_stl_with_solid_in_header() {
        // Binary header starting with "solid"
        let original = create_box(1.0, 1.0, 1.0);
        let mut buf = Vec::new();
        export_stl_binary(&original, &mut buf).unwrap();

        // Overwrite first 5 bytes of header with "solid"
        buf[0..5].copy_from_slice(b"solid");

        let mut cursor = Cursor::new(buf);
        let imported = import_stl(&mut cursor).unwrap();

        assert_eq!(imported.triangle_count(), original.triangle_count());
        assert!((imported.volume() - original.volume()).abs() < 1e-4);
    }

    #[test]
    fn test_weld_vertices() {
        let b = create_box(1.0, 1.0, 1.0);
        // Original box has 24 vertices (flat shaded faces)
        assert_eq!(b.vertex_count(), 24);
        assert_eq!(b.triangle_count(), 12);

        let welded = weld_vertices(&b, 1e-4);
        // A cube has 8 geometric corner vertices
        assert_eq!(welded.vertex_count(), 8);
        assert_eq!(welded.triangle_count(), 12);
        assert!((welded.volume() - b.volume()).abs() < 1e-4);

        // Each of the 8 welded cube corners must have symmetric unit normal [±1/√3, ±1/√3, ±1/√3]
        let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
        for v in &welded.vertices {
            assert!((v.normal[0].abs() - inv_sqrt3).abs() < 1e-6, "Unexpected corner normal: {:?}", v.normal);
            assert!((v.normal[1].abs() - inv_sqrt3).abs() < 1e-6, "Unexpected corner normal: {:?}", v.normal);
            assert!((v.normal[2].abs() - inv_sqrt3).abs() < 1e-6, "Unexpected corner normal: {:?}", v.normal);
        }
    }

    #[test]
    fn test_weld_vertices_across_quantization_cell_boundary() {
        let tol = 1e-4;
        // Place shared edge vertices right across both the half-integer rounding boundary (0.5 * tol = 5e-5)
        // and the integer floor boundary (1.0 * tol = 1e-4), separated by only 2e-15 m (<< tol).
        let x_half_minus = 0.5 * tol - 1e-15;
        let x_half_plus = 0.5 * tol + 1e-15;
        let y_int_minus = 1.0 * tol - 1e-15;
        let y_int_plus = 1.0 * tol + 1e-15;

        let vertices = vec![
            // Triangle 1
            Vertex::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            Vertex::new([x_half_minus, y_int_minus, 0.0], [0.0, 0.0, 1.0]),
            Vertex::new([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
            // Triangle 2 sharing the vertex at (x_half, y_int, 0) and (0, 1, 0)
            Vertex::new([x_half_plus, y_int_plus, 0.0], [0.0, 0.0, 1.0]),
            Vertex::new([1.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
            Vertex::new([0.0, 1.0 + 1e-15, 0.0], [0.0, 0.0, 1.0]),
        ];
        let indices = vec![[0, 1, 2], [3, 4, 5]];
        let mesh = TriangleMesh::new(vertices, indices);

        let welded = weld_vertices(&mesh, tol);
        assert_eq!(
            welded.vertex_count(),
            4,
            "Vertices separated by 2e-15 across cell boundary must weld into 4 unique quad vertices"
        );
        assert_eq!(welded.triangle_count(), 2);
        for v in &welded.vertices {
            assert!((v.normal[2] - 1.0).abs() < 1e-9, "Flat quad normal must remain [0, 0, 1]");
        }
    }

    #[test]
    fn test_weld_vertices_thin_plate_and_prism_preserve_valid_normals() {
        // 10 x 10 x 2.0 plate: face area ratio is 100 : 20 (5:1), min angle > 11°
        let plate = create_box(10.0, 10.0, 2.0);
        let welded_plate = weld_vertices(&plate, 1e-5);
        assert_eq!(welded_plate.vertex_count(), 8);
        let diag_plate = validate_mesh(&welded_plate, None);
        assert_eq!(diag_plate.status, "PASS", "Plate issues: {:?}", diag_plate.issues);
        assert_eq!(diag_plate.geometry.normals_vs_winding_mismatches, 0);


        // Triangular prism with sharp 60-degree lateral edges and 90-degree caps
        let prism = create_polygon_prism(3, 10.0, None, Some(5.0), None);
        let welded_prism = weld_vertices(&prism, 1e-5);
        let diag_prism = validate_mesh(&welded_prism, None);
        assert_eq!(diag_prism.status, "PASS", "Prism issues: {:?}", diag_prism.issues);

        assert_eq!(diag_prism.geometry.normals_vs_winding_mismatches, 0);
    }
}
