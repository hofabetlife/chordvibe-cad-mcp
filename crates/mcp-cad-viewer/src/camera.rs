//! Orbit camera implementation with preset viewpoints and WGPU uniform buffers.

use std::f32::consts::PI;
use nalgebra::{Matrix4, Point3, Vector3};
use serde::{Deserialize, Serialize};

/// WGPU standard coordinate conversion matrix.
/// WGPU uses depth range [0, 1], while standard OpenGL uses [-1, 1].
#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: Matrix4<f32> = Matrix4::new(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.5,
    0.0, 0.0, 0.0, 1.0,
);

/// Standard camera viewpoint presets used in CAD and engineering systems.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CameraPreset {
    /// Axonometric / isometric 3D view (35.26° elevation, 45° azimuth).
    Isometric,
    /// Front elevation view looking towards -Z.
    Front,
    /// Top plan view looking downwards towards -Y.
    Top,
    /// Right side view looking towards -X.
    Right,
}

/// Projection mode used by the camera.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum ProjectionMode {
    /// Standard perspective 3D projection.
    #[default]
    Perspective,
    /// Parallel orthographic projection for technical drafting and CAD blueprints.
    Orthographic,
}

/// Orbit camera for CAD viewing with target tracking, spherical angles, and projection parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct OrbitCamera {
    /// Target center of interest in 3D space [x, y, z].
    pub target: [f32; 3],
    /// Distance from the camera eye to the target.
    pub distance: f32,
    /// Azimuth angle (yaw) in radians around the Y axis.
    pub yaw: f32,
    /// Elevation angle (pitch) in radians above/below the horizontal plane.
    pub pitch: f32,
    /// Vertical Field of View (FOV) in radians.
    pub fov_y: f32,
    /// Viewport aspect ratio (width / height).
    pub aspect: f32,
    /// Near clipping plane distance.
    pub z_near: f32,
    /// Far clipping plane distance.
    pub z_far: f32,
    /// Projection mode (Perspective or Orthographic).
    pub projection: ProjectionMode,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: [0.0, 0.0, 0.0],
            distance: 5.0,
            yaw: PI / 4.0,                                    // 45 degrees
            pitch: (1.0f32 / 2.0f32.sqrt()).atan(),           // ~35.264 degrees (true isometric)
            fov_y: 45.0f32.to_radians(),
            aspect: 4.0 / 3.0,
            z_near: 0.05,
            z_far: 10000.0,
            projection: ProjectionMode::Perspective,
        }
    }
}

impl OrbitCamera {
    /// Creates a new orbit camera with custom parameters.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        target: [f32; 3],
        distance: f32,
        yaw: f32,
        pitch: f32,
        fov_y: f32,
        aspect: f32,
        z_near: f32,
        z_far: f32,
    ) -> Self {
        Self {
            target,
            distance: distance.max(0.001),
            yaw,
            pitch,
            fov_y,
            aspect: aspect.max(0.001),
            z_near: z_near.max(0.0001),
            z_far: z_far.max(z_near + 0.001),
            projection: ProjectionMode::Perspective,
        }
    }

    /// Sets the projection mode (Perspective or Orthographic).
    pub fn with_projection(mut self, projection: ProjectionMode) -> Self {
        self.projection = projection;
        self
    }

    /// Creates an orbit camera initialized to a standard CAD preset.
    pub fn from_preset(preset: CameraPreset, target: [f32; 3], distance: f32, aspect: f32) -> Self {
        let mut cam = Self {
            target,
            distance: distance.max(0.001),
            aspect: aspect.max(0.001),
            ..Default::default()
        };
        cam.apply_preset(preset);
        cam
    }

    /// Sets the camera yaw and pitch angles to match the specified CAD preset.
    pub fn apply_preset(&mut self, preset: CameraPreset) {
        match preset {
            CameraPreset::Isometric => {
                self.yaw = PI / 4.0;
                self.pitch = (1.0f32 / 2.0f32.sqrt()).atan(); // ~0.61548 rad (~35.264°)
            }
            CameraPreset::Front => {
                self.yaw = 0.0;
                self.pitch = 0.0;
            }
            CameraPreset::Top => {
                self.yaw = 0.0;
                self.pitch = PI / 2.0;
            }
            CameraPreset::Right => {
                self.yaw = PI / 2.0;
                self.pitch = 0.0;
            }
        }
    }

    /// Computes the camera eye position in 3D world space.
    pub fn eye_position(&self) -> [f32; 3] {
        let cos_p = self.pitch.cos();
        let sin_p = self.pitch.sin();
        let cos_y = self.yaw.cos();
        let sin_y = self.yaw.sin();

        [
            self.target[0] + self.distance * cos_p * sin_y,
            self.target[1] + self.distance * sin_p,
            self.target[2] + self.distance * cos_p * cos_y,
        ]
    }

    /// Computes the camera Up vector in 3D world space.
    pub fn up_vector(&self) -> Vector3<f32> {
        // If looking almost straight down or up along the Y axis, flip Up to Z
        if self.pitch >= (PI / 2.0 - 1e-4) {
            Vector3::new(0.0, 0.0, -1.0)
        } else if self.pitch <= (-PI / 2.0 + 1e-4) {
            Vector3::new(0.0, 0.0, 1.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        }
    }

    /// Computes the 4x4 View matrix.
    pub fn view_matrix(&self) -> Matrix4<f32> {
        let eye = self.eye_position();
        let eye_pt = Point3::new(eye[0], eye[1], eye[2]);
        let target_pt = Point3::new(self.target[0], self.target[1], self.target[2]);
        let up = self.up_vector();

        Matrix4::look_at_rh(&eye_pt, &target_pt, &up)
    }

    /// Computes the 4x4 Projection matrix (Perspective or Orthographic) adjusted for WGPU depth range [0, 1].
    pub fn proj_matrix(&self) -> Matrix4<f32> {
        match self.projection {
            ProjectionMode::Perspective => {
                let znear = (self.distance * 0.005).clamp(0.0001, 1.0).min(self.z_near);
                let zfar = (self.distance * 100.0).max(self.z_far).max(10000.0);
                let p = nalgebra::Perspective3::new(self.aspect, self.fov_y, znear, zfar);
                OPENGL_TO_WGPU_MATRIX * p.to_homogeneous()
            }
            ProjectionMode::Orthographic => {
                let half_h = (self.distance * (self.fov_y * 0.5).tan()).max(0.001);
                let half_w = half_h * self.aspect.max(0.001);
                let depth_range = (self.distance * 100.0).max(self.z_far).max(10000.0);
                let o = nalgebra::Orthographic3::new(-half_w, half_w, -half_h, half_h, -depth_range, depth_range);
                OPENGL_TO_WGPU_MATRIX * o.to_homogeneous()
            }
        }
    }

    /// Computes the combined View-Projection matrix: `P * V`.
    pub fn view_proj_matrix(&self) -> Matrix4<f32> {
        self.proj_matrix() * self.view_matrix()
    }

    /// Constructs the GPU uniform structure ready for binding in WGSL shaders.
    pub fn build_uniform(&self) -> CameraUniform {
        let vp = self.view_proj_matrix();
        let eye = self.eye_position();
        CameraUniform::new(&vp, eye)
    }

    /// Smoothly zooms the camera distance by a multiplicative factor.
    pub fn zoom_by(&mut self, factor: f32) {
        self.distance = (self.distance * factor).clamp(0.001, 100000.0);
        self.z_near = (self.distance * 0.005).clamp(0.0001, 0.5);
        self.z_far = (self.distance * 100.0).max(10000.0);
    }

    /// Pans the camera target across the screen/view plane.
    pub fn pan(&mut self, dx: f32, dy: f32) {
        let view = self.view_matrix();
        let right = Vector3::new(view[(0, 0)], view[(0, 1)], view[(0, 2)]);
        let up = Vector3::new(view[(1, 0)], view[(1, 1)], view[(1, 2)]);

        let scale = match self.projection {
            ProjectionMode::Perspective => self.distance * 0.0015,
            ProjectionMode::Orthographic => (self.distance * (self.fov_y * 0.5).tan() * 2.0) * 0.0015,
        };

        let delta = -dx * scale * right + dy * scale * up;
        self.target[0] += delta[0];
        self.target[1] += delta[1];
        self.target[2] += delta[2];
    }

    /// Automatically adjusts camera target, distance, and clip planes to frame a bounding box.
    pub fn fit_bounding_box(&mut self, min: [f32; 3], max: [f32; 3]) {
        self.target = [
            (min[0] + max[0]) * 0.5,
            (min[1] + max[1]) * 0.5,
            (min[2] + max[2]) * 0.5,
        ];

        let dx = (max[0] - min[0]).abs();
        let dy = (max[1] - min[1]).abs();
        let dz = (max[2] - min[2]).abs();
        let radius = (dx * dx + dy * dy + dz * dz).sqrt() * 0.5;
        let effective_radius = if radius > 1e-4 { radius } else { 1.0 };

        let half_fov = self.fov_y * 0.5;
        let dist = (effective_radius / half_fov.sin()) * 1.3;
        self.distance = dist;

        self.z_near = (dist * 0.005).clamp(0.0001, 0.5);
        self.z_far = (dist * 100.0).max(10000.0);
    }
}

/// Uniform structure sent to WGPU vertex shader containing View-Projection matrix
/// and camera position for specular lighting calculations.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    /// 4x4 View-Projection matrix in column-major layout.
    pub view_proj: [[f32; 4]; 4],
    /// Camera world-space position `[x, y, z, 1.0]` (16-byte aligned).
    pub camera_pos: [f32; 4],
}

impl CameraUniform {
    /// Creates a CameraUniform from a 4x4 matrix and 3D eye position.
    pub fn new(view_proj: &Matrix4<f32>, eye: [f32; 3]) -> Self {
        let slice = view_proj.as_slice();
        let mut vp = [[0.0f32; 4]; 4];
        for col in 0..4 {
            for row in 0..4 {
                vp[col][row] = slice[col * 4 + row];
            }
        }

        Self {
            view_proj: vp,
            camera_pos: [eye[0], eye[1], eye[2], 1.0],
        }
    }
}
