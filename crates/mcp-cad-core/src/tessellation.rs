//! Adaptive tessellation control for CAD geometry discretization.
//!
//! Provides chordal error (sagitta) and angular tolerance settings
//! for lazy on-demand polygonization of analytical and parametric surfaces.

use std::f64::consts::PI;
use serde::{Deserialize, Serialize};

/// Tessellation quality preset or custom tolerance configuration.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub enum TessellationQuality {
    /// Fast coarse tessellation suitable for real-time interactive viewports.
    /// Chord tolerance ~ 0.5 mm, angular tolerance ~ 15-20°.
    Coarse,
    /// Balanced tessellation suitable for modeling, FEA meshing, and physics.
    /// Chord tolerance ~ 0.1 mm, angular tolerance ~ 10-11°.
    #[default]
    Medium,
    /// High-precision tessellation suitable for STL export, CAM, and 3D printing.
    /// Chord tolerance ~ 0.02 mm, angular tolerance ~ 3-5°.
    Fine,
    /// User-defined explicit tolerances.
    Custom {
        /// Maximum allowed chordal deviation (sagitta distance) in meters (or model units).
        chord_tolerance: f64,
        /// Maximum allowed angular deflection between adjacent chord segments in degrees.
        angular_tolerance_deg: f64,
        /// Optional maximum chord edge length.
        max_edge_length: Option<f64>,
    },
}

/// Dynamic tessellation parameters used to calculate surface discretization steps.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TessellationSettings {
    /// Quality preset mode.
    pub quality: TessellationQuality,
    /// Maximum allowed chordal deviation (sagitta distance) $\delta$ in meters (or model units).
    pub chord_tolerance: f64,
    /// Maximum angular deflection between adjacent segments in degrees.
    pub angular_tolerance_deg: f64,
    /// Minimum number of segments for circular profiles.
    pub min_segments: usize,
    /// Maximum number of segments allowed (prevents polygon explosion).
    pub max_segments: usize,
    /// Optional maximum edge length constraint.
    pub max_edge_length: Option<f64>,
}

impl Default for TessellationSettings {
    fn default() -> Self {
        Self::medium()
    }
}

impl TessellationSettings {
    /// Creates coarse settings for interactive preview (~0.5 mm chord tolerance, ~20° angular tolerance).
    pub fn coarse() -> Self {
        Self {
            quality: TessellationQuality::Coarse,
            chord_tolerance: 0.0005,
            angular_tolerance_deg: 20.0,
            min_segments: 8,
            max_segments: 64,
            max_edge_length: None,
        }
    }

    /// Creates standard balanced settings (~0.1 mm chord tolerance, ~11.25° angular tolerance).
    pub fn medium() -> Self {
        Self {
            quality: TessellationQuality::Medium,
            chord_tolerance: 0.0001,
            angular_tolerance_deg: 11.25,
            min_segments: 16,
            max_segments: 128,
            max_edge_length: None,
        }
    }

    /// Creates high-accuracy settings for STL export / 3D printing (~0.02 mm chord tolerance, ~4° angular tolerance).
    pub fn fine() -> Self {
        Self {
            quality: TessellationQuality::Fine,
            chord_tolerance: 0.00002,
            angular_tolerance_deg: 4.0,
            min_segments: 32,
            max_segments: 512,
            max_edge_length: None,
        }
    }

    /// Creates custom tessellation settings with explicit tolerances.
    pub fn custom(
        chord_tolerance: f64,
        angular_tolerance_deg: f64,
        max_edge_length: Option<f64>,
    ) -> Self {
        Self {
            quality: TessellationQuality::Custom {
                chord_tolerance,
                angular_tolerance_deg,
                max_edge_length,
            },
            chord_tolerance,
            angular_tolerance_deg,
            min_segments: 8,
            max_segments: 1024,
            max_edge_length,
        }
    }

    /// Builder: modifies chord tolerance.
    pub fn with_chord_tolerance(mut self, tolerance: f64) -> Self {
        self.chord_tolerance = tolerance;
        self
    }

    /// Builder: modifies angular tolerance in degrees.
    pub fn with_angular_tolerance_deg(mut self, deg: f64) -> Self {
        self.angular_tolerance_deg = deg;
        self
    }

    /// Builder: modifies minimum number of segments.
    pub fn with_min_segments(mut self, min: usize) -> Self {
        self.min_segments = min;
        self
    }

    /// Builder: modifies maximum number of segments.
    pub fn with_max_segments(mut self, max: usize) -> Self {
        self.max_segments = max;
        self
    }

    /// Builder: modifies maximum edge length constraint.
    pub fn with_max_edge_length(mut self, max_len: Option<f64>) -> Self {
        self.max_edge_length = max_len;
        self
    }

    /// Calculates required number of segments for a full circle of radius $R$
    /// satisfying both chordal error $\delta$ and angular tolerance $\theta_{\text{max}}$:
    ///
    /// $$N_{\text{segments}} = \max\left(N_{\min}, \left\lceil \frac{\pi}{\arccos\left(1 - \frac{\delta}{R}\right)} \right\rceil\right)$$
    pub fn calculate_segments(&self, radius: f64) -> usize {
        if radius <= 1e-12 {
            return self.min_segments;
        }

        let n_chord = if self.chord_tolerance > 0.0 {
            let ratio = (1.0 - (self.chord_tolerance / radius)).clamp(-1.0, 1.0 - 1e-12);
            let angle = ratio.acos();
            if angle > 1e-12 {
                (PI / angle).ceil() as usize
            } else {
                self.max_segments
            }
        } else {
            self.max_segments
        };

        let n_angular = if self.angular_tolerance_deg > 0.0 {
            (360.0 / self.angular_tolerance_deg).ceil() as usize
        } else {
            self.max_segments
        };

        let mut n = n_chord.max(n_angular);

        if let Some(max_edge_len) = self.max_edge_length {
            if max_edge_len > 1e-12 {
                let n_edge = (2.0 * PI * radius / max_edge_len).ceil() as usize;
                n = n.max(n_edge);
            }
        }

        n.clamp(self.min_segments, self.max_segments)
    }

    /// Calculates required number of segments for a circular arc of given angle (in radians).
    pub fn calculate_segments_arc(&self, radius: f64, angle_rad: f64) -> usize {
        let full_segments = self.calculate_segments(radius);
        let fraction = (angle_rad.abs() / (2.0 * PI)).clamp(0.0, 1.0);
        let arc_segments = (full_segments as f64 * fraction).ceil() as usize;
        let min_arc = ((self.min_segments as f64) * fraction).ceil() as usize;
        arc_segments.max(min_arc.max(4)).min(self.max_segments)
    }

    /// Calculates (sectors, stacks) resolution for UV sphere tessellation.
    pub fn calculate_sphere_resolution(&self, radius: f64) -> (usize, usize) {
        let sectors = self.calculate_segments(radius);
        let stacks = (sectors / 2).max(self.min_segments / 2).max(4);
        (sectors, stacks)
    }
}

impl From<TessellationQuality> for TessellationSettings {
    fn from(q: TessellationQuality) -> Self {
        match q {
            TessellationQuality::Coarse => Self::coarse(),
            TessellationQuality::Medium => Self::medium(),
            TessellationQuality::Fine => Self::fine(),
            TessellationQuality::Custom {
                chord_tolerance,
                angular_tolerance_deg,
                max_edge_length,
            } => Self::custom(chord_tolerance, angular_tolerance_deg, max_edge_length),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_presets_order_of_precision() {
        let coarse = TessellationSettings::coarse();
        let medium = TessellationSettings::medium();
        let fine = TessellationSettings::fine();

        let r = 0.05; // 50 mm radius
        let n_coarse = coarse.calculate_segments(r);
        let n_medium = medium.calculate_segments(r);
        let n_fine = fine.calculate_segments(r);

        assert!(
            n_fine > n_medium,
            "Fine (N={}) must produce more segments than Medium (N={})",
            n_fine,
            n_medium
        );
        assert!(
            n_medium >= n_coarse,
            "Medium (N={}) must produce >= segments than Coarse (N={})",
            n_medium,
            n_coarse
        );
    }

    #[test]
    fn test_radius_scaling_effect() {
        let settings = TessellationSettings::medium();
        // Larger radius requires more segments to maintain chordal error delta
        let r_small = 0.01;
        let r_large = 0.50;

        let n_small = settings.calculate_segments(r_small);
        let n_large = settings.calculate_segments(r_large);

        assert!(
            n_large >= n_small,
            "Large radius (N={}) should require >= segments than small radius (N={})",
            n_large,
            n_small
        );
    }

    #[test]
    fn test_builder_overrides() {
        let s = TessellationSettings::medium()
            .with_chord_tolerance(0.00001)
            .with_min_segments(40);

        assert_eq!(s.chord_tolerance, 0.00001);
        assert_eq!(s.min_segments, 40);
        let n = s.calculate_segments(0.05);
        assert!(n >= 40);
    }
}
