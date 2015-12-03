//! Point management.

use nalgebra::{Rot3, Vec3};
use pos;
use pos::{Accuracy, Radians};
use utm;

use rotation::RotationOrder;

#[derive(Debug, Default)]
pub struct UtmPoint {
    northing: f64,
    easting: f64,
    altitude: f64,
    roll: Radians<f64>,
    pitch: Radians<f64>,
    yaw: Radians<f64>,
    accuracy: Option<Accuracy>,
}

impl UtmPoint {
    /// Converts a pos point into a utm point.
    pub fn from_latlon(point: &pos::Point, utm_zone: u8) -> UtmPoint {
        let (northing, easting, meridian_convergence) = utm::radians_to_utm_wgs84(point.latitude.0, point.longitude.0, utm_zone);
        UtmPoint {
            northing: northing,
            easting: easting,
            altitude: point.altitude,
            roll: point.roll,
            pitch: point.pitch,
            yaw: point.yaw + Radians(meridian_convergence),
            accuracy: point.accuracy,
        }
    }

    /// Returns the rotation matrix for this UTM point.
    pub fn rotation_matrix(&self, rotation_order: &RotationOrder) -> Rot3<f64> {
        rotation_order.rot3(self.roll.0, self.pitch.0, self.yaw.0)
    }

    /// Returns this point's location as a vec3.
    pub fn location(&self) -> Vec3<f64> {
        Vec3::new(self.easting, self.northing, self.altitude)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::{Eye, Rot3};
    use pos::Radians;

    #[test]
    fn no_rotation() {
        let point = UtmPoint {
            roll: Radians(0.0),
            pitch: Radians(0.0),
            yaw: Radians(0.0),
            ..Default::default()
        };
        let rotation_order = Default::default();
        assert_eq!(Rot3::new_identity(3), point.rotation_matrix(&rotation_order));
    }
}
