//! Georeference LiDAR points.

use pabst;

use Result;
use error::Error;
use imu_gnss::{ImuGnss, UtmZone};
use linalg::{Matrix3, Vector3};

const DEFAULT_CHUNK_SIZE: usize = 1000;

pub struct Georeferencer {
    boresight_matrix: Matrix3,
    chunk_size: usize,
    lever_arm: Vector3,
    time_offset: f64,
    utm_zone: UtmZone,
}

impl Georeferencer {
    /// Creates a new georeferencer.
    ///
    /// # Examples
    ///
    /// ```
    /// use georef::georef::Georeferencer;
    /// use georef::imu_gnss::UtmZone;
    /// let georeferencer = Georeferencer::new(UtmZone(6));
    /// ```
    pub fn new(utm_zone: UtmZone) -> Georeferencer {
        Georeferencer {
            boresight_matrix: Matrix3::identity(),
            chunk_size: DEFAULT_CHUNK_SIZE,
            lever_arm: Vector3(0.0, 0.0, 0.0),
            time_offset: 0.0,
            utm_zone: utm_zone,
        }
    }

    /// Georeference a point cloud.
    pub fn georeference<P: pabst::Point, S: pabst::Source<Point=P>, T: pabst::Sink>(&self, mut source: S, imu_gnss: &ImuGnss, sink: &mut T) -> Result<()> {
        let mut hint = 0;
        loop {
            let points = match try!(source.source(self.chunk_size)) {
                Some(points) => points,
                None => break,
            };
            for mut point in points {
                let time = try!(point.gps_time().ok_or(Error::MissingGpsTime)) + self.time_offset;
                let (pos, new_hint) = try!(imu_gnss.interpolate_trajectory(time, hint));
                hint = new_hint;
                let pos_utm = pos.into_utm(self.utm_zone);
                let Vector3(x, y, z) = self.boresight_matrix *
                                       Vector3(-1.0 * point.z(), point.x(), point.y()) +
                                       self.lever_arm;
                let Vector3(x, y, z) = pos_utm.rotation_matrix() * Vector3(x, y, z) +
                                       pos_utm.location();
                point.set_x(x);
                point.set_y(y);
                point.set_z(z);
                try!(sink.sink(&point));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[cfg(feature = "rxp")]
mod tests {
    use super::*;

    use std::io::Cursor;

    use las;
    use rivlib;

    use imu_gnss::{ImuGnss, UtmZone};

    #[test]
    fn rxp_georeference() {
        let source = rivlib::Stream::open("data/0916_2014_girdwood35.rxp", true).unwrap();
        let ref pos = ImuGnss::from_path("data/0916_2014_ie.pos").unwrap();
        let ref mut sink = las::File::new();
        let georeferencer = Georeferencer::new(UtmZone(6));
        georeferencer.georeference(source, pos, sink).unwrap();

        let ref mut cursor = Cursor::new(Vec::new());
        sink.write_to(cursor).unwrap();
        cursor.set_position(0);
        let file = las::File::read_from(cursor).unwrap();
        assert_eq!(257576, file.points().len());
    }
}
