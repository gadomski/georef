//! Georeference LiDAR points.

use nalgebra::{Eye, Mat3, Vec3};
use pabst;

use Result;
use error::Error;
use imu_gnss::{ImuGnss, UtmZone};

const DEFAULT_CHUNK_SIZE: usize = 1000;

/// A decodable configuration object.
#[derive(Clone, Copy, Debug, Default, RustcDecodable)]
pub struct GeorefConfig {
    /// The UTM zone of the output points.
    pub utm_zone: u8,
    /// Limit the number of points written out.
    pub limit: Option<usize>,
}

/// A configurable structure for georeferencing points.
#[derive(Clone, Copy, Debug)]
pub struct Georeferencer {
    boresight_matrix: Mat3<f64>,
    chunk_size: usize,
    lever_arm: Vec3<f64>,
    limit: Option<usize>,
    time_offset: f64,
    utm_zone: UtmZone,
}

impl Georeferencer {
    /// Creates a new georeferencer.
    ///
    /// # Examples
    ///
    /// ```
    /// use georef::georef::{GeorefConfig, Georeferencer};
    /// let config = GeorefConfig { utm_zone: 6, ..Default::default() };
    /// let georeferencer = Georeferencer::new(config);
    /// ```
    pub fn new(config: GeorefConfig) -> Georeferencer {
        Georeferencer {
            boresight_matrix: Mat3::new_identity(3),
            chunk_size: DEFAULT_CHUNK_SIZE,
            lever_arm: Vec3::new(0.0, 0.0, 0.0),
            limit: config.limit,
            time_offset: 0.0,
            utm_zone: UtmZone(config.utm_zone),
        }
    }

    /// Georeference a point cloud.
    pub fn georeference(&self,
                        source: &mut pabst::Source,
                        imu_gnss: &ImuGnss,
                        sink: &mut pabst::Sink)
                        -> Result<()> {
        let mut hint = 0;
        let mut npoints = 0;
        loop {
            let points = match try!(source.source(self.chunk_size)) {
                Some(points) => points,
                None => break,
            };
            for mut point in points {
                let time = try!(point.gps_time.ok_or(Error::MissingGpsTime)) + self.time_offset;
                let (pos, new_hint) = try!(imu_gnss.interpolate_trajectory(time, hint));
                hint = new_hint;
                let pos_utm = pos.into_utm(self.utm_zone);
                let v = self.boresight_matrix * Vec3::new(-1.0 * point.z, point.x, point.y) +
                        self.lever_arm;
                let v = pos_utm.rotation_matrix() * v + pos_utm.location();
                point.x = v.x;
                point.y = v.y;
                point.z = v.z;
                try!(sink.sink(&point));
                npoints += 1;
                if let Some(limit) = self.limit {
                    if npoints >= limit {
                        return Ok(());
                    }
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[cfg(feature = "rxp")]
mod tests {
    use super::*;

    use std::fs::remove_file;

    use pabst::{open_file_source, open_file_sink};

    use imu_gnss::ImuGnss;
    use pos::read_pos_file;

    #[test]
    fn rxp_georeference() {
        let mut source = open_file_source("data/0916_2014_girdwood35.rxp", None).unwrap();
        let ref pos = ImuGnss::new(read_pos_file("data/0916_2014_ie.pos").unwrap());
        let mut sink = open_file_sink("temp.las", None).unwrap();
        let georeferencer = Georeferencer::new(GeorefConfig { utm_zone: 6 });
        georeferencer.georeference(&mut *source, pos, &mut *sink).unwrap();

        sink.close_sink().unwrap();

        let mut source = open_file_source("temp.las", None).unwrap();
        assert_eq!(257576, source.source_to_end(300000).unwrap().len());
        remove_file("temp.las").unwrap();
    }
}
