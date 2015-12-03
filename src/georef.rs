//! Georeference LiDAR points.

use std::result;

use nalgebra::{Col, Eye, Rot3, Vec3};
use pabst;
use pos;
use rustc_serialize::Decodable;
use toml;

use Result;
use error::Error;
use point::UtmPoint;
use rotation::RotationOrder;

const DEFAULT_CHUNK_SIZE: usize = 1000;

/// A decodable configuration object.
#[derive(Debug, RustcDecodable)]
pub struct GeorefConfig {
    /// The boresight matrix.
    ///
    /// This is the rotational offset between the scanner and the GNSS/IMU.
    pub boresight: Rpy,
    /// The size of each processing chunk.
    pub chunk_size: Option<usize>,
    /// The lever arm.
    ///
    /// This is the x, y, and z displacements between the GNSS/IMU and the scanner.
    pub lever_arm: Vec3<f64>,
    /// A mapping between the scanner's own coordinate frame and that of the IMU's.
    pub socs_map: SocsStringMap,
    /// The rotation order for our IMU.
    pub rotation_order: [String; 3],
    /// A time value to apply to each laser point.
    ///
    /// Used if there is some skew between the laser and scanner clocks.
    pub time_offset: Option<f64>,
    /// The UTM zone of the output points.
    pub utm_zone: u8,
    /// Limit the number of points written out.
    pub limit: Option<usize>,
}

impl Default for GeorefConfig {
    fn default() -> GeorefConfig {
        GeorefConfig {
            boresight: Rpy {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            },
            chunk_size: None,
            lever_arm: Vec3::new(0.0, 0.0, 0.0),
            rotation_order: Default::default(),
            socs_map: Default::default(),
            time_offset: None,
            utm_zone: 0,
            limit: None,
        }
    }
}

impl GeorefConfig {
    /// Creates a new georef config from a toml value.
    pub fn from_toml(table: toml::Value) -> result::Result<GeorefConfig, toml::DecodeError> {
        GeorefConfig::decode(&mut toml::Decoder::new(table))
    }
}

/// Roll, pitch, and yaw.
#[derive(Clone, Copy, Debug, Default, RustcDecodable)]
pub struct Rpy {
    roll: f64,
    pitch: f64,
    yaw: f64,
}

impl Rpy {
    /// Converts this roll, pitch, and yaw into a rotation matrix.
    pub fn into_rot3(self, rotation_order: &RotationOrder) -> Rot3<f64> {
        rotation_order.rot3(self.roll, self.pitch, self.yaw)
    }
}

/// A mapping between the scanner's own coordinate frame and the IMU's that's easy to decode.
#[derive(Debug, Default, RustcDecodable)]
pub struct SocsStringMap {
    x: String,
    y: String,
    z: String,
}

#[derive(Debug, RustcDecodable)]
struct SocsMap {
    rotation_matrix: Rot3<f64>,
}

impl SocsMap {
    fn new(map: SocsStringMap) -> Result<SocsMap> {
        let mut rot = Rot3::new_identity(3);
        for (i, s) in vec![map.x, map.y, map.z].iter().enumerate() {
            rot.set_col(i,
                        match s.as_ref() {
                            "x" => Vec3::x(),
                            "-x" => -Vec3::x(),
                            "y" => Vec3::y(),
                            "-y" => -Vec3::y(),
                            "z" => Vec3::z(),
                            "-z" => -Vec3::z(),
                            _ => return Err(Error::SocsMap(s.clone())),
                        });
        }
        Ok(SocsMap { rotation_matrix: rot })
    }

    fn vec3(&self, point: &pabst::Point) -> Vec3<f64> {
        Vec3::new(point.x, point.y, point.z) * self.rotation_matrix
    }
}


/// A configurable structure for georeferencing points.
#[derive(Debug)]
pub struct Georeferencer {
    boresight_matrix: Rot3<f64>,
    chunk_size: usize,
    lever_arm: Vec3<f64>,
    limit: Option<usize>,
    rotation_order: RotationOrder,
    socs_map: SocsMap,
    time_offset: f64,
    utm_zone: u8,
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
    pub fn new(config: GeorefConfig) -> Result<Georeferencer> {
        let rotation_order = try!(RotationOrder::new(config.rotation_order[0].as_ref(),
                                                     config.rotation_order[1].as_ref(),
                                                     config.rotation_order[2].as_ref()));
        Ok(Georeferencer {
            boresight_matrix: rotation_order.rot3(config.boresight.roll,
                                                  config.boresight.pitch,
                                                  config.boresight.yaw),
            chunk_size: config.chunk_size.unwrap_or(DEFAULT_CHUNK_SIZE),
            lever_arm: config.lever_arm,
            limit: config.limit,
            rotation_order: rotation_order,
            socs_map: try!(SocsMap::new(config.socs_map)),
            time_offset: config.time_offset.unwrap_or(0.0),
            utm_zone: config.utm_zone,
        })
    }

    /// Georeference a point cloud.
    pub fn georeference(&self,
                        source: &mut pabst::Source,
                        interpolator: &mut pos::Interpolator,
                        sink: &mut pabst::Sink)
                        -> Result<()> {
        let mut npoints = 0;
        loop {
            let points = match try!(source.source(self.chunk_size)) {
                Some(points) => points,
                None => break,
            };
            for mut point in points {
                try!(self.georeference_point(&mut point, interpolator));
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

    /// Georeference a single point.
    pub fn georeference_point(&self,
                              point: &mut pabst::Point,
                              interpolator: &mut pos::Interpolator)
                              -> Result<()> {
        let time = try!(point.gps_time.ok_or(Error::MissingGpsTime)) + self.time_offset;
        let pos = try!(interpolator.interpolate(time));
        let pos = UtmPoint::from_latlon(&pos, self.utm_zone);
        let p = pos.rotation_matrix(&self.rotation_order) *
                (self.boresight_matrix * self.socs_map.vec3(&point) + self.lever_arm) +
                pos.location();
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        Ok(())
    }
}
