//! Position and attitude information.

use std::f64::consts::PI;

use nalgebra::{Mat3, Vec3};
use utm::radians_to_utm_wgs84;

use Result;
use error::Error;

/// A collection of ImuGnss records.
#[derive(Debug)]
pub struct ImuGnss {
    points: Vec<ImuGnssPoint>,
}

impl ImuGnss {
    /// Creates a new set of IMU/GNSS records.
    ///
    /// # Examples
    ///
    /// ```
    /// use georef::imu_gnss::ImuGnss;
    /// let imu_gnss = ImuGnss::new(Vec::new());
    /// ```
    pub fn new(points: Vec<ImuGnssPoint>) -> ImuGnss {
        ImuGnss { points: points }
    }

    /// Returns this position file's points.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use georef::imu_gnss::ImuGnss;
    /// use georef::pos::read_pos_file;
    /// let imu_gnss = ImuGnss::new(read_pos_file("data/0916_2014_ie.pos").unwrap());
    /// let points = imu_gnss.points();
    /// ```
    pub fn points(&self) -> &Vec<ImuGnssPoint> {
        &self.points
    }

    /// Interpolates a position and attidue value for the given time.
    ///
    /// It takes a hint value that is used as the starting index when searching through the points.
    /// Since the usual access pattern for trajcectory information is linear, knowledge of the
    /// previous position is helpful but not technically necessary.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use georef::imu_gnss::ImuGnss;
    /// use georef::pos::read_pos_file;
    /// let imu_gnss = ImuGnss::new(read_pos_file("data/0916_2014_ie.pos").unwrap());
    /// let (point, hint) = imu_gnss.interpolate_trajectory(61191.0, 0).unwrap();
    /// ```
    pub fn interpolate_trajectory(&self,
                                  time: f64,
                                  mut hint: usize)
                                  -> Result<(ImuGnssPoint, usize)> {
        loop {
            if hint >= self.points.len() - 1 {
                return Err(Error::OutsideOfImuGnssRecords);
            }
            let ref first = self.points[hint];
            let ref second = self.points[hint + 1];
            if second.time < first.time {
                return Err(Error::NonmonotonicImuGnssRecords);
            }
            if time < first.time {
                if hint > 0 {
                    hint -= 1;
                    continue;
                } else {
                    return Err(Error::OutsideOfImuGnssRecords);
                }
            } else if time > second.time {
                hint += 1;
                continue;
            }
            let factor = (time - first.time) / (second.time - first.time);
            return Ok((ImuGnssPoint {
                time: first.time + (second.time - first.time) * factor,
                longitude: Radians(first.longitude.0 +
                                   (second.longitude.0 - first.longitude.0) * factor),
                latitude: Radians(first.latitude.0 +
                                  (second.latitude.0 - first.latitude.0) * factor),
                height: first.height + (second.height - first.height) * factor as f32,
                roll: Radians(first.roll.0 + (second.roll.0 - first.roll.0) * factor),
                pitch: Radians(first.pitch.0 + (second.pitch.0 - first.pitch.0) * factor),
                heading: Radians(first.heading.0 + (second.heading.0 - first.heading.0) * factor),
            },
                       hint));
        }
    }
}

/// A location and orientation point.
#[derive(Clone, Copy, Debug, Default)]
#[allow(missing_docs)]
pub struct ImuGnssPoint {
    pub time: f64,
    pub latitude: Radians,
    pub longitude: Radians,
    pub height: f32,
    pub roll: Radians,
    pub pitch: Radians,
    pub heading: Radians,
}

impl ImuGnssPoint {
    /// Creates a new, default point.
    ///
    /// # Examples
    ///
    /// ```
    /// use georef::imu_gnss::ImuGnssPoint;
    /// let point = ImuGnssPoint::new();
    /// ```
    pub fn new() -> ImuGnssPoint {
        Default::default()
    }

    /// Converts this lat-lon point to a UTM point in the given zone.
    ///
    /// # Examples
    ///
    /// ```
    /// use georef::imu_gnss::{ImuGnssPoint, Radians, UtmZone};
    /// let mut point = ImuGnssPoint::new();
    /// point.latitude = Radians::from_degrees(60.9679875497);
    /// point.longitude = Radians::from_degrees(-149.119325194);
    /// point.heading = Radians::from_degrees(132.88734);
    /// let utm_point = point.into_utm(UtmZone(6));
    /// assert!((385273.02 - utm_point.easting).abs() < 1e-2);
    /// assert!((6761077.20 - utm_point.northing).abs() < 1e-2);
    /// assert!((2.351667 - utm_point.heading.0).abs() < 1e-6);
    /// ```
    #[allow(non_snake_case)]
    pub fn into_utm(self, zone: UtmZone) -> ImuGnssUtmPoint {
        let (northing, easting, meridian_convergence) = radians_to_utm_wgs84(self.latitude.0,
                                                                             self.longitude.0,
                                                                             zone.0);
        ImuGnssUtmPoint {
            time: self.time,
            northing: northing,
            easting: easting,
            height: self.height,
            roll: self.roll,
            pitch: self.pitch,
            heading: Radians(self.heading.0 + meridian_convergence),
        }
    }
}

/// A IMU/GNSS point in UTM.
#[derive(Clone, Copy, Debug)]
#[allow(missing_docs)]
pub struct ImuGnssUtmPoint {
    pub time: f64,
    pub northing: f64,
    pub easting: f64,
    pub height: f32,
    pub roll: Radians,
    pub pitch: Radians,
    pub heading: Radians,
}

impl ImuGnssUtmPoint {
    /// Returns a rotation matrix for this point.
    ///
    /// TODO generalize this for other rotation definitions.
    ///
    /// # Examples
    ///
    /// TODO provide examples
    pub fn rotation_matrix(&self) -> Mat3<f64> {
        let sr = self.roll.0.sin();
        let cr = self.roll.0.cos();
        let sp = self.pitch.0.sin();
        let cp = self.pitch.0.cos();
        let sy = self.heading.0.sin();
        let cy = self.heading.0.cos();
        Mat3::new(cr * cy + sp * sr * sy,
                  cp * sy,
                  cy * sr - cr * sp * sy,
                  cy * sp * sr - cr * sy,
                  cp * cy,
                  -sr * sy - cr * cy * sp,
                  -cp * sr,
                  sp,
                  cp * cr)
    }

    /// Returns this point's location as a `Vector3`.
    pub fn location(&self) -> Vec3<f64> {
        Vec3::new(self.easting, self.northing, self.height as f64)
    }
}

/// Newtype wrapper around a raidan value.
#[derive(Clone, Copy, Debug, Default)]
pub struct Radians(pub f64);

impl Radians {
    /// Convert a degree value to a radian value.
    pub fn from_degrees(degrees: f64) -> Radians {
        Radians(degrees * PI / 180.0)
    }
}

/// A newtype wrapper around a UTM zone.
#[derive(Clone, Copy, Debug)]
pub struct UtmZone(pub u8);
