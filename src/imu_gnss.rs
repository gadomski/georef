//! Position and attitude information.

use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufReader, BufRead};
use std::path::Path;

use Result;
use error::Error;
use linalg::{Matrix3, Vector3};

pub struct ImuGnss {
    points: Vec<ImuGnssPoint>,
}

impl ImuGnss {
    /// Reads IMU/GNSS information from a file.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use georef::imu_gnss::ImuGnss;
    /// let imu_gnss = ImuGnss::from_path("data/0916_2014_ie.pos").unwrap();
    /// ```
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<ImuGnss> {
        let reader = BufReader::new(try!(File::open(path)));
        ImuGnss::read_from(reader)
    }

    /// Reads IMU/GNSS information from a `BufRead`.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use std::fs::File;
    /// use std::io::BufReader;
    /// use georef::imu_gnss::ImuGnss;
    /// let reader = BufReader::new(File::open("data/0916_2014_ie.pos").unwrap());
    /// let imu_gnss = ImuGnss::read_from(reader).unwrap();
    /// ```
    pub fn read_from<B: BufRead>(mut reader: B) -> Result<ImuGnss> {
        let ref mut header: String = String::new();
        try!(reader.read_line(header));
        let npoints = try!(header.trim().parse::<usize>());
        let mut points = Vec::with_capacity(npoints);
        for line in reader.lines() {
            let line = try!(line);
            let values: Vec<_> = line.split_whitespace().collect();
            if values.len() == 0 {
                // Empty line, go ahead and carry on
                continue;
            }
            let point = ImuGnssPoint {
                time: try!(values[0].parse()),
                latitude: Radians::from_degrees(try!(values[1].parse())),
                longitude: Radians::from_degrees(try!(values[2].parse())),
                height: try!(values[3].parse()),
                roll: Radians::from_degrees(try!(values[4].parse())),
                pitch: Radians::from_degrees(try!(values[5].parse())),
                heading: Radians::from_degrees(try!(values[6].parse())),
            };
            points.push(point);
        }
        Ok(ImuGnss { points: points })
    }

    /// Returns this position file's points.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use georef::imu_gnss::ImuGnss;
    /// let imu_gnss = ImuGnss::from_path("data/0916_2014_ie.pos").unwrap();
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
    /// let imu_gnss = ImuGnss::from_path("data/0916_2014_ie.pos").unwrap();
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

#[derive(Debug, Default)]
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
        let latitude = self.latitude.0;
        let longitude = self.longitude.0;
        let long_origin = zone.0 as f64 * 6.0 - 183.0;
        let a = 6378137.0;
        let f = 1.0 / 298.257222101;
        let e2 = 2.0 * f - f * f;
        let ep2 = e2 / (1.0 - e2);

        let N = a / (1.0 - e2 * latitude.sin() * latitude.sin()).sqrt();
        let T = latitude.tan() * latitude.tan();
        let C = ep2 * latitude.cos() * latitude.cos();
        let A = latitude.cos() * (longitude - (long_origin * PI / 180.0));

        let term1 = 1.0 - e2 / 4.0 - (3.0 * e2 * e2) / 64.0 - (5.0 * e2 * e2 * e2) / 256.0;
        let term2 = (3.0 * e2) / 8.0 + (3.0 * e2 * e2) / 32.0 + (45.0 * e2 * e2 * e2) / 1024.0;
        let term3 = (15.0 * e2 * e2) / 256.0 + (45.0 * e2 * e2 * e2) / 1024.0;
        let term4 = (35.0 * e2 * e2 * e2) / 3072.0;

        let M = a *
                (term1 * latitude - term2 * (2.0 * latitude).sin() +
                 term3 * (4.0 * latitude).sin() - term4 * (6.0 * latitude).sin());

        let x1 = ((1.0 - T + C) * A * A * A) / 6.0;
        let x2 = ((5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A * A * A * A * A) / 120.0;
        let x = 0.9996 * N * (A + x1 + x2);

        let y1 = (5.0 - T + 9.0 * C + 4.0 * C * C) * (A * A * A * A) / 24.0;
        let y2 = (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * (A * A * A * A * A * A) /
                 720.0;
        let y3 = (A * A) / 2.0 + y1 + y2;
        let y = 0.9996 * (M + N * latitude.tan() * y3);

        let northing = y;
        let easting = x + 500000.0;
        ImuGnssUtmPoint {
            time: self.time,
            northing: northing,
            easting: easting,
            height: self.height,
            roll: self.roll,
            pitch: self.pitch,
            heading: Radians(self.heading.0 + meridian_convergence(easting, northing)),
        }
    }
}

#[allow(non_snake_case)]
fn meridian_convergence(easting: f64, northing: f64) -> f64 {
    let WGS84_a = 6378137.0; /* Meters */
    let WGS84_f = 1.0 / 298.257222101;

    let e2: f64 = 2.0 * WGS84_f - WGS84_f * WGS84_f;
    let e1 = (1.0 - (1.0 - e2).sqrt()) / (1.0 + (1.0 - e2).sqrt());
    let mu_const = WGS84_a * (1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0);

    let Np = northing / 0.9996;
    let mu = Np / mu_const;
    let foot_lat = footprint_latitude(e1, mu);

    let Ep = (easting - 500000.0) / 0.9996;
    let N = WGS84_a / (1.0 - e2 * foot_lat.sin() * foot_lat.sin()).sqrt();
    let M = (WGS84_a * (1.0 - e2)) / (1.0 - e2 * foot_lat.sin() * foot_lat.sin()).powf(1.5);

    let conv1 = -(Ep / N) * foot_lat.tan();
    let H30 = (Ep / N).powi(3);
    let K28 = N / M;
    let K29 = K28 * K28;
    let J29 = foot_lat.tan() * foot_lat.tan();
    let conv2 = (foot_lat.tan() * H30 / 3.0) * (-2.0 * K29 + 3.0 * K28 + J29);
    conv1 + conv2
}

#[allow(non_snake_case)]
fn footprint_latitude(e1: f64, mu: f64) -> f64 {
    let term1 = 3.0 * e1 / 2.0 - 27.0 * e1 * e1 * e1 / 32.0;
    let term2 = 21.0 * e1 * e1 / 16.0 - 55.0 * e1 * e1 * e1 * e1 / 32.0;
    let term3 = 151.0 * e1 * e1 * e1 / 96.0;
    let term4 = 1097.0 * e1 * e1 * e1 * e1 / 512.0;

    mu + term1 * (2.0 * mu).sin() + term2 * (4.0 * mu).sin() + term3 * (6.0 * mu).sin() +
    term4 * (8.0 * mu).sin()
}

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
    pub fn rotation_matrix(&self) -> Matrix3 {
        let sr = self.roll.0.sin();
        let cr = self.roll.0.cos();
        let sp = self.pitch.0.sin();
        let cp = self.pitch.0.cos();
        let sy = self.heading.0.sin();
        let cy = self.heading.0.cos();
        let mut matrix = Matrix3::new();
        matrix[(0, 0)] = cr * cy + sp * sr * sy;
        matrix[(0, 1)] = cp * sy;
        matrix[(0, 2)] = cy * sr - cr * sp * sy;
        matrix[(1, 0)] = cy * sp * sr - cr * sy;
        matrix[(1, 1)] = cp * cy;
        matrix[(1, 2)] = -sr * sy - cr * cy * sp;
        matrix[(2, 0)] = -cp * sr;
        matrix[(2, 1)] = sp;
        matrix[(2, 2)] = cp * cr;
        matrix
    }

    /// Returns this point's location as a `Vector3`.
    pub fn location(&self) -> Vector3 {
        Vector3(self.easting, self.northing, self.height as f64)
    }
}

#[derive(Debug, Default)]
pub struct Radians(pub f64);

impl Radians {
    pub fn from_degrees(degrees: f64) -> Radians {
        Radians(degrees * PI / 180.0)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct UtmZone(pub u8);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_count() {
        let imu_gnss = ImuGnss::from_path("data/0916_2014_ie.pos").unwrap();
        assert_eq!(722800, imu_gnss.points().len());
    }
}
