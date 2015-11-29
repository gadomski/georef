//! Read pos data.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use Result;
use imu_gnss::{ImuGnssPoint, Radians};

/// Reads a pos file into a vector of points.
///
/// # Examples
///
/// ```no_run
/// use georef::pos::read_pos_file;
/// let points = read_pos_file("data/0916_2014_ie.pos").unwrap();
/// ```
pub fn read_pos_file<P: AsRef<Path>>(path: P) -> Result<Vec<ImuGnssPoint>> {
    let mut reader = BufReader::new(try!(File::open(path)));
    let ref mut header: String = String::new();
    let _ = try!(reader.read_line(header));
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
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_count() {
        let points = read_pos_file("data/0916_2014_ie.pos").unwrap();
        assert_eq!(722800, points.len());
    }
}
