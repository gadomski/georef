//! Rotation order information.

use std::str::FromStr;

use nalgebra::{Rot3, Vec3};

use {Error, Result};

#[derive(Debug)]
pub struct RotationOrder {
    first: RotationMatrix,
    second: RotationMatrix,
    third: RotationMatrix,
}

impl RotationOrder {
    /// Creates a new rotation order from three strings.
    pub fn new(first: &str, second: &str, third: &str) -> Result<RotationOrder> {
        Ok(RotationOrder {
            first: try!(first.parse()),
            second: try!(second.parse()),
            third: try!(third.parse()),
        })
    }

    /// Returns a rotation matrix for the three provided angles.
    pub fn rot3(&self, roll: f64, pitch: f64, yaw: f64) -> Rot3<f64> {
        self.first.rot3(roll, pitch, yaw) * self.second.rot3(roll, pitch, yaw) *
        self.third.rot3(roll, pitch, yaw)
    }
}

impl Default for RotationOrder {
    fn default() -> RotationOrder {
        RotationOrder::new("r3(yaw)", "r2(pitch)", "r1(roll)").unwrap()
    }
}

#[derive(Debug)]
pub struct RotationMatrix {
    type_: RotationMatrixType,
    negative: bool,
    angle: RotationMatrixAngle,
}

impl RotationMatrix {
    fn rot3(&self, roll: f64, pitch: f64, yaw: f64) -> Rot3<f64> {
        let factor = if self.negative {
            -1.0
        } else {
            1.0
        };
        Rot3::new(self.type_.vec3() * factor * self.angle.select(roll, pitch, yaw))
    }
}

#[derive(Debug)]
enum RotationMatrixType {
    R1,
    R2,
    R3,
}

impl RotationMatrixType {
    fn vec3(&self) -> Vec3<f64> {
        match *self {
            RotationMatrixType::R1 => Vec3::new(1.0, 0.0, 0.0),
            RotationMatrixType::R2 => Vec3::new(0.0, 1.0, 0.0),
            RotationMatrixType::R3 => Vec3::new(0.0, 0.0, 1.0),
        }
    }
}

#[derive(Debug)]
enum RotationMatrixAngle {
    Roll,
    Pitch,
    Yaw,
}

impl RotationMatrixAngle {
    fn select(&self, r: f64, p: f64, y: f64) -> f64 {
        match *self {
            RotationMatrixAngle::Roll => r,
            RotationMatrixAngle::Pitch => p,
            RotationMatrixAngle::Yaw => y,
        }
    }
}

impl FromStr for RotationMatrix {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self> {
        let v: Vec<_> = s.split("(").collect();
        if v.len() != 2 || !v[1].ends_with(")") {
            return Err(Error::ParseRotate(s.to_string()));
        }
        let negative = v[1].starts_with("-");
        let angle_name = v[1].trim_left_matches("-").trim_right_matches(")");
        Ok(RotationMatrix {
            type_: try!(v[0].parse()),
            negative: negative,
            angle: try!(angle_name.parse()),
        })
    }
}

impl FromStr for RotationMatrixType {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self> {
        match s {
            "r1" | "R1" | "1" => Ok(RotationMatrixType::R1),
            "r2" | "R2" | "2" => Ok(RotationMatrixType::R2),
            "r3" | "R3" | "3" => Ok(RotationMatrixType::R3),
            _ => Err(Error::ParseRotate(s.to_string())),
        }
    }
}

impl FromStr for RotationMatrixAngle {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self> {
        match s {
            "roll" => Ok(RotationMatrixAngle::Roll),
            "pitch" => Ok(RotationMatrixAngle::Pitch),
            "yaw" => Ok(RotationMatrixAngle::Yaw),
            _ => Err(Error::ParseRotate(s.to_string())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paces() {
        assert!("r1(roll)".parse::<RotationMatrix>().is_ok());
        assert!("r1(-roll)".parse::<RotationMatrix>().is_ok());
        assert!("r2(pitch)".parse::<RotationMatrix>().is_ok());
        assert!("r3(yaw)".parse::<RotationMatrix>().is_ok());
        assert!("r1(roll".parse::<RotationMatrix>().is_err());
        assert!("r1[roll)".parse::<RotationMatrix>().is_err());
        assert!("r4(roll)".parse::<RotationMatrix>().is_err());
        assert!("r1(rollz)".parse::<RotationMatrix>().is_err());
        assert!("r1(rol)".parse::<RotationMatrix>().is_err());
    }
}
