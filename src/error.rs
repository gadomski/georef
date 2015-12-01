//! Our custom error enum.

use std::error;
use std::fmt;
use std::io;
use std::num::{ParseIntError, ParseFloatError};

use pabst;
use pof;

/// Our custom error enum.
#[derive(Debug)]
pub enum Error {
    /// Wrapper around `std::io::Error`.
    Io(io::Error),
    /// A source point is missing a gps time value.
    MissingGpsTime,
    /// The IMU/GNSS records do not increase monotonically.
    NonmonotonicImuGnssRecords,
    /// The point is outside of the IMU/GNSS records.
    OutsideOfImuGnssRecords,
    /// Wrapper around `pabst::Error`.
    Pabst(pabst::Error),
    /// Wrapper around `std::num::ParseIntError`.
    ParseInt(ParseIntError),
    /// Wrapper around `std::num::ParseFloatError`.
    ParseFloat(ParseFloatError),
    /// Wrapper around `pof::Error`.
    Pof(pof::Error),
}

impl error::Error for Error {
    fn description(&self) -> &str {
        match *self {
            Error::Io(ref err) => err.description(),
            Error::MissingGpsTime => "missing gps time from point",
            Error::NonmonotonicImuGnssRecords => "imu/gnss records do not monotonically increase",
            Error::OutsideOfImuGnssRecords => "lidar point is outside of imu/gnss records",
            Error::Pabst(ref err) => err.description(),
            Error::ParseInt(ref err) => err.description(),
            Error::ParseFloat(ref err) => err.description(),
            Error::Pof(_) => "pof error",
        }
    }

    fn cause(&self) -> Option<&error::Error> {
        match *self {
            Error::Io(ref err) => Some(err),
            Error::Pabst(ref err) => Some(err),
            Error::ParseInt(ref err) => Some(err),
            Error::ParseFloat(ref err) => Some(err),
            _ => None,
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Error::Io(ref err) => write!(f, "IO error: {}", err),
            Error::MissingGpsTime => write!(f, "Missing gps time"),
            Error::NonmonotonicImuGnssRecords => write!(f, "IMU/GNSS records do not increase monotonically"),
            Error::OutsideOfImuGnssRecords => write!(f, "LiDAR point is outside of IMU/GNSS records"),
            Error::Pabst(ref err) => write!(f, "Pabst error: {}", err),
            Error::ParseInt(ref err) => write!(f, "Parse int error: {}", err),
            Error::ParseFloat(ref err) => write!(f, "Parse float error: {}", err),
            Error::Pof(_) => write!(f, "Pof error"),
        }
    }
}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Error {
        Error::Io(err)
    }
}

impl From<pabst::Error> for Error {
    fn from(err: pabst::Error) -> Error {
        Error::Pabst(err)
    }
}

impl From<ParseIntError> for Error {
    fn from(err: ParseIntError) -> Error {
        Error::ParseInt(err)
    }
}

impl From<ParseFloatError> for Error {
    fn from(err: ParseFloatError) -> Error {
        Error::ParseFloat(err)
    }
}

impl From<pof::Error> for Error {
    fn from(err: pof::Error) -> Error {
        Error::Pof(err)
    }
}
