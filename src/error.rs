//! Our custom error enum.

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
