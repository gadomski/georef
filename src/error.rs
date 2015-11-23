//! Our custom error enum.

use std::io;
use std::num::{ParseIntError, ParseFloatError};

use pabst;

#[derive(Debug)]
pub enum Error {
    Io(io::Error),
    MissingGpsTime,
    NonmonotonicImuGnssRecords,
    OutsideOfImuGnssRecords,
    Pabst(pabst::Error),
    ParseInt(ParseIntError),
    ParseFloat(ParseFloatError),
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
