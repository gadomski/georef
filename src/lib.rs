//! Georeference point cloud data.

extern crate las;
extern crate pabst;
extern crate pof;

pub mod error;
pub mod georef;
pub mod imu_gnss;
pub mod linalg;

pub use georef::Georeferencer;
pub use imu_gnss::{ImuGnss, UtmZone};

use std::result;

pub type Result<T> = result::Result<T, error::Error>;
