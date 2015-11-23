//! Georeference point cloud data.

extern crate las;
extern crate pabst;
#[cfg(feature = "rxp")]
extern crate rivlib;

pub mod error;
pub mod georef;
pub mod imu_gnss;
pub mod linalg;

use std::result;

pub type Result<T> = result::Result<T, error::Error>;
