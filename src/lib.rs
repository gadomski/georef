//! Georeference point cloud data.

#![deny(fat_ptr_transmutes, missing_copy_implementations, missing_debug_implementations, missing_docs, trivial_casts, trivial_numeric_casts, unused_extern_crates, unused_import_braces, unused_qualifications, unused_results, variant_size_differences)]

extern crate nalgebra;
extern crate pabst;
extern crate pos;
extern crate rustc_serialize;
extern crate toml;
extern crate utm;

mod point;
mod rotation;
pub mod error;
pub mod georef;

pub use error::Error;
pub use georef::{GeorefConfig, Georeferencer};

use std::result;

/// Our custom result type.
pub type Result<T> = result::Result<T, Error>;
