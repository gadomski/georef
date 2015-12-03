//! Test some georeferencing on some data from Girdwood, AK.

#![cfg(feature = "rxp")]

extern crate georef;
extern crate pabst;
extern crate pos;
extern crate rustc_serialize;
extern crate toml;

use std::fs::File;
use std::io::Read;

use georef::{Georeferencer, GeorefConfig};
use pabst::{open_file_source, open_file_sink};
use pos::Interpolator;

#[test]
fn georeference_it() {
    let mut s = String::new();
    File::open("data/config.toml").unwrap().read_to_string(&mut s).unwrap();
    let mut config = toml::Parser::new(s.as_ref()).parse().unwrap();
    let georeferencer = Georeferencer::new(GeorefConfig::from_toml(config.remove("georef")
                                                                         .unwrap())
                                               .unwrap()).unwrap();
    let ref mut source = open_file_source("data/0916_2014_girdwood35.rxp", config.remove("source"))
                             .unwrap();
    let pos_source = Box::new(pos::pos::Reader::from_path("data/0916_2014_ie.pos").unwrap());
    let ref mut interpolator = Interpolator::new(pos_source).unwrap();
    let mut sink = open_file_sink("target/debug/girdwood.las", config.remove("sink")).unwrap();
    georeferencer.georeference(source, interpolator, &mut sink).unwrap();
    sink.close_sink().unwrap();

    let mut source = open_file_source("target/debug/girdwood.las", None).unwrap();
    let points = source.source_to_end(10000).unwrap();
    assert_eq!(257576, points.len());
    for point in points {
        assert!(point.x > 384830.41);
        assert!(point.y > 6760439.58);
        assert!(point.x < 386447.47);
        assert!(point.y < 6762328.15);
    }
}
