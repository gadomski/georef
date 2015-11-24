//! Executable inferface for georeferncing point clouds.

extern crate docopt;
extern crate georef;
extern crate las;
extern crate pabst;
extern crate rustc_serialize;

use std::collections::HashMap;

use docopt::Docopt;
use georef::{Georeferencer, ImuGnss, UtmZone};

const USAGE: &'static str = "
Georeference point clouds.

Usage:
    georef <infile> \
                             <gnss-imu-file> <utm-zone> <outfile>
    georef (-h | --help)
    \
                             georef --version

Options:
    -h --help       Display this message.
    \
                             --version       Display version.
";

#[derive(Debug, RustcDecodable)]
struct Args {
    arg_infile: String,
    arg_gnss_imu_file: String,
    arg_outfile: String,
    arg_utm_zone: u8,
}

fn main() {
    let args: Args = Docopt::new(USAGE)
                         .and_then(|d| {
                             d.version(Some(env!("CARGO_PKG_VERSION").to_string())).decode()
                         })
                         .unwrap_or_else(|e| e.exit());

    let mut source_options = HashMap::new();
    source_options.insert("sync-to-pps".to_string(), "true".to_string());
    let mut source = pabst::open_file_source(args.arg_infile, source_options).unwrap();
    let ref pos = ImuGnss::from_path(args.arg_gnss_imu_file).unwrap();
    let mut sink_options = HashMap::new();
    sink_options.insert("scale-factors".to_string(), "0.01 0.01 0.01".to_string());
    sink_options.insert("auto-offsets".to_string(), "true".to_string());
    let mut sink = pabst::open_file_sink(args.arg_outfile, sink_options).unwrap();
    let georeferencer = Georeferencer::new(UtmZone(args.arg_utm_zone));
    georeferencer.georeference(&mut *source, pos, &mut *sink).unwrap();
    sink.close_sink().unwrap();
}
