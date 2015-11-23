//! Executable inferface for georeferncing point clouds.
//!
//! For now, this just converts rxp to las. However, when we make pabst smarter this will do more.

extern crate docopt;
extern crate georef;
extern crate las;
extern crate pabst;
#[cfg(feature = "rxp")]
extern crate rivlib;
extern crate rustc_serialize;

use docopt::Docopt;
#[cfg(feature = "rxp")]
use georef::imu_gnss::ImuGnss;
#[cfg(feature = "rxp")]
use georef::imu_gnss::UtmZone;
#[cfg(feature = "rxp")]
use georef::georef::Georeferencer;

const USAGE: &'static str = "
Georeference point clouds.

Usage:
    georef <infile> \
                             <gnss-imu-file> <outfile>
    georef (-h | --help)
    georef \
                             --version

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
}

#[cfg(feature = "rxp")]
fn handle_args(args: Args) {
    let source = rivlib::Stream::open(args.arg_infile, true).unwrap();
    let ref pos = ImuGnss::from_path(args.arg_gnss_imu_file).unwrap();
    let ref mut sink = las::File::new().scale_factors(0.01, 0.01, 0.01).auto_offsets(true);
    let georeferencer = Georeferencer::new(UtmZone(6));
    georeferencer.georeference(source, pos, sink).unwrap();
    sink.to_path(args.arg_outfile).unwrap();
}

#[cfg(not(feature = "rxp"))]
#[allow(unused_variables)]
fn handle_args(args: Args) {
    println!("That's all, folks!");
}

fn main() {
    let args: Args = Docopt::new(USAGE)
                         .and_then(|d| {
                             d.version(Some(env!("CARGO_PKG_VERSION").to_string())).decode()
                         })
                         .unwrap_or_else(|e| e.exit());

    handle_args(args);
}
