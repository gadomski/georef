//! Executable inferface for georeferncing point clouds.

extern crate docopt;
extern crate georef;
extern crate las;
extern crate pabst;
extern crate pof;
extern crate rustc_serialize;
extern crate toml;

use std::ffi::OsStr;
use std::fs::File;
use std::io::Read;
use std::path::Path;
use std::process::exit;

use docopt::Docopt;
use georef::{Error, Result, Georeferencer, ImuGnss, ImuGnssPoint, Radians, UtmZone};
use georef::pos::read_pos_file;
use pof::pof::Reader as PofReader;

const USAGE: &'static str = "
Georeference point clouds.

Usage:
    georef <infile> \
                             <gnss-imu-file> <outfile> [--config-file=<config-file>] \
                             [--utm-zone=<z>]
    georef (-h | --help)
    georef --version

\
                             Options:
    -h --help                       Display this message.
    \
                             --version                       Display  version.
    \
                             --config-file=<config-file>     Configuration file.
    \
                             --utm-zone=<n>                  UTM zone for final data, overrides \
                             value in configuration file.
";

#[derive(Debug, RustcDecodable)]
struct Args {
    flag_config_file: Option<String>,
    flag_utm_zone: Option<u8>,
    arg_infile: String,
    arg_gnss_imu_file: String,
    arg_outfile: String,
}

fn main() {
    let args: Args = Docopt::new(USAGE)
                         .and_then(|d| {
                             d.version(Some(env!("CARGO_PKG_VERSION").to_string())).decode()
                         })
                         .unwrap_or_else(|e| e.exit());

    let config = if let Some(config_file) = args.flag_config_file {
        let mut config = String::new();
        File::open(config_file).unwrap().read_to_string(&mut config).unwrap();
        toml::Parser::new(&config).parse().unwrap()
    } else {
        toml::Table::new()
    };

    println!("Opening the point source");
    let mut source = pabst::open_file_source(args.arg_infile,
                                             config.get("source").and_then(|b| b.as_table()))
                         .unwrap();

    println!("Reading IMU/GNSS data");
    let ref pos = imu_gnss_from_path(args.arg_gnss_imu_file).unwrap();

    println!("Opening the point sink");
    let mut sink = pabst::open_file_sink(args.arg_outfile,
                                         config.get("sink").and_then(|b| b.as_table()))
                       .unwrap();

    let georef_config = config.get("georef").and_then(|b| b.as_table());
    let utm_zone = if let Some(zone) = args.flag_utm_zone {
        UtmZone(zone)
    } else if let Some(table) = georef_config {
        UtmZone(table.get("utm-zone").unwrap().as_integer().unwrap() as u8)
    } else {
        println!("No UTM zone provided");
        exit(1);
    };
    let georeferencer = Georeferencer::new(utm_zone);

    println!("Georeferencing");
    georeferencer.georeference(&mut *source, pos, &mut *sink).unwrap();
    println!("Done");

    sink.close_sink().unwrap();
}

fn imu_gnss_from_path<P: AsRef<Path> + AsRef<OsStr>>(path: P) -> Result<ImuGnss> {
    let path = Path::new(&path);
    let ext = path.extension().and_then(|p| p.to_str());
    match ext {
        Some("pos") => Ok(ImuGnss::new(try!(read_pos_file(path)))),
        Some("pof") => {
            let records = try!(PofReader::from_path(path))
                              .into_iter()
                              .map(|p| {
                                  ImuGnssPoint {
                                      time: p.time,
                                      latitude: Radians::from_degrees(p.latitude),
                                      longitude: Radians::from_degrees(p.longitude),
                                      height: p.altitude as f32,
                                      roll: Radians::from_degrees(p.roll),
                                      pitch: Radians::from_degrees(p.pitch),
                                      heading: Radians::from_degrees(p.yaw),
                                  }
                              })
                              .collect();
            Ok(ImuGnss::new(records))
        }
        Some(_) | None => panic!("unknown file extension"),
    }
}
