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

use rustc_serialize::Decodable;

use docopt::Docopt;
use georef::{Error, Result, Georeferencer, GeorefConfig, ImuGnss, ImuGnssPoint, Radians};
use georef::pos::read_pos_file;
use pof::pof::Reader as PofReader;

const USAGE: &'static str = "
Georeference point clouds.

Usage:
    georef <infile> \
                             <gnss-imu-file> <config-file> <outfile>
    georef (-h | --help)
    \
                             georef --version

Options:
    -h --help                       \
                             Display this message.
    --version                       Display  \
                             version.
    ";

#[derive(Debug, RustcDecodable)]
struct Args {
    flag_config_file: Option<String>,
    flag_utm_zone: Option<u8>,
    arg_config_file: String,
    arg_infile: String,
    arg_gnss_imu_file: String,
    arg_outfile: String,
}

macro_rules! exit {
    ($($tts:tt)*) => {{
        println!($($tts)*);
        exit(1);
    }}
}

fn main() {
    let args: Args = Docopt::new(USAGE)
                         .and_then(|d| {
                             d.version(Some(env!("CARGO_PKG_VERSION").to_string())).decode()
                         })
                         .unwrap_or_else(|e| e.exit());

    let mut config = String::new();
    File::open(args.arg_config_file)
        .unwrap_or_else(|e| {
            exit!("ERROR: problem reading config file: {}", e);
        })
        .read_to_string(&mut config)
        .unwrap_or_else(|e| {
            exit!("ERROR: problem reading file to string: {}", e);
        });
    let mut parser = toml::Parser::new(&config);
    let mut config = parser.parse().unwrap_or_else(|| {
        exit!("ERROR: unable to parse config file: {:?}", parser.errors);
    });
    let mut source = pabst::open_file_source(args.arg_infile, config.remove("source"))
                         .unwrap_or_else(|e| {
                             exit!("ERROR: while opening source: {}", e);
                         });
    let ref pos = imu_gnss_from_path(args.arg_gnss_imu_file).unwrap_or_else(|e| {
        exit!("ERROR: unable to read IMU/GNSS data: {}", e);
    });
    let mut sink = pabst::open_file_sink(args.arg_outfile, config.remove("sink"))
                       .unwrap_or_else(|e| {
                           exit!("ERROR: while opening sink: {}", e);
                       });

    let georef_config = config.remove("georef")
                              .unwrap_or_else(|| {
                                  exit!("ERROR: configuration file must have a 'georef' table");
                              });
    let georef_config = GeorefConfig::decode(&mut toml::Decoder::new(georef_config)).unwrap_or_else(|e| {
        exit!("ERROR: unable to decode georef configuration: {}", e);
    });
    let georeferencer = Georeferencer::new(georef_config);

    georeferencer.georeference(&mut *source, pos, &mut *sink).unwrap();
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
