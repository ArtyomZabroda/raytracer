use std::env;

use raytracer::{Config, render_image};

fn main() {
    let args_iter = env::args();
    let config = match Config::build(args_iter) {
        Ok(config) => config,
        Err(e) => panic!("{e}")
    };
    render_image(config);
}