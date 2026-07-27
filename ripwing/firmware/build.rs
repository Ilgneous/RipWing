//! Build script: put this crate's directory on the linker search path so
//! `memory.x` (our chip memory layout) is found, and rebuild if it changes.
//!
//! cortex-m-rt's own `link.x` does `INCLUDE memory.x`, so the linker must be
//! able to find memory.x. Emitting the crate dir as a search path is the
//! standard way to provide it.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Copy memory.x into OUT_DIR and add OUT_DIR to the linker search path.
    let memory_x = include_bytes!("memory.x");
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(memory_x)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // Rebuild if the memory layout changes.
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
