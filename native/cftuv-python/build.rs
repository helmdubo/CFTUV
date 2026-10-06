//! Embeds the digest of the Rust sources (`digest.rs`) into the extension as `CFTUV_NATIVE_SOURCE_DIGEST`: the Rust half of `cftuv_native.native_build_id()`.
//!
//! The digest is content, not bytes of the binary (a link stamps a timestamp and a GUID into every `.pyd`). Every hashed file, and every `src` directory
//! of the workspace, is watched: an edit anywhere in the native code re-runs this script and so rebuilds this crate with the new digest.

use std::env;
use std::path::PathBuf;

#[path = "digest.rs"]
mod digest;

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("cargo sets CARGO_MANIFEST_DIR"));
    let root = manifest.parent().expect("cftuv-python lives in the native workspace");
    let found = digest::tree_digest(root).unwrap_or_else(|error| panic!("cftuv-python: cannot hash the Rust sources under {}: {error}", root.display()));
    println!("cargo:rustc-env=CFTUV_NATIVE_SOURCE_DIGEST={}", found.hex);
    for path in &found.watched {
        println!("cargo:rerun-if-changed={}", path.display());
    }
}
