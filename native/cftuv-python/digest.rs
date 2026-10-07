//! Content digest of the Rust sources of the native workspace: the Rust half of `cftuv_native.native_build_id()`.
//!
//! Shared by `build.rs` (which embeds the digest of the tree it is built from as `CFTUV_NATIVE_SOURCE_DIGEST`) and by the extension itself
//! (`tree_digest`, for the tests to run the algorithm on a tree of their own). The digest depends on CONTENT only, never on the link: the MSVC linker stamps a
//! timestamp and a PDB GUID into every `_core.pyd`, so two builds of the same code differ byte for byte, and hashing the binary would not be an identity.
//!
//! The rule, mirrored by `tools/native_build_id.py::rust_tree_digest`, over the workspace rooted at `native/`:
//!
//! * the files hashed are `Cargo.toml` and `Cargo.lock` of the root; in each crate directory (a direct subdirectory holding a `Cargo.toml`, `target` and
//!   hidden directories excepted) the files directly inside it named `Cargo.toml`, `pyproject.toml` or `*.rs` (`build.rs`, `digest.rs`), and every `*.rs`
//!   below its `src/`, recursively. What links into the extension and how it is built; not tests, examples, data, or the `.py` shim (`buildid.py` hashes that
//!   at run time);
//! * the digest is the sha256 over those files in ascending order of their relative path (UTF-8 bytes, `/` separators), each as
//!   `path 0x00 decimal length 0x00 content 0x0A`, the content with every CRLF turned into LF (a Windows checkout and a Linux one agree).

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use sha2::{Digest, Sha256};

/// The digest of a tree and what a build script must watch to know when it is out of date.
#[allow(dead_code)]
pub struct TreeDigest {
    /// Lowercase hex sha256.
    pub hex: String,
    /// Every hashed file and every `src` directory (a new file in one changes the digest).
    pub watched: Vec<PathBuf>,
}

fn invalid(message: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

fn is_manifest_or_source(name: &str) -> bool {
    name == "Cargo.toml" || name == "pyproject.toml" || name.ends_with(".rs")
}

fn file_name(entry: &fs::DirEntry) -> io::Result<String> {
    entry.file_name().into_string().map_err(|name| invalid(format!("a file name that is not UTF-8: {name:?}")))
}

/// Every `*.rs` under `dir`, recursively, as `(relative path, path)`.
fn collect_sources(dir: &Path, relative: &str, out: &mut Vec<(String, PathBuf)>) -> io::Result<()> {
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let name = file_name(&entry)?;
        let found = format!("{relative}/{name}");
        let kind = entry.file_type()?;
        if kind.is_dir() {
            collect_sources(&entry.path(), &found, out)?;
        } else if kind.is_file() && name.ends_with(".rs") {
            out.push((found, entry.path()));
        }
    }
    Ok(())
}

/// The digest of the workspace at `root` (the `native/` directory), by the rule in the module note.
pub fn tree_digest(root: &Path) -> io::Result<TreeDigest> {
    let mut files: Vec<(String, PathBuf)> = Vec::new();
    let mut watched_dirs: Vec<PathBuf> = Vec::new();
    for name in ["Cargo.toml", "Cargo.lock"] {
        let path = root.join(name);
        if !path.is_file() {
            return Err(io::Error::new(io::ErrorKind::NotFound, format!("{} is not a native workspace root: no {name}", root.display())));
        }
        files.push((name.to_string(), path));
    }
    for entry in fs::read_dir(root)? {
        let entry = entry?;
        let name = file_name(&entry)?;
        if name == "target" || name.starts_with('.') || !entry.file_type()?.is_dir() || !entry.path().join("Cargo.toml").is_file() {
            continue;
        }
        for item in fs::read_dir(entry.path())? {
            let item = item?;
            let item_name = file_name(&item)?;
            if item.file_type()?.is_file() && is_manifest_or_source(&item_name) {
                files.push((format!("{name}/{item_name}"), item.path()));
            }
        }
        let source = entry.path().join("src");
        if source.is_dir() {
            collect_sources(&source, &format!("{name}/src"), &mut files)?;
            watched_dirs.push(source);
        }
    }
    files.sort_by(|left, right| left.0.as_bytes().cmp(right.0.as_bytes()));
    let mut hasher = Sha256::new();
    for (relative, path) in &files {
        let raw = fs::read(path)?;
        let mut content = Vec::with_capacity(raw.len());
        let mut index = 0;
        while index < raw.len() {
            if raw[index] == b'\r' && raw.get(index + 1) == Some(&b'\n') {
                index += 1;
                continue;
            }
            content.push(raw[index]);
            index += 1;
        }
        hasher.update(relative.as_bytes());
        hasher.update(b"\0");
        hasher.update(content.len().to_string().as_bytes());
        hasher.update(b"\0");
        hasher.update(&content);
        hasher.update(b"\n");
    }
    let hex: String = hasher.finalize().iter().map(|byte| format!("{byte:02x}")).collect();
    let mut watched: Vec<PathBuf> = files.into_iter().map(|(_, path)| path).collect();
    watched.extend(watched_dirs);
    Ok(TreeDigest { hex, watched })
}
