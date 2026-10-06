//! Shared helpers of the vector tests: a tiny JSON reader (no external crates), hex codecs for `UBig` / `IBig`
//! and the FNV-1a 64 digest the generator `tools/native_canon_vectors.py` writes.
#![allow(dead_code)]

use std::collections::BTreeMap;
use std::path::PathBuf;

use dashu_int::{IBig, UBig};

#[derive(Clone, Debug, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Int(i64),
    Str(String),
    Arr(Vec<Json>),
    Obj(BTreeMap<String, Json>),
}

impl Json {
    pub fn get(&self, key: &str) -> &Json {
        match self {
            Json::Obj(map) => map.get(key).unwrap_or(&Json::Null),
            _ => &Json::Null,
        }
    }
    pub fn has(&self, key: &str) -> bool {
        matches!(self, Json::Obj(map) if map.contains_key(key))
    }
    pub fn str(&self) -> &str {
        match self {
            Json::Str(text) => text,
            other => panic!("expected a string, found {other:?}"),
        }
    }
    pub fn arr(&self) -> &[Json] {
        match self {
            Json::Arr(items) => items,
            Json::Null => &[],
            other => panic!("expected an array, found {other:?}"),
        }
    }
    pub fn int(&self) -> i64 {
        match self {
            Json::Int(value) => *value,
            other => panic!("expected an integer, found {other:?}"),
        }
    }
    pub fn is_null(&self) -> bool {
        matches!(self, Json::Null)
    }
    pub fn flag(&self) -> bool {
        matches!(self, Json::Bool(true))
    }
    pub fn text(&self) -> String {
        write_json(self)
    }
}

pub fn write_json(value: &Json) -> String {
    match value {
        Json::Null => "null".to_owned(),
        Json::Bool(flag) => flag.to_string(),
        Json::Int(number) => number.to_string(),
        Json::Str(text) => format!("\"{text}\""),
        Json::Arr(items) => format!("[{}]", items.iter().map(write_json).collect::<Vec<_>>().join(",")),
        Json::Obj(map) => format!(
            "{{{}}}",
            map.iter().map(|(key, item)| format!("\"{key}\":{}", write_json(item))).collect::<Vec<_>>().join(",")
        ),
    }
}

struct Parser<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl Parser<'_> {
    fn skip(&mut self) {
        while self.at < self.bytes.len() && self.bytes[self.at].is_ascii_whitespace() {
            self.at += 1;
        }
    }

    fn value(&mut self) -> Json {
        self.skip();
        match self.bytes[self.at] {
            b'n' => {
                self.at += 4;
                Json::Null
            }
            b't' => {
                self.at += 4;
                Json::Bool(true)
            }
            b'f' => {
                self.at += 5;
                Json::Bool(false)
            }
            b'"' => Json::Str(self.string()),
            b'[' => {
                self.at += 1;
                let mut items = Vec::new();
                loop {
                    self.skip();
                    if self.bytes[self.at] == b']' {
                        self.at += 1;
                        return Json::Arr(items);
                    }
                    items.push(self.value());
                    self.skip();
                    if self.bytes[self.at] == b',' {
                        self.at += 1;
                    }
                }
            }
            b'{' => {
                self.at += 1;
                let mut map = BTreeMap::new();
                loop {
                    self.skip();
                    if self.bytes[self.at] == b'}' {
                        self.at += 1;
                        return Json::Obj(map);
                    }
                    let key = self.string();
                    self.skip();
                    assert_eq!(self.bytes[self.at], b':');
                    self.at += 1;
                    let item = self.value();
                    map.insert(key, item);
                    self.skip();
                    if self.bytes[self.at] == b',' {
                        self.at += 1;
                    }
                }
            }
            _ => {
                let start = self.at;
                while self.at < self.bytes.len() && (self.bytes[self.at] == b'-' || self.bytes[self.at].is_ascii_digit()) {
                    self.at += 1;
                }
                Json::Int(std::str::from_utf8(&self.bytes[start..self.at]).unwrap().parse().unwrap())
            }
        }
    }

    fn string(&mut self) -> String {
        assert_eq!(self.bytes[self.at], b'"');
        self.at += 1;
        let mut out = String::new();
        while self.bytes[self.at] != b'"' {
            if self.bytes[self.at] == b'\\' {
                self.at += 1;
            }
            out.push(self.bytes[self.at] as char);
            self.at += 1;
        }
        self.at += 1;
        out
    }
}

pub fn parse_json(text: &str) -> Json {
    Parser { bytes: text.as_bytes(), at: 0 }.value()
}

/// `tests/vectors/<name>.json`.
pub fn load_vectors(name: &str) -> Json {
    let path: PathBuf = [env!("CARGO_MANIFEST_DIR"), "tests", "vectors", &format!("{name}.json")].iter().collect();
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("{}: {error}; run tools/native_canon_vectors.py", path.display()));
    parse_json(&text)
}

pub fn hx(value: &UBig) -> String {
    format!("{value:x}")
}

pub fn hx_signed(value: &IBig) -> String {
    format!("{value:x}")
}

pub fn ub(text: &str) -> UBig {
    UBig::from_str_radix(text, 16).unwrap_or_else(|error| panic!("bad hex {text:?}: {error}"))
}

pub fn ib(text: &str) -> IBig {
    IBig::from_str_radix(text, 16).unwrap_or_else(|error| panic!("bad signed hex {text:?}: {error}"))
}

pub fn fnv1a64(data: &[u8]) -> u64 {
    let mut value: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in data {
        value = (value ^ u64::from(*byte)).wrapping_mul(0x100_0000_01b3);
    }
    value
}

pub fn jstr(text: impl Into<String>) -> Json {
    Json::Str(text.into())
}

pub fn jint(value: u64) -> Json {
    Json::Int(value as i64)
}

pub fn jhex(value: &UBig) -> Json {
    Json::Str(hx(value))
}

pub fn jarr(items: Vec<Json>) -> Json {
    Json::Arr(items)
}
