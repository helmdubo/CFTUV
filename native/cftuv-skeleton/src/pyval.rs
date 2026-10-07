//! The Python values the superlevel layer keys, groups and orders by: [`Val`], an immutable tree of `None`, `bool`, `int`, `str`, `Fraction`, tuples, lists and
//! dataclass instances with the semantics CPython gives them.
//!
//! The planning code of the oracle (`superlevel.py`, `superlevel_closure.py`, `symbolic_*.py`) is written against Python's own data model, and three of its rules
//! decide answers, so a port that replaces the values by "natural" typed structs would change them:
//!
//! * IDENTITY is value equality of nested tuples (`==` between an `ExactIdentityKeyV1` and a plain tuple of the same items is `True`, `Fraction(2) == 2`, `None == None`);
//!   a key is hashed for dictionaries and sets, and the hash is memoised by the key (the oracle's `ExactIdentityKeyV1` does the same), so a [`Val`] computes its hash
//!   once at construction and compares by pointer, then by hash, then by structure;
//! * ORDER by `repr` is the order that defines vertex ids, twin ids and the push order of the queue (`sorted(..., key=repr)` has about sixty live sites), so [`Val::repr`]
//!   prints exactly what CPython prints and caches the text per value (`'10' < '9'`: a numeric order is NOT equivalent);
//! * ORDER by `<` is the order of the tuple comparison of CPython (`tuplerichcompare`): the first index where the items differ by `==` decides, by `<` of those two
//!   items, and an unordered pair (`None` against a tuple, a dataclass against anything) raises `TypeError` there and nowhere earlier. [`py_less`] returns that
//!   as an [`UnorderedPair`]; the oracle's sorts that hit it raise an internal error, which the port reports as a named refusal (it never guesses an order).
//!
//! A value is built once and shared (`Rc`); the tree is never mutated.

use std::borrow::Cow;
use std::cell::OnceCell;
use std::cmp::Ordering;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

use cftuv_canon::fxhash::FxHasher;
use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::error::{SkelError, SkelResult};
use crate::repr::{repr_frac, repr_str};

/// A class or a field name: static in the code of the port, owned when it came over the wire.
pub type Name = Cow<'static, str>;

#[derive(Clone)]
pub struct Val(Rc<Node>);

struct Node {
    hash: u64,
    text: OnceCell<Rc<str>>,
    kind: Kind,
}

enum Kind {
    None,
    Bool(bool),
    Int(IBig),
    Str(Rc<str>),
    Frac(Rat),
    Tuple(Vec<Val>),
    List(Vec<Val>),
    /// A dataclass instance: the class name and the fields in declaration order (fields with `repr=False` are left out by the builder of the value,
    /// as they are by `compare=False` ones).
    Data(Name, Vec<(Name, Val)>),
    /// A member of a `str`-mixin enum: class, member name, value.
    Member(Name, Name, Val),
    /// A `set` or `frozenset` (the flag): the members unique by value and in the canonical order of their `repr`. CPython prints a set in the order of its hash table, which
    /// no key may depend on; the port prints and compares the canonical order, and the seams bring the oracle's sets to the same form.
    Set(Vec<Val>, bool),
    /// A `dict`: the pairs in insertion order.
    Dict(Vec<(Val, Val)>),
}

/// `'<' not supported between instances of ...`: the pair of values that has no order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnorderedPair {
    pub left: &'static str,
    pub right: &'static str,
}

impl UnorderedPair {
    /// The refusal the port answers with where the oracle raises `TypeError` from a sort (an internal error of the oracle, never an answer).
    pub fn refusal(&self, site: &str) -> SkelError {
        SkelError::Unsupported(format!("TypeError in the oracle at {site}: '<' not supported between instances of '{}' and '{}'", self.left, self.right))
    }
}

fn mix(hash: u64, word: u64) -> u64 {
    (hash.rotate_left(5) ^ word).wrapping_mul(0x517c_c1b7_2722_0a95)
}

fn hash_of<T: Hash + ?Sized>(value: &T) -> u64 {
    let mut hasher = FxHasher::default();
    value.hash(&mut hasher);
    hasher.finish()
}

/// The hash of a number as a rational: equal values (`True`, `1`, `Fraction(1, 1)`) hash alike, as they do in Python.
fn number_hash(numerator: &IBig, denominator: &UBig) -> u64 {
    mix(mix(0x6e75_6d62, hash_of(numerator)), hash_of(denominator))
}

fn kind_hash(kind: &Kind) -> u64 {
    match kind {
        Kind::None => 0x4e6f_6e65,
        Kind::Bool(flag) => number_hash(&IBig::from(i8::from(*flag)), &UBig::ONE),
        Kind::Int(number) => number_hash(number, &UBig::ONE),
        Kind::Str(text) => mix(0x7374_72, hash_of(&**text)),
        Kind::Frac(number) => number_hash(number.numerator(), number.denominator()),
        Kind::Tuple(items) => items.iter().fold(mix(0x7475_706c, items.len() as u64), |hash, item| mix(hash, item.0.hash)),
        Kind::List(items) => items.iter().fold(mix(0x6c69_7374, items.len() as u64), |hash, item| mix(hash, item.0.hash)),
        Kind::Data(name, fields) => fields.iter().fold(mix(0x6461_7461, hash_of(&**name)), |hash, (_, item)| mix(hash, item.0.hash)),
        Kind::Member(class, member, _) => mix(mix(0x656e_756d, hash_of(&**class)), hash_of(&**member)),
        Kind::Set(items, frozen) => items.iter().fold(mix(0x7365_74 + u64::from(*frozen), items.len() as u64), |hash, item| mix(hash, item.0.hash)),
        Kind::Dict(pairs) => pairs.iter().fold(mix(0x6469_6374, pairs.len() as u64), |hash, (key, item)| mix(mix(hash, key.0.hash), item.0.hash)),
    }
}

/// The value of a number kind as `(numerator, denominator)`.
fn as_number(kind: &Kind) -> Option<(IBig, UBig)> {
    match kind {
        Kind::Bool(flag) => Some((IBig::from(i8::from(*flag)), UBig::ONE)),
        Kind::Int(number) => Some((number.clone(), UBig::ONE)),
        Kind::Frac(number) => Some((number.numerator().clone(), number.denominator().clone())),
        _ => None,
    }
}

fn type_name(kind: &Kind) -> &'static str {
    match kind {
        Kind::None => "NoneType",
        Kind::Bool(_) => "bool",
        Kind::Int(_) => "int",
        Kind::Str(_) => "str",
        Kind::Frac(_) => "Fraction",
        Kind::Tuple(_) => "tuple",
        Kind::List(_) => "list",
        Kind::Data(_, _) => "dataclass",
        Kind::Member(_, _, _) => "enum",
        Kind::Set(_, false) => "set",
        Kind::Set(_, true) => "frozenset",
        Kind::Dict(_) => "dict",
    }
}

impl Val {
    fn new(kind: Kind) -> Val {
        let hash = kind_hash(&kind);
        Val(Rc::new(Node { hash, text: OnceCell::new(), kind }))
    }

    pub fn none() -> Val {
        Val::new(Kind::None)
    }

    pub fn boolean(flag: bool) -> Val {
        Val::new(Kind::Bool(flag))
    }

    pub fn int(number: i64) -> Val {
        Val::new(Kind::Int(IBig::from(number)))
    }

    pub fn big(number: IBig) -> Val {
        Val::new(Kind::Int(number))
    }

    /// A string; the repr of a character outside ASCII depends on the Unicode database, and the kernel's texts are ASCII.
    pub fn str(text: &str) -> Val {
        assert!(text.is_ascii(), "a non-ASCII string has no byte-exact repr in the port: {text:?}");
        Val::new(Kind::Str(Rc::from(text)))
    }

    pub fn frac(number: Rat) -> Val {
        Val::new(Kind::Frac(number))
    }

    pub fn tuple(items: Vec<Val>) -> Val {
        Val::new(Kind::Tuple(items))
    }

    pub fn list(items: Vec<Val>) -> Val {
        Val::new(Kind::List(items))
    }

    pub fn data(name: impl Into<Name>, fields: Vec<(&'static str, Val)>) -> Val {
        Val::new(Kind::Data(name.into(), fields.into_iter().map(|(field, item)| (Cow::Borrowed(field), item)).collect()))
    }

    /// A dataclass instance with owned names (the decoder of the wire).
    pub fn data_named(name: String, fields: Vec<(String, Val)>) -> Val {
        Val::new(Kind::Data(Cow::Owned(name), fields.into_iter().map(|(field, item)| (Cow::Owned(field), item)).collect()))
    }

    pub fn member(class: impl Into<Name>, member: impl Into<Name>, value: Val) -> Val {
        Val::new(Kind::Member(class.into(), member.into(), value))
    }

    /// A `set` (`frozen == false`) or `frozenset` of the items: equal items once (the first kept), in the canonical order of their `repr`.
    pub fn set(items: Vec<Val>, frozen: bool) -> Val {
        if items.len() <= 1 {
            return Val::new(Kind::Set(items, frozen));
        }
        let mut unique: Vec<Val> = Vec::with_capacity(items.len());
        let mut seen: std::collections::HashSet<Val> = std::collections::HashSet::new();
        for item in items {
            if seen.insert(item.clone()) {
                unique.push(item);
            }
        }
        unique.sort_by(|left, right| left.repr().as_bytes().cmp(right.repr().as_bytes()));
        Val::new(Kind::Set(unique, frozen))
    }

    /// A `dict` of the pairs in insertion order (the keys are the caller's to keep unique).
    pub fn dict(pairs: Vec<(Val, Val)>) -> Val {
        Val::new(Kind::Dict(pairs))
    }

    /// The members of a set (in the canonical order).
    pub fn set_items(&self) -> Option<&[Val]> {
        match &self.0.kind {
            Kind::Set(items, _) => Some(items),
            _ => None,
        }
    }

    /// A tuple of ints (an edge key `(x0, y0, x1, y1)` or `(x, y, x, y, ordinal)`, a ray, a list of vertex ids).
    pub fn ints(items: &[i64]) -> Val {
        Val::tuple(items.iter().map(|item| Val::int(*item)).collect())
    }

    /// `((radicand, coefficient), ...)`: the `terms` of a sum as the oracle's keys hold them (a coefficient is a `Fraction`, an `int` where the sum kept one).
    pub fn terms_of(sum: &SqrtSum) -> Val {
        Val::tuple(
            sum.terms()
                .iter()
                .map(|term| Val::tuple(vec![Val::big(IBig::from(term.radicand.clone())), if term.coef.is_py_int() { Val::big(term.coef.value().numerator().clone()) } else { Val::frac(term.coef.value().clone()) }]))
                .collect(),
        )
    }

    pub fn is_none(&self) -> bool {
        matches!(self.0.kind, Kind::None)
    }

    pub fn as_int(&self) -> Option<&IBig> {
        match &self.0.kind {
            Kind::Int(number) => Some(number),
            _ => None,
        }
    }

    pub fn as_i64(&self) -> Option<i64> {
        self.as_int().and_then(|number| i64::try_from(number).ok())
    }

    pub fn as_str(&self) -> Option<&str> {
        match &self.0.kind {
            Kind::Str(text) => Some(text),
            _ => None,
        }
    }

    pub fn as_frac(&self) -> Option<&Rat> {
        match &self.0.kind {
            Kind::Frac(number) => Some(number),
            _ => None,
        }
    }

    pub fn as_bool(&self) -> Option<bool> {
        match &self.0.kind {
            Kind::Bool(flag) => Some(*flag),
            _ => None,
        }
    }

    /// The items of a tuple or a list.
    pub fn items(&self) -> Option<&[Val]> {
        match &self.0.kind {
            Kind::Tuple(items) | Kind::List(items) => Some(items),
            _ => None,
        }
    }

    pub fn is_tuple(&self) -> bool {
        matches!(self.0.kind, Kind::Tuple(_))
    }

    /// `value[index]` of a tuple or a list.
    pub fn get(&self, index: usize) -> Option<&Val> {
        self.items().and_then(|items| items.get(index))
    }

    /// The field of a dataclass instance by name.
    pub fn field(&self, name: &str) -> Option<&Val> {
        match &self.0.kind {
            Kind::Data(_, fields) => fields.iter().find(|(field, _)| field == name).map(|(_, item)| item),
            _ => None,
        }
    }

    /// The class name and the fields of a dataclass instance.
    pub fn as_data(&self) -> Option<(&str, &[(Name, Val)])> {
        match &self.0.kind {
            Kind::Data(name, fields) => Some((name, fields)),
            _ => None,
        }
    }

    pub fn as_member(&self) -> Option<(&str, &str, &Val)> {
        match &self.0.kind {
            Kind::Member(class, member, value) => Some((class, member, value)),
            _ => None,
        }
    }

    /// A copy of a dataclass instance with one field replaced (`dataclasses.replace`).
    pub fn replaced(&self, name: &str, value: Val) -> Option<Val> {
        match &self.0.kind {
            Kind::Data(class, fields) if fields.iter().any(|(field, _)| field == name) => {
                Some(Val::new(Kind::Data(class.clone(), fields.iter().map(|(field, item)| (field.clone(), if field == name { value.clone() } else { item.clone() })).collect())))
            }
            _ => None,
        }
    }

    pub fn hash_value(&self) -> u64 {
        self.0.hash
    }

    /// `repr(value)`, byte for byte, computed once per value.
    pub fn repr(&self) -> Rc<str> {
        Rc::clone(self.0.text.get_or_init(|| {
            let mut out = String::new();
            write_repr(&self.0.kind, &mut out);
            Rc::from(out)
        }))
    }

    pub fn same(&self, other: &Val) -> bool {
        Rc::ptr_eq(&self.0, &other.0)
    }
}

fn write_items(out: &mut String, open: char, close: char, items: &[Val], one_tuple: bool) {
    out.push(open);
    for (index, item) in items.iter().enumerate() {
        if index > 0 {
            out.push_str(", ");
        }
        out.push_str(&item.repr());
    }
    if one_tuple && items.len() == 1 {
        out.push(',');
    }
    out.push(close);
}

fn write_repr(kind: &Kind, out: &mut String) {
    match kind {
        Kind::None => out.push_str("None"),
        Kind::Bool(flag) => out.push_str(if *flag { "True" } else { "False" }),
        Kind::Int(number) => out.push_str(&number.to_string()),
        Kind::Str(text) => {
            // ASCII by construction (`Val::str`); the call cannot refuse.
            let _ = repr_str(text, out);
        }
        Kind::Frac(number) => repr_frac(number, out),
        Kind::Tuple(items) => write_items(out, '(', ')', items, true),
        Kind::List(items) => write_items(out, '[', ']', items, false),
        Kind::Data(name, fields) => {
            out.push_str(name);
            out.push('(');
            for (index, (field, item)) in fields.iter().enumerate() {
                if index > 0 {
                    out.push_str(", ");
                }
                out.push_str(field);
                out.push('=');
                out.push_str(&item.repr());
            }
            out.push(')');
        }
        Kind::Member(class, member, value) => {
            out.push('<');
            out.push_str(class);
            out.push('.');
            out.push_str(member);
            out.push_str(": ");
            out.push_str(&value.repr());
            out.push('>');
        }
        Kind::Set(items, frozen) => match (items.is_empty(), frozen) {
            (true, false) => out.push_str("set()"),
            (true, true) => out.push_str("frozenset()"),
            (false, _) => {
                if *frozen {
                    out.push_str("frozenset(");
                }
                write_items(out, '{', '}', items, false);
                if *frozen {
                    out.push(')');
                }
            }
        },
        Kind::Dict(pairs) => {
            out.push('{');
            for (index, (key, item)) in pairs.iter().enumerate() {
                if index > 0 {
                    out.push_str(", ");
                }
                out.push_str(&key.repr());
                out.push_str(": ");
                out.push_str(&item.repr());
            }
            out.push('}');
        }
    }
}

/// `left == right` of two values (never an error: an unrelated pair is simply not equal).
pub fn py_eq(left: &Val, right: &Val) -> bool {
    if left.same(right) {
        return true;
    }
    if left.0.hash != right.0.hash {
        return false;
    }
    match (&left.0.kind, &right.0.kind) {
        (Kind::None, Kind::None) => true,
        (Kind::Str(first), Kind::Str(second)) => first == second,
        (Kind::Tuple(first), Kind::Tuple(second)) | (Kind::List(first), Kind::List(second)) => first.len() == second.len() && first.iter().zip(second).all(|(a, b)| py_eq(a, b)),
        (Kind::Data(first_name, first), Kind::Data(second_name, second)) => {
            first_name == second_name && first.len() == second.len() && first.iter().zip(second).all(|((name_a, a), (name_b, b))| name_a == name_b && py_eq(a, b))
        }
        (Kind::Member(first_class, first_member, _), Kind::Member(second_class, second_member, _)) => first_class == second_class && first_member == second_member,
        (Kind::Set(first, first_frozen), Kind::Set(second, second_frozen)) => first_frozen == second_frozen && first.len() == second.len() && first.iter().zip(second).all(|(a, b)| py_eq(a, b)),
        (Kind::Dict(first), Kind::Dict(second)) => first.len() == second.len() && first.iter().zip(second).all(|((key_a, a), (key_b, b))| py_eq(key_a, key_b) && py_eq(a, b)),
        (first, second) => match (as_number(first), as_number(second)) {
            (Some((a_n, a_d)), Some((b_n, b_d))) => a_n == b_n && a_d == b_d,
            _ => false,
        },
    }
}

impl PartialEq for Val {
    fn eq(&self, other: &Val) -> bool {
        py_eq(self, other)
    }
}

impl Eq for Val {}

impl Hash for Val {
    fn hash<H: Hasher>(&self, state: &mut H) {
        state.write_u64(self.0.hash);
    }
}

impl std::fmt::Debug for Val {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.repr())
    }
}

/// `left < right` of two values with CPython's rules (see the module note): numbers by value, strings by code point, tuples and lists by the first index that
/// differs, and an unordered pair is an [`UnorderedPair`].
pub fn py_less(left: &Val, right: &Val) -> Result<bool, UnorderedPair> {
    Ok(py_compare(left, right)? == Ordering::Less)
}

fn unordered(left: &Kind, right: &Kind) -> UnorderedPair {
    UnorderedPair { left: type_name(left), right: type_name(right) }
}

/// The order of two values, `Equal` only when `left < right` and `right < left` are both false for a total order: sorting needs only `<`, which this is built from.
fn py_compare(left: &Val, right: &Val) -> Result<Ordering, UnorderedPair> {
    match (&left.0.kind, &right.0.kind) {
        (Kind::Str(first), Kind::Str(second)) => Ok(first.as_bytes().cmp(second.as_bytes())),
        (Kind::Tuple(first), Kind::Tuple(second)) | (Kind::List(first), Kind::List(second)) => {
            for (a, b) in first.iter().zip(second) {
                if !py_eq(a, b) {
                    return Ok(if py_less(a, b)? { Ordering::Less } else { Ordering::Greater });
                }
            }
            Ok(first.len().cmp(&second.len()))
        }
        (Kind::Member(_, _, first), Kind::Member(_, _, second)) => py_compare(first, second),
        (first, second) => match (as_number(first), as_number(second)) {
            (Some((a_n, a_d)), Some((b_n, b_d))) => {
                let (a_d, b_d) = (IBig::from(a_d), IBig::from(b_d));
                Ok((a_n * &b_d).cmp(&(b_n * &a_d)))
            }
            _ => Err(unordered(first, second)),
        },
    }
}

/// `exact_identity.identity_order_key(value)` (the oracle of commit 3da8cdd): the key that orders identity keys whose slots may be `None`: `None` is `(1,)`, a tuple is
/// `(0, (keys of its items))`, anything else `(0, value)`, so a `None` sorts AFTER any value of the same slot and the order of keys without a `None` is the plain one.
pub fn identity_order_val(value: &Val) -> Val {
    if value.is_none() {
        return Val::tuple(vec![Val::int(1)]);
    }
    if value.is_tuple() {
        let items: Vec<Val> = value.items().unwrap_or(&[]).iter().map(identity_order_val).collect();
        return Val::tuple(vec![Val::int(0), Val::tuple(items)]);
    }
    Val::tuple(vec![Val::int(0), value.clone()])
}

/// `sorted(items, key=key)` for a key that CPython orders with `<` (a stable sort in CPython 3.11's comparison sequence). An unordered pair that the sort reaches is
/// the oracle's `TypeError`.
pub fn sorted_by_val<T: Clone>(items: &[T], key: impl Fn(&T) -> Val) -> Result<Vec<T>, UnorderedPair> {
    if items.len() < 2 {
        return Ok(items.to_vec());
    }
    let keys: Vec<Val> = items.iter().map(&key).collect();
    let mut failure: Option<UnorderedPair> = None;
    let order = {
        let mut less = |left: &usize, right: &usize| -> cftuv_clip::error::ClipResult<bool> {
            match py_less(&keys[*left], &keys[*right]) {
                Ok(answer) => Ok(answer),
                Err(error) => {
                    failure = Some(error);
                    Err(cftuv_clip::error::ClipError::Value("unordered"))
                }
            }
        };
        cftuv_clip::cpython311::sort_by_less((0..items.len()).collect(), &mut less)
    };
    match order {
        Ok(order) => Ok(order.into_iter().map(|index| items[index].clone()).collect()),
        Err(_) => Err(failure.unwrap_or(UnorderedPair { left: "?", right: "?" })),
    }
}

/// [`sorted_by_val`] with the refusal of the port in place of the pair.
pub fn sorted_by_val_or_refuse<T: Clone>(items: &[T], key: impl Fn(&T) -> Val, site: &str) -> SkelResult<Vec<T>> {
    sorted_by_val(items, key).map_err(|pair| pair.refusal(site))
}

/// `sorted(items, key=repr)` where the key is the `repr` of a value: a total order, so the standard stable sort is the oracle's.
pub fn sorted_by_repr<T: Clone>(items: &[T], key: impl Fn(&T) -> Val) -> Vec<T> {
    let mut keyed: Vec<(Rc<str>, T)> = items.iter().map(|item| (key(item).repr(), item.clone())).collect();
    keyed.sort_by(|left, right| left.0.as_bytes().cmp(right.0.as_bytes()));
    keyed.into_iter().map(|(_, item)| item).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use cftuv_core::sqrt_sum::Term;

    fn half() -> Rat {
        Rat::new(IBig::from(1), IBig::from(2)).unwrap()
    }

    fn pair(first: Val, second: Val) -> Val {
        Val::tuple(vec![first, second])
    }

    #[test]
    fn equal_numbers_are_one_key_and_the_other_kinds_are_not() {
        assert_eq!(Val::int(2), Val::frac(Rat::from_int(IBig::from(2))));
        assert_eq!(Val::int(2).hash_value(), Val::frac(Rat::from_int(IBig::from(2))).hash_value());
        assert_eq!(Val::boolean(true), Val::int(1));
        assert_ne!(Val::int(2), Val::str("2"));
        assert_ne!(Val::none(), Val::tuple(vec![]));
        assert_eq!(Val::none(), Val::none());
        assert_eq!(pair(Val::int(1), Val::none()), pair(Val::int(1), Val::none()));
        assert_ne!(Val::tuple(vec![Val::int(1)]), Val::list(vec![Val::int(1)]));
    }

    #[test]
    fn the_repr_is_the_one_of_cpython() {
        assert_eq!(&*Val::tuple(vec![]).repr(), "()");
        assert_eq!(&*Val::tuple(vec![Val::int(1)]).repr(), "(1,)");
        assert_eq!(&*pair(Val::int(-3), Val::none()).repr(), "(-3, None)");
        assert_eq!(&*Val::frac(half()).repr(), "Fraction(1, 2)");
        assert_eq!(&*Val::str("it's").repr(), "\"it's\"");
        let data = Val::data("JunctionRefV1", vec![("kind", Val::str("EXISTING")), ("key", Val::ints(&[3]))]);
        assert_eq!(&*data.repr(), "JunctionRefV1(kind='EXISTING', key=(3,))");
        assert_eq!(&*Val::member("EventKind", "EDGE", Val::str("EDGE")).repr(), "<EventKind.EDGE: 'EDGE'>");
        let sum = SqrtSum::from_terms(vec![Term { radicand: UBig::from(2u8), coef: cftuv_core::rat::Coef::fraction(half()) }]).unwrap();
        assert_eq!(&*Val::terms_of(&sum).repr(), "((2, Fraction(1, 2)),)");
    }

    #[test]
    fn tuples_order_by_the_first_difference_and_an_unordered_pair_raises_only_there() {
        assert!(py_less(&Val::ints(&[1, 2]), &Val::ints(&[1, 3])).unwrap());
        assert!(py_less(&Val::ints(&[1]), &Val::ints(&[1, 0])).unwrap());
        assert!(!py_less(&Val::ints(&[1, 2]), &Val::ints(&[1, 2])).unwrap());
        // the first items differ, so the `None`s further right are never compared
        assert!(py_less(&pair(Val::int(1), Val::none()), &pair(Val::int(2), Val::tuple(vec![]))).unwrap());
        // the first items are equal, then `None < ()` is asked
        assert!(py_less(&pair(Val::int(1), Val::none()), &pair(Val::int(1), Val::tuple(vec![]))).is_err());
        // equal `None`s are equal: the next items decide
        assert!(py_less(&pair(Val::none(), Val::int(1)), &pair(Val::none(), Val::int(2))).unwrap());
        assert!(py_less(&Val::int(1), &Val::frac(half())).is_ok());
        assert!(!py_less(&Val::int(1), &Val::frac(half())).unwrap());
        assert!(py_less(&Val::str("10"), &Val::str("9")).unwrap());
    }

    #[test]
    fn sorting_by_a_key_that_mixes_none_and_tuples_refuses_exactly_where_python_raises() {
        let items = vec![pair(Val::int(1), Val::none()), pair(Val::int(1), Val::tuple(vec![]))];
        assert!(sorted_by_val(&items, Val::clone).is_err());
        let fine = vec![pair(Val::int(2), Val::none()), pair(Val::int(1), Val::tuple(vec![]))];
        let sorted = sorted_by_val(&fine, Val::clone).unwrap();
        assert_eq!(sorted[0], fine[1]);
        assert_eq!(sorted_by_val(&items[..1], Val::clone).unwrap().len(), 1);
    }

    #[test]
    fn the_identity_order_key_puts_none_after_any_value_of_the_slot_and_leaves_the_rest_alone() {
        let with_point = pair(Val::int(1), Val::tuple(vec![Val::int(5)]));
        let without = pair(Val::int(1), Val::none());
        assert!(py_less(&identity_order_val(&with_point), &identity_order_val(&without)).unwrap());
        assert!(!py_less(&identity_order_val(&without), &identity_order_val(&with_point)).unwrap());
        // no `TypeError` where the plain order raised
        assert!(py_less(&with_point, &without).is_err());
        // the order of keys without a None is the plain one
        let (low, high) = (pair(Val::int(1), Val::int(2)), pair(Val::int(1), Val::int(3)));
        assert_eq!(py_less(&low, &high).unwrap(), py_less(&identity_order_val(&low), &identity_order_val(&high)).unwrap());
        assert_eq!(py_less(&high, &low).unwrap(), py_less(&identity_order_val(&high), &identity_order_val(&low)).unwrap());
    }

    #[test]
    fn sorting_by_repr_puts_ten_before_nine() {
        let numbers = [Val::int(9), Val::int(10), Val::int(100)];
        let sorted = sorted_by_repr(&numbers, Val::clone);
        assert_eq!(sorted.iter().map(|value| value.as_i64().unwrap()).collect::<Vec<_>>(), vec![10, 100, 9]);
    }

    #[test]
    fn a_set_prints_in_the_canonical_order_of_its_members_and_is_equal_whatever_the_order_it_was_given_in() {
        let members = |order: [i64; 3]| order.iter().map(|each| Val::int(*each)).collect::<Vec<Val>>();
        let (first, second) = (Val::set(members([9, 10, 2]), false), Val::set(members([2, 9, 10]), false));
        assert_eq!(first, second);
        assert_eq!(&*first.repr(), "{10, 2, 9}");
        assert_eq!(&*Val::set(members([1, 1, 1]), true).repr(), "frozenset({1})");
        assert_eq!(&*Val::set(Vec::new(), false).repr(), "set()");
        assert_eq!(&*Val::set(Vec::new(), true).repr(), "frozenset()");
        assert_ne!(Val::set(members([1, 2, 3]), true), Val::set(members([1, 2, 3]), false));
        assert!(py_less(&first, &second).is_err());
    }

    #[test]
    fn a_dict_keeps_the_order_it_was_given_and_prints_like_one() {
        let pairs = vec![(Val::int(2), Val::str("b")), (Val::int(1), Val::tuple(vec![Val::none()]))];
        let dict = Val::dict(pairs.clone());
        assert_eq!(&*dict.repr(), "{2: 'b', 1: (None,)}");
        assert_eq!(&*Val::dict(Vec::new()).repr(), "{}");
        assert_ne!(dict, Val::dict(pairs.into_iter().rev().collect()));
    }

    #[test]
    fn a_dataclass_is_equal_by_class_and_fields_and_unordered() {
        let first = Val::data("A", vec![("x", Val::int(1))]);
        let second = Val::data("A", vec![("x", Val::int(1))]);
        assert_eq!(first, second);
        assert_ne!(first, Val::data("B", vec![("x", Val::int(1))]));
        assert!(py_less(&first, &second).is_err());
        assert_eq!(first.field("x").and_then(Val::as_i64), Some(1));
        assert_eq!(first.replaced("x", Val::int(5)).unwrap().field("x").and_then(Val::as_i64), Some(5));
    }
}
