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
use std::cell::{Cell, OnceCell, RefCell};
use std::cmp::Ordering;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

use cftuv_canon::fxhash::FxHasher;
use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SqrtSum, Term};

use crate::error::{SkelError, SkelResult};
use crate::repr::{push_ibig, push_ubig, repr_frac, repr_str};

/// A class or a field name: static in the code of the port, owned when it came over the wire.
pub type Name = Cow<'static, str>;

#[derive(Clone)]
pub struct Val(Rc<Node>);

struct Node {
    hash: u64,
    text: OnceCell<Rc<str>>,
    /// The first [`HEAD_BYTES`] bytes (a little more) of the text of a value whose text is longer: what a comparison of two texts reads when it is decided that early (see
    /// [`repr_cmp`]). A text shorter than that is the whole text and lives in `text`; this one is never set then.
    head: OnceCell<Box<str>>,
    /// How many times a text that was being written reached this value without its own text: the first time it is written through (no text is made for it), the
    /// second makes and keeps its own (see [`write_child`]).
    visits: Cell<u8>,
    kind: Kind,
}

thread_local! {
    /// The buffer a root text is written in (taken for the write and put back, so a text made inside a text brings its own).
    static SCRATCH: RefCell<String> = const { RefCell::new(String::new()) };
}

/// The items of a tuple or a list. Up to four are kept in the node itself (a tuple of a key is almost always a pair, a triple or a quadruple): one allocation for the value,
/// not two.
enum Items {
    Zero,
    One(Val),
    Two([Val; 2]),
    Three([Val; 3]),
    Four([Val; 4]),
    Many(Vec<Val>),
    /// `((radicand, coefficient), ...)`, the terms of a sum ([`Val::terms_of`]): the tuple of pairs of numbers it stands for is made of the terms when somebody reads it as
    /// a sequence of values (the hash, the equality of two of these and the text are made from the terms themselves).
    Terms(Rc<[Term]>, OnceCell<Box<[Val]>>),
}

impl Items {
    fn from_array<const N: usize>(items: [Val; N]) -> Items {
        if N > 4 {
            return Items::Many(Vec::from(items));
        }
        let mut each = items.into_iter();
        let mut next = || each.next().expect("as many items as the array has");
        match N {
            0 => Items::Zero,
            1 => Items::One(next()),
            2 => Items::Two([next(), next()]),
            3 => Items::Three([next(), next(), next()]),
            _ => Items::Four([next(), next(), next(), next()]),
        }
    }

    /// How many items (the terms of a sum are not made into values to be counted).
    fn len(&self) -> usize {
        match self {
            Items::Zero => 0,
            Items::One(_) => 1,
            Items::Two(_) => 2,
            Items::Three(_) => 3,
            Items::Four(_) => 4,
            Items::Many(many) => many.len(),
            Items::Terms(terms, _) => terms.len(),
        }
    }

    fn from_vec(items: Vec<Val>) -> Items {
        let count = items.len();
        if !(1..=4).contains(&count) {
            return if count == 0 { Items::Zero } else { Items::Many(items) };
        }
        let mut each = items.into_iter();
        let mut next = || each.next().expect("as many items as the length said");
        match count {
            1 => Items::One(next()),
            2 => Items::Two([next(), next()]),
            3 => Items::Three([next(), next(), next()]),
            _ => Items::Four([next(), next(), next(), next()]),
        }
    }
}

impl std::ops::Deref for Items {
    type Target = [Val];

    fn deref(&self) -> &[Val] {
        match self {
            Items::Zero => &[],
            Items::One(only) => std::slice::from_ref(only),
            Items::Two(pair) => pair,
            Items::Three(triple) => triple,
            Items::Four(quadruple) => quadruple,
            Items::Many(many) => many,
            Items::Terms(terms, made) => made.get_or_init(|| terms.iter().map(pair_of_term).collect()),
        }
    }
}

/// `(radicand, coefficient)` of a term as the values the oracle's keys hold.
fn pair_of_term(term: &Term) -> Val {
    Val::tuple_of([Val::big(IBig::from(term.radicand.clone())), if term.coef.is_py_int() { Val::big(term.coef.value().numerator().clone()) } else { Val::frac(term.coef.value().clone()) }])
}

/// The hash of the tuple `((radicand, coefficient), ...)` of the values [`pair_of_term`] makes of the terms, made from the terms (the hash of a tuple is a fold of the hashes of its
/// items, and the hash of a number is the one of its value, `True`, `1` and `Fraction(1, 1)` alike).
fn terms_hash(terms: &[Term]) -> u64 {
    let pair = |term: &Term| {
        let radicand = number_hash(&IBig::from(term.radicand.clone()), &UBig::ONE);
        let coefficient = if term.coef.is_py_int() { number_hash(term.coef.value().numerator(), &UBig::ONE) } else { number_hash(term.coef.value().numerator(), term.coef.value().denominator()) };
        mix(mix(mix(0x7475_706c, 2), radicand), coefficient)
    };
    terms.iter().fold(mix(0x7475_706c, terms.len() as u64), |hash, term| mix(hash, pair(term)))
}

impl<'a> IntoIterator for &'a Items {
    type Item = &'a Val;
    type IntoIter = std::slice::Iter<'a, Val>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

/// The values every key holds again and again (`None`, `True`, `False`, the ints a key is made of when it is small), one node each per thread: a value is immutable and a
/// node that two keys share is a node that is formatted, hashed and compared once.
struct Common {
    none: Val,
    yes: Val,
    no: Val,
    ints: Vec<Val>,
}

/// The smallest and the largest int that is shared.
const COMMON_INTS: std::ops::RangeInclusive<i64> = -1..=255;

/// The longest string and the most strings that are shared.
const SHARED_STR_BYTES: usize = 32;
const SHARED_STRS: usize = 256;

thread_local! {
    static STRS: RefCell<std::collections::HashMap<Box<str>, Val, cftuv_canon::fxhash::FxBuild>> = RefCell::new(std::collections::HashMap::default());
}

thread_local! {
    static COMMON: Common = Common {
        none: Val::new(Kind::None),
        yes: Val::new(Kind::Bool(true)),
        no: Val::new(Kind::Bool(false)),
        ints: COMMON_INTS.map(|number| Val::new(Kind::Int(IBig::from(number)))).collect(),
    };
}

enum Kind {
    None,
    Bool(bool),
    Int(IBig),
    Str(Rc<str>),
    Frac(Rat),
    Tuple(Items),
    List(Items),
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
        Kind::Tuple(Items::Terms(terms, _)) => terms_hash(terms),
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
    #[cfg_attr(feature = "profile", track_caller)]
    fn new(kind: Kind) -> Val {
        crate::profile::count(crate::profile::Stat::ValNew, 1);
        #[cfg(feature = "profile")]
        crate::profile::count_site(std::panic::Location::caller());
        #[cfg(feature = "profile")]
        crate::profile::count(
            match &kind {
                Kind::None => crate::profile::Stat::ValNone,
                Kind::Bool(_) => crate::profile::Stat::ValBool,
                Kind::Int(number) if IBig::from(-1) <= *number && *number < IBig::from(256) => crate::profile::Stat::ValSmallInt,
                Kind::Int(_) => crate::profile::Stat::ValBigInt,
                Kind::Str(_) => crate::profile::Stat::ValStr,
                Kind::Frac(_) => crate::profile::Stat::ValFrac,
                Kind::Tuple(_) => crate::profile::Stat::ValTuple,
                Kind::Data(_, _) => crate::profile::Stat::ValData,
                _ => crate::profile::Stat::ValOther,
            },
            1,
        );
        let hash = kind_hash(&kind);
        Val(Rc::new(Node { hash, text: OnceCell::new(), head: OnceCell::new(), visits: Cell::new(0), kind }))
    }

    pub fn none() -> Val {
        COMMON.with(|common| common.none.clone())
    }

    pub fn boolean(flag: bool) -> Val {
        COMMON.with(|common| if flag { common.yes.clone() } else { common.no.clone() })
    }

    #[cfg_attr(feature = "profile", track_caller)]
    pub fn int(number: i64) -> Val {
        if COMMON_INTS.contains(&number) {
            return COMMON.with(|common| common.ints[(number + 1) as usize].clone());
        }
        Val::new(Kind::Int(IBig::from(number)))
    }

    #[cfg_attr(feature = "profile", track_caller)]
    pub fn big(number: IBig) -> Val {
        match i64::try_from(&number) {
            Ok(small) => Val::int(small),
            Err(_) => Val::new(Kind::Int(number)),
        }
    }

    /// A string; the repr of a character outside ASCII depends on the Unicode database, and the kernel's texts are ASCII.
    #[cfg_attr(feature = "profile", track_caller)]
    pub fn str(text: &str) -> Val {
        assert!(text.is_ascii(), "a non-ASCII string has no byte-exact repr in the port: {text:?}");
        if text.len() > SHARED_STR_BYTES {
            return Val::new(Kind::Str(Rc::from(text)));
        }
        // the names and tags of the planning layer (`EXISTING`, `BIRTH`, `CONTACT`, ...) are a few dozen texts that every key holds again and again: one node each per thread, as
        // the small ints are
        let found = STRS.with(|table| table.borrow().get(text).cloned());
        if let Some(found) = found {
            return found;
        }
        let made = Val::new(Kind::Str(Rc::from(text)));
        STRS.with(|table| {
            let mut table = table.borrow_mut();
            if table.len() < SHARED_STRS {
                table.insert(Box::from(text), made.clone());
            }
        });
        made
    }

    #[cfg_attr(feature = "profile", track_caller)]
    pub fn frac(number: Rat) -> Val {
        Val::new(Kind::Frac(number))
    }

    #[cfg_attr(feature = "profile", track_caller)]
    pub fn tuple(items: Vec<Val>) -> Val {
        Val::new(Kind::Tuple(Items::from_vec(items)))
    }

    /// A tuple of the items of an array: up to four are kept in the node, no vector is made.
    #[cfg_attr(feature = "profile", track_caller)]
    pub fn tuple_of<const N: usize>(items: [Val; N]) -> Val {
        Val::new(Kind::Tuple(Items::from_array(items)))
    }

    #[cfg_attr(feature = "profile", track_caller)]
    pub fn list(items: Vec<Val>) -> Val {
        Val::new(Kind::List(Items::from_vec(items)))
    }

    #[cfg_attr(feature = "profile", track_caller)]
    pub fn data(name: impl Into<Name>, fields: Vec<(&'static str, Val)>) -> Val {
        Val::new(Kind::Data(name.into(), fields.into_iter().map(|(field, item)| (Cow::Borrowed(field), item)).collect()))
    }

    /// A dataclass instance with owned names (the decoder of the wire).
    #[cfg_attr(feature = "profile", track_caller)]
    pub fn data_named(name: String, fields: Vec<(String, Val)>) -> Val {
        Val::new(Kind::Data(Cow::Owned(name), fields.into_iter().map(|(field, item)| (Cow::Owned(field), item)).collect()))
    }

    #[cfg_attr(feature = "profile", track_caller)]
    pub fn member(class: impl Into<Name>, member: impl Into<Name>, value: Val) -> Val {
        Val::new(Kind::Member(class.into(), member.into(), value))
    }

    /// A `set` (`frozen == false`) or `frozenset` of the items: equal items once (the first kept), in the canonical order of their `repr`.
    #[cfg_attr(feature = "profile", track_caller)]
    pub fn set(items: Vec<Val>, frozen: bool) -> Val {
        if items.len() <= 1 {
            return Val::new(Kind::Set(items, frozen));
        }
        let mut unique: Vec<Val> = Vec::with_capacity(items.len());
        let mut seen: std::collections::HashSet<Val, cftuv_canon::fxhash::FxBuild> = std::collections::HashSet::default();
        for item in items {
            if seen.insert(item.clone()) {
                unique.push(item);
            }
        }
        unique.sort_by(repr_cmp);
        Val::new(Kind::Set(unique, frozen))
    }

    /// A `dict` of the pairs in insertion order (the keys are the caller's to keep unique).
    #[cfg_attr(feature = "profile", track_caller)]
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
    #[cfg_attr(feature = "profile", track_caller)]
    pub fn ints(items: &[i64]) -> Val {
        Val::tuple(items.iter().map(|item| Val::int(*item)).collect())
    }

    /// `((radicand, coefficient), ...)`: the `terms` of a sum as the oracle's keys hold them (a coefficient is a `Fraction`, an `int` where the sum kept one).
    #[cfg_attr(feature = "profile", track_caller)]
    pub fn terms_of(sum: &SqrtSum) -> Val {
        crate::profile::count(crate::profile::Stat::TermsOf, 1);
        let terms = sum.terms();
        if terms.is_empty() {
            return Val::tuple(Vec::new());
        }
        Val::new(Kind::Tuple(Items::Terms(Rc::from(terms), OnceCell::new())))
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

    /// The terms of a value made by [`Val::terms_of`] (the pairs are not made of them to be read): `None` for any other value.
    pub fn as_terms(&self) -> Option<&[Term]> {
        match &self.0.kind {
            Kind::Tuple(Items::Terms(terms, _)) => Some(terms),
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
            let mut out = SCRATCH.with(RefCell::take);
            out.clear();
            write_repr(&self.0.kind, &mut out);
            crate::profile::count(crate::profile::Stat::ReprBuilds, 1);
            crate::profile::count_seen(crate::profile::Stat::ReprRoots, crate::profile::Stat::ReprRootsRepeated, || self.0.hash);
            crate::profile::count(crate::profile::Stat::ReprBytes, out.len() as u64);
            let text: Rc<str> = Rc::from(out.as_str());
            SCRATCH.with(|slot| *slot.borrow_mut() = out);
            text
        }))
    }

    /// The text as far as a comparison needs it first: the whole text where it is known or short, else its first [`HEAD_BYTES`] bytes, made by the same writer and stopped
    /// there (never the whole text of a value that is only compared).
    fn head(&self) -> Head<'_> {
        if let Some(text) = self.0.text.get() {
            return Head { text, complete: true };
        }
        if let Some(head) = self.0.head.get() {
            return Head { text: head, complete: false };
        }
        let mut sink = Bounded { out: HEAD_SCRATCH.with(RefCell::take), limit: HEAD_BYTES, stopped: false };
        sink.out.clear();
        write_repr(&self.0.kind, &mut sink);
        crate::profile::count(crate::profile::Stat::ReprHeads, 1);
        let Bounded { out, stopped, .. } = sink;
        if stopped {
            let _ = self.0.head.set(Box::from(out.as_str()));
        } else {
            let _ = self.0.text.set(Rc::from(out.as_str()));
        }
        HEAD_SCRATCH.with(|slot| *slot.borrow_mut() = out);
        match (self.0.text.get(), self.0.head.get()) {
            (Some(text), _) => Head { text, complete: true },
            (None, Some(head)) => Head { text: head, complete: false },
            (None, None) => Head { text: "", complete: false },
        }
    }

    pub fn same(&self, other: &Val) -> bool {
        Rc::ptr_eq(&self.0, &other.0)
    }
}

/// How many bytes of a text [`repr_cmp`] looks at before it asks for the whole text. Two values of the planning layer differ, as a rule, within their first hundred bytes
/// (the key of the edge comes first); the texts are two and ten kilobytes long.
const HEAD_BYTES: usize = 192;

thread_local! {
    static HEAD_SCRATCH: RefCell<String> = const { RefCell::new(String::new()) };
}

struct Head<'a> {
    text: &'a str,
    /// `text` is the whole `repr`, not its beginning.
    complete: bool,
}

/// `repr(left).as_bytes().cmp(repr(right).as_bytes())` without making the whole of either text unless the first bytes of both are the same: the order of the planning layer
/// (`sorted(..., key=repr)`) decided by the heads of the texts, which are exact beginnings of them.
pub fn repr_cmp(left: &Val, right: &Val) -> Ordering {
    if left.same(right) {
        return Ordering::Equal;
    }
    let (first, second) = (left.head(), right.head());
    let common = first.text.len().min(second.text.len());
    match first.text.as_bytes()[..common].cmp(&second.text.as_bytes()[..common]) {
        Ordering::Equal => {}
        decided => return decided,
    }
    // a whole text that is shorter than the cut one is a beginning of it (the cut one is not over where the whole one is), so the smaller; anything else the heads do not tell
    match (first.complete, second.complete) {
        (true, true) => return first.text.len().cmp(&second.text.len()),
        (true, false) if first.text.len() < second.text.len() => return Ordering::Less,
        (false, true) if second.text.len() < first.text.len() => return Ordering::Greater,
        _ => {}
    }
    crate::profile::count(crate::profile::Stat::ReprHeadTies, 1);
    if strictly_same(left, right) {
        return Ordering::Equal;
    }
    left.repr().as_bytes().cmp(right.repr().as_bytes())
}

/// `repr(left) < repr(right)` as [`repr_cmp`] decides it.
pub fn repr_less(left: &Val, right: &Val) -> bool {
    repr_cmp(left, right) == Ordering::Less
}

/// Two values that print alike because they are made alike: the same kinds all the way down (`2` and `Fraction(2, 1)` are equal numbers and print differently, so they are
/// not this).
fn strictly_same(left: &Val, right: &Val) -> bool {
    if left.same(right) {
        return true;
    }
    if left.0.hash != right.0.hash {
        return false;
    }
    match (&left.0.kind, &right.0.kind) {
        (Kind::None, Kind::None) => true,
        (Kind::Bool(first), Kind::Bool(second)) => first == second,
        (Kind::Int(first), Kind::Int(second)) => first == second,
        (Kind::Str(first), Kind::Str(second)) => first == second,
        (Kind::Frac(first), Kind::Frac(second)) => first == second,
        (Kind::Tuple(Items::Terms(first, _)), Kind::Tuple(Items::Terms(second, _))) => {
            first.len() == second.len() && first.iter().zip(second.iter()).all(|(a, b)| a.radicand == b.radicand && a.coef.value() == b.coef.value() && a.coef.is_py_int() == b.coef.is_py_int())
        }
        (Kind::Tuple(first), Kind::Tuple(second)) | (Kind::List(first), Kind::List(second)) => first.len() == second.len() && first.iter().zip(second).all(|(a, b)| strictly_same(a, b)),
        (Kind::Data(first_name, first), Kind::Data(second_name, second)) => {
            first_name == second_name && first.len() == second.len() && first.iter().zip(second).all(|((name_a, a), (name_b, b))| name_a == name_b && strictly_same(a, b))
        }
        (Kind::Member(first_class, first_member, first), Kind::Member(second_class, second_member, second)) => first_class == second_class && first_member == second_member && strictly_same(first, second),
        (Kind::Set(first, first_frozen), Kind::Set(second, second_frozen)) => first_frozen == second_frozen && first.len() == second.len() && first.iter().zip(second).all(|(a, b)| strictly_same(a, b)),
        (Kind::Dict(first), Kind::Dict(second)) => first.len() == second.len() && first.iter().zip(second).all(|((key_a, a), (key_b, b))| strictly_same(key_a, key_b) && strictly_same(a, b)),
        _ => false,
    }
}

/// Where a text is written: a whole `String`, or a [`Bounded`] one that stops taking bytes at its limit.
trait Sink {
    fn text(&mut self, text: &str);
    fn ch(&mut self, character: char);
    /// A piece the writer of a number or a string makes into a `String`.
    fn piece(&mut self, write: impl FnOnce(&mut String));
    /// The text of a part.
    fn child(&mut self, item: &Val);
    /// No more bytes are taken.
    fn done(&self) -> bool;
}

impl Sink for String {
    fn text(&mut self, text: &str) {
        self.push_str(text);
    }

    fn ch(&mut self, character: char) {
        self.push(character);
    }

    fn piece(&mut self, write: impl FnOnce(&mut String)) {
        write(self);
    }

    /// The part's own text when it has one, else written through into `self` (no allocation, no copy of a copy). A part that is reached a second time without a text makes
    /// its own, so a value that many texts hold is formatted twice at most.
    fn child(&mut self, item: &Val) {
        if let Some(text) = item.0.text.get() {
            self.push_str(text);
        } else if item.0.visits.get() == 0 {
            item.0.visits.set(1);
            write_repr(&item.0.kind, self);
        } else {
            self.push_str(&item.repr());
        }
    }

    fn done(&self) -> bool {
        false
    }
}

/// The beginning of a text: bytes are taken up to `limit` (the piece that crosses it is taken whole, a cached text is cut), and then no more, so what was taken is exactly the
/// beginning of the whole text.
struct Bounded {
    out: String,
    limit: usize,
    stopped: bool,
}

impl Bounded {
    fn check(&mut self) {
        if self.out.len() >= self.limit {
            self.stopped = true;
        }
    }

    /// The first bytes of a text that is already made, as many as the room takes.
    fn take_beginning(&mut self, text: &str) {
        let room = self.limit - self.out.len();
        if text.len() >= room {
            self.out.push_str(&text[..room]);
            self.stopped = true;
        } else {
            self.out.push_str(text);
        }
    }
}

impl Sink for Bounded {
    fn text(&mut self, text: &str) {
        if !self.stopped {
            self.out.push_str(text);
            self.check();
        }
    }

    fn ch(&mut self, character: char) {
        if !self.stopped {
            self.out.push(character);
            self.check();
        }
    }

    fn piece(&mut self, write: impl FnOnce(&mut String)) {
        if !self.stopped {
            write(&mut self.out);
            self.check();
        }
    }

    fn child(&mut self, item: &Val) {
        if self.stopped {
            return;
        }
        let room = self.limit - self.out.len();
        if let Some(text) = item.0.text.get() {
            self.take_beginning(text);
        } else if let Some(head) = item.0.head.get().filter(|head| head.len() >= room) {
            self.take_beginning(head);
        } else {
            write_repr(&item.0.kind, self);
        }
    }

    fn done(&self) -> bool {
        self.stopped
    }
}

fn write_items<S: Sink>(out: &mut S, open: char, close: char, items: &[Val], one_tuple: bool) {
    out.ch(open);
    for (index, item) in items.iter().enumerate() {
        if out.done() {
            return;
        }
        if index > 0 {
            out.text(", ");
        }
        out.child(item);
    }
    if one_tuple && items.len() == 1 {
        out.ch(',');
    }
    out.ch(close);
}

/// The text of the tuple of pairs [`pair_of_term`] makes of the terms.
fn write_terms<S: Sink>(out: &mut S, terms: &[Term]) {
    out.ch('(');
    for (index, term) in terms.iter().enumerate() {
        if out.done() {
            return;
        }
        if index > 0 {
            out.text(", ");
        }
        out.piece(|buffer| {
            buffer.push('(');
            push_ubig(buffer, &term.radicand);
            buffer.push_str(", ");
            if term.coef.is_py_int() {
                push_ibig(buffer, term.coef.value().numerator());
            } else {
                repr_frac(term.coef.value(), buffer);
            }
            buffer.push(')');
        });
    }
    if terms.len() == 1 {
        out.ch(',');
    }
    out.ch(')');
}

fn write_repr<S: Sink>(kind: &Kind, out: &mut S) {
    match kind {
        Kind::None => out.text("None"),
        Kind::Bool(flag) => out.text(if *flag { "True" } else { "False" }),
        Kind::Int(number) => out.piece(|buffer| push_ibig(buffer, number)),
        Kind::Str(text) => {
            // ASCII by construction (`Val::str`); the call cannot refuse.
            out.piece(|buffer| {
                let _ = repr_str(text, buffer);
            });
        }
        Kind::Frac(number) => out.piece(|buffer| repr_frac(number, buffer)),
        Kind::Tuple(Items::Terms(terms, _)) => write_terms(out, terms),
        Kind::Tuple(items) => write_items(out, '(', ')', items, true),
        Kind::List(items) => write_items(out, '[', ']', items, false),
        Kind::Data(name, fields) => {
            out.text(name);
            out.ch('(');
            for (index, (field, item)) in fields.iter().enumerate() {
                if out.done() {
                    return;
                }
                if index > 0 {
                    out.text(", ");
                }
                out.text(field);
                out.ch('=');
                out.child(item);
            }
            out.ch(')');
        }
        Kind::Member(class, member, value) => {
            out.ch('<');
            out.text(class);
            out.ch('.');
            out.text(member);
            out.text(": ");
            out.child(value);
            out.ch('>');
        }
        Kind::Set(items, frozen) => match (items.is_empty(), frozen) {
            (true, false) => out.text("set()"),
            (true, true) => out.text("frozenset()"),
            (false, _) => {
                if *frozen {
                    out.text("frozenset(");
                }
                write_items(out, '{', '}', items, false);
                if *frozen {
                    out.ch(')');
                }
            }
        },
        Kind::Dict(pairs) => {
            out.ch('{');
            for (index, (key, item)) in pairs.iter().enumerate() {
                if out.done() {
                    return;
                }
                if index > 0 {
                    out.text(", ");
                }
                out.child(key);
                out.text(": ");
                out.child(item);
            }
            out.ch('}');
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
        (Kind::Tuple(Items::Terms(first, _)), Kind::Tuple(Items::Terms(second, _))) => first.len() == second.len() && first.iter().zip(second.iter()).all(|(a, b)| a.radicand == b.radicand && a.coef.value() == b.coef.value()),
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
        return Val::tuple_of([Val::int(1)]);
    }
    if value.is_tuple() {
        let items: Vec<Val> = value.items().unwrap_or(&[]).iter().map(identity_order_val).collect();
        return Val::tuple_of([Val::int(0), Val::tuple(items)]);
    }
    Val::tuple_of([Val::int(0), value.clone()])
}

/// `py_eq(identity_order_val(a), identity_order_val(b))` without making either key: `None` is equal to `None` alone, two tuples are equal by their items in this sense, and
/// anything else is equal by [`py_eq`].
fn identity_eq(a: &Val, b: &Val) -> bool {
    if a.same(b) {
        return true;
    }
    match (a.is_none(), b.is_none()) {
        (true, true) => return true,
        (true, false) | (false, true) => return false,
        (false, false) => {}
    }
    match (&a.0.kind, &b.0.kind) {
        (Kind::Tuple(first), Kind::Tuple(second)) => first.len() == second.len() && first.iter().zip(second).all(|(x, y)| identity_eq(x, y)),
        (Kind::Tuple(_), _) | (_, Kind::Tuple(_)) => false,
        _ => py_eq(a, b),
    }
}

/// `py_compare(identity_order_val(a), identity_order_val(b))` without making either key (the sort of births makes a tree of a thousand values for each key to compare a few of
/// them): a `None` is after any other value of its slot, two tuples go by the first item that is not equal in this sense, a tuple and anything else that is not `None` have no
/// order (the pair the oracle's `TypeError` names is the tuple of the keys of the items and the value), and two other values go by [`py_less`].
fn identity_compare(a: &Val, b: &Val) -> Result<Ordering, UnorderedPair> {
    match (a.is_none(), b.is_none()) {
        (true, true) => return Ok(Ordering::Equal),
        (true, false) => return Ok(Ordering::Greater),
        (false, true) => return Ok(Ordering::Less),
        (false, false) => {}
    }
    match (&a.0.kind, &b.0.kind) {
        (Kind::Tuple(first), Kind::Tuple(second)) => {
            for (x, y) in first.iter().zip(second) {
                if !identity_eq(x, y) {
                    return Ok(if identity_compare(x, y)? == Ordering::Less { Ordering::Less } else { Ordering::Greater });
                }
            }
            Ok(first.len().cmp(&second.len()))
        }
        (Kind::Tuple(_), other) => Err(UnorderedPair { left: "tuple", right: type_name(other) }),
        (other, Kind::Tuple(_)) => Err(UnorderedPair { left: type_name(other), right: "tuple" }),
        _ => {
            if py_eq(a, b) {
                Ok(Ordering::Equal)
            } else if py_less(a, b)? {
                Ok(Ordering::Less)
            } else {
                Ok(Ordering::Greater)
            }
        }
    }
}

/// `sorted(items, key=key)` with CPython 3.11's sequence of comparisons over the keys, `less` the `<` of two of them.
fn sorted_by_less<T: Clone>(items: &[T], keys: &[Val], less: impl Fn(&Val, &Val) -> Result<bool, UnorderedPair>) -> Result<Vec<T>, UnorderedPair> {
    let mut failure: Option<UnorderedPair> = None;
    let order = {
        let mut less = |left: &usize, right: &usize| -> cftuv_clip::error::ClipResult<bool> {
            match less(&keys[*left], &keys[*right]) {
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

/// `sorted(items, key=key)` for a key that CPython orders with `<` (a stable sort in CPython 3.11's comparison sequence). An unordered pair that the sort reaches is
/// the oracle's `TypeError`.
pub fn sorted_by_val<T: Clone>(items: &[T], key: impl Fn(&T) -> Val) -> Result<Vec<T>, UnorderedPair> {
    if items.len() < 2 {
        return Ok(items.to_vec());
    }
    let keys: Vec<Val> = items.iter().map(&key).collect();
    sorted_by_less(items, &keys, py_less)
}

/// [`sorted_by_val`] with the refusal of the port in place of the pair.
pub fn sorted_by_val_or_refuse<T: Clone>(items: &[T], key: impl Fn(&T) -> Val, site: &str) -> SkelResult<Vec<T>> {
    sorted_by_val(items, key).map_err(|pair| pair.refusal(site))
}

/// `sorted(items, key=exact_identity.identity_order_key)`: [`sorted_by_val`] over the keys that [`identity_order_val`] makes of the given ones, the same comparisons with the same
/// answers (and the same unordered pair where the oracle raises) and none of those keys built.
pub fn sorted_by_identity_order<T: Clone>(items: &[T], key: impl Fn(&T) -> Val) -> Result<Vec<T>, UnorderedPair> {
    if items.len() < 2 {
        return Ok(items.to_vec());
    }
    let keys: Vec<Val> = items.iter().map(&key).collect();
    sorted_by_less(items, &keys, |left, right| Ok(identity_compare(left, right)? == Ordering::Less))
}

/// [`sorted_by_identity_order`] with the refusal of the port in place of the pair.
pub fn sorted_by_identity_order_or_refuse<T: Clone>(items: &[T], key: impl Fn(&T) -> Val, site: &str) -> SkelResult<Vec<T>> {
    sorted_by_identity_order(items, key).map_err(|pair| pair.refusal(site))
}

/// `items.sort(key=repr)` in place, the key of each made once: [`sorted_by_repr`] on the vector itself.
pub fn sort_by_repr_key<T>(items: &mut Vec<T>, key: impl Fn(&T) -> Val) {
    if items.len() < 2 {
        return;
    }
    let mut keyed: Vec<(Val, T)> = items.drain(..).map(|item| (key(&item), item)).collect();
    keyed.sort_by(|left, right| repr_cmp(&left.0, &right.0));
    items.extend(keyed.into_iter().map(|(_, item)| item));
}

/// `sorted(items, key=repr)` where the key is the `repr` of a value: a total order, so the standard stable sort is the oracle's.
pub fn sorted_by_repr<T: Clone>(items: &[T], key: impl Fn(&T) -> Val) -> Vec<T> {
    let mut keyed: Vec<(Val, T)> = items.iter().map(|item| (key(item), item.clone())).collect();
    keyed.sort_by(|left, right| repr_cmp(&left.0, &right.0));
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

    /// A value of every shape the planning layer keys by, made from a seed: the long ones are what the head of a text cuts.
    fn next(seed: &mut u64, bound: u64) -> u64 {
        *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (*seed >> 33) % bound
    }

    fn sample(seed: &mut u64, depth: u32) -> Val {
        let shape = next(seed, if depth == 0 { 6 } else { 10 });
        match shape {
            0 => Val::none(),
            1 => Val::int(next(seed, 2000) as i64 - 1000),
            2 => Val::frac(Rat::new(IBig::from(next(seed, 100000) as i64 - 50000), IBig::from(next(seed, 900) as i64 + 1)).unwrap()),
            3 => Val::str(["EXISTING", "BIRTH", "SPLIT", "a'b"][next(seed, 4) as usize]),
            4 => Val::boolean(next(seed, 2) == 0),
            5 => Val::ints(&[next(seed, 500) as i64, -(next(seed, 500) as i64), 7, next(seed, 3) as i64]),
            6 | 7 => {
                let count = next(seed, 6);
                Val::tuple((0..count).map(|_| sample(seed, depth - 1)).collect())
            }
            8 => Val::data("Thing", vec![("a", sample(seed, depth - 1)), ("b", sample(seed, depth - 1)), ("c", sample(seed, depth - 1))]),
            _ => {
                let count = next(seed, 5);
                let frozen = next(seed, 2) == 0;
                Val::set((0..count).map(|_| sample(seed, depth - 1)).collect(), frozen)
            }
        }
    }

    /// The same list of values every call: random ones, and the ones that make a head tie or end early.
    fn head_cases() -> Vec<Val> {
        let mut seed = 0x5eed_u64;
        let mut values: Vec<Val> = (0..300).map(|_| sample(&mut seed, 5)).collect();
        // texts longer than the head, equal on the head and different after it, and equal values made twice
        let long = |tail: i64| Val::tuple(vec![Val::tuple((0..HEAD_BYTES as i64).map(Val::int).collect()), Val::int(tail)]);
        values.extend([long(1), long(2), long(1), long(10), long(9)]);
        values.push(Val::tuple((0..HEAD_BYTES as i64).map(Val::int).collect()));
        values.push(Val::int(2));
        values.push(Val::frac(Rat::from_int(IBig::from(2))));
        values.push(Val::tuple(vec![Val::tuple(vec![Val::int(2)])]));
        values.push(Val::tuple(vec![Val::tuple(vec![Val::frac(Rat::from_int(IBig::from(2)))])]));
        // a text of one length is a beginning of a longer one made of the same items
        let items: Vec<Val> = (0..60).map(Val::int).collect();
        for count in [40, 50, 60] {
            values.push(Val::list(items[..count].to_vec()));
        }
        values
    }

    #[test]
    fn the_order_by_the_head_of_the_text_is_the_order_of_the_texts() {
        // `values` is only ever compared with `repr_cmp` (a text is made only when the heads tie); `reference` is the same list whose whole texts are the expected order
        let (values, reference) = (head_cases(), head_cases());
        let mut pairs = 0;
        for (index, left) in values.iter().enumerate() {
            for (other, right) in values.iter().enumerate().skip(index.saturating_sub(40)) {
                let expected = reference[index].repr().as_bytes().cmp(reference[other].repr().as_bytes());
                assert_eq!(repr_cmp(left, right), expected, "{:?} against {:?}", left, right);
                assert_eq!(repr_cmp(right, left), expected.reverse());
                pairs += 1;
            }
        }
        assert!(pairs > 10_000);
        // sorted by the head, sorted by the text
        let mut order: Vec<usize> = (0..values.len()).collect();
        order.sort_by(|a, b| repr_cmp(&values[*a], &values[*b]));
        let mut by_text: Vec<usize> = (0..values.len()).collect();
        by_text.sort_by(|a, b| reference[*a].repr().as_bytes().cmp(reference[*b].repr().as_bytes()));
        assert_eq!(order, by_text);
    }

    /// A key of the shape of an identity key: nested tuples of ints, strings, fractions, `None`, lists and now and then a set.
    fn identity_sample(seed: &mut u64, depth: u32) -> Val {
        match next(seed, if depth == 0 { 5 } else { 8 }) {
            0 => Val::none(),
            1 => Val::int(next(seed, 4) as i64),
            2 => Val::frac(Rat::new(IBig::from(next(seed, 7) as i64), IBig::from(next(seed, 3) as i64 + 1)).unwrap()),
            3 => Val::str(["a", "b"][next(seed, 2) as usize]),
            4 => Val::boolean(next(seed, 2) == 0),
            5 | 6 => {
                let count = next(seed, 4);
                Val::tuple((0..count).map(|_| identity_sample(seed, depth - 1)).collect())
            }
            _ => {
                if next(seed, 3) == 0 {
                    Val::set(vec![Val::int(next(seed, 3) as i64)], true)
                } else {
                    let count = next(seed, 3);
                    Val::list((0..count).map(|_| identity_sample(seed, depth - 1)).collect())
                }
            }
        }
    }

    #[test]
    fn the_identity_order_compares_as_the_keys_it_would_make_do() {
        let mut seed = 0x1de7_u64;
        let (mut sorted_ok, mut refused) = (0, 0);
        for round in 0..400 {
            let count = 2 + (round % 7);
            let items: Vec<Val> = (0..count).map(|_| identity_sample(&mut seed, 3)).collect();
            let by_keys = sorted_by_val(&items, identity_order_val);
            let direct = sorted_by_identity_order(&items, Val::clone);
            match (&by_keys, &direct) {
                (Ok(first), Ok(second)) => {
                    sorted_ok += 1;
                    assert_eq!(first.len(), second.len());
                    assert!(first.iter().zip(second).all(|(a, b)| a.same(b)), "the same order for {items:?}");
                }
                (Err(first), Err(second)) => {
                    refused += 1;
                    assert_eq!(first, second, "the same pair for {items:?}");
                }
                _ => panic!("one refuses and the other does not: {items:?}"),
            }
            // and the answer of one comparison, both ways
            for pair in items.windows(2) {
                let (keys, direct) = (py_less(&identity_order_val(&pair[0]), &identity_order_val(&pair[1])), identity_compare(&pair[0], &pair[1]).map(|order| order == Ordering::Less));
                assert_eq!(keys, direct, "{:?} < {:?}", pair[0], pair[1]);
            }
        }
        assert!(sorted_ok > 20 && refused > 20, "{sorted_ok} sorted, {refused} refused");
    }

    /// The tuple of pairs the oracle's key holds for the terms, made value by value (what `terms_of` made before it kept the terms).
    fn terms_by_values(sum: &SqrtSum) -> Val {
        Val::tuple(sum.terms().iter().map(pair_of_term).collect())
    }

    #[test]
    fn the_terms_of_a_sum_are_the_tuple_of_pairs_they_stand_for() {
        use cftuv_core::rat::Coef;
        let fraction = |n: i64, d: i64| Coef::fraction(Rat::new(IBig::from(n), IBig::from(d)).unwrap());
        let term = |radicand: u64, coef: Coef| Term { radicand: UBig::from(radicand), coef };
        let sums = [
            SqrtSum::from_terms(vec![]).unwrap(),
            SqrtSum::from_terms(vec![term(1, fraction(3, 7))]).unwrap(),
            SqrtSum::from_terms(vec![term(1, Coef::int(IBig::from(4)))]).unwrap(),
            SqrtSum::from_terms(vec![term(1, fraction(-1, 2)), term(2, Coef::int(IBig::from(300))), term(54355477354110137, fraction(38365947, 2836155804872))]).unwrap(),
            SqrtSum::from_terms(vec![term(1, fraction(4, 1)), term(3, fraction(1, 3))]).unwrap(),
        ];
        for sum in &sums {
            let (kept, made) = (Val::terms_of(sum), terms_by_values(sum));
            assert_eq!(kept.hash_value(), made.hash_value(), "{:?}", made);
            assert_eq!(&*kept.repr(), &*made.repr());
            assert_eq!(kept, made);
            assert_eq!(made, kept);
            assert_eq!(kept.items().map(<[Val]>::len), made.items().map(<[Val]>::len));
            assert!(kept.is_tuple());
            // the head of the text is the beginning of it, the same for both
            let (fresh, other) = (Val::terms_of(sum), terms_by_values(sum));
            assert_eq!(repr_cmp(&fresh, &other), Ordering::Equal);
        }
        // equal numbers are equal pairs whether the coefficient is an int or a fraction, and the two print differently
        let (as_int, as_fraction) = (Val::terms_of(&sums[2]), Val::terms_of(&SqrtSum::from_terms(vec![term(1, fraction(4, 1))]).unwrap()));
        assert_eq!(as_int, as_fraction);
        assert_ne!(&*as_int.repr(), &*as_fraction.repr());
        assert!(!strictly_same(&as_int, &as_fraction));
        assert!(strictly_same(&as_int, &Val::terms_of(&sums[2])));
        // different sums are different keys, and they order as their pairs do
        assert_ne!(Val::terms_of(&sums[1]), Val::terms_of(&sums[3]));
        assert_eq!(py_less(&Val::terms_of(&sums[1]), &Val::terms_of(&sums[3])), py_less(&terms_by_values(&sums[1]), &terms_by_values(&sums[3])));
        // a key read as a sequence is the pairs
        let key = Val::tuple_of([Val::terms_of(&sums[3]), Val::terms_of(&sums[1])]);
        assert_eq!(key.get(0).and_then(|terms| terms.get(1)).and_then(|pair| pair.get(0)).and_then(Val::as_i64), Some(2));
    }

    #[test]
    fn a_text_cut_at_the_head_is_the_beginning_of_the_whole_text() {
        let long = Val::tuple(vec![Val::tuple((0..400).map(Val::int).collect()), Val::str("end")]);
        let head = long.head();
        assert!(!head.complete && head.text.len() >= HEAD_BYTES);
        assert!(long.repr().starts_with(head.text));
        // once the whole text is there it is the head
        assert!(long.head().complete);
        let short = Val::tuple(vec![Val::int(1)]);
        assert!(short.head().complete);
        assert_eq!(short.head().text, "(1,)");
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
