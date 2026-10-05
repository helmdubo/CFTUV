"""Python side of the boundary buffer format (`native/cftuv-core/src/codec.rs` is the other side).

Compact, little-endian, no decimal strings; any integer size. Layout (the Rust module documents the same):

    uint      LEB128
    int       uint header = (byte_count << 1) | negative, then the magnitude as little-endian bytes
    coef      flag u8: 0 Python int | 1 Fraction with denominator 1 | 2 Fraction (numerator, denominator)
    sum       uint term count, per term: int radicand, coef
    value     tag u8: 0 None | 1 False | 2 True | 3 int | 4 Fraction | 5 float | 6 sum | 7 list | 8 error

Decoding builds a `Fraction` WITHOUT re-normalising it: the numerator and denominator go straight into the
`Fraction` slots (the sender guarantees lowest terms, the Rust codec checks that in strict mode). This module
imports no extension: it only turns objects into bytes and bytes into objects. `SqrtSumV1` is looked up lazily
so that importing the package does not pull in the kernel.
"""

from __future__ import annotations

import struct
from fractions import Fraction

TAG_NONE = 0
TAG_FALSE = 1
TAG_TRUE = 2
TAG_INT = 3
TAG_FRAC = 4
TAG_FLOAT = 5
TAG_SUM = 6
TAG_LIST = 7
TAG_ERROR = 8

COEF_INT = 0
COEF_FRACTION_INTEGRAL = 1
COEF_FRACTION = 2

MAGIC = b"CFN1"
FLAG_STRICT = 1
FLAG_NO_MEMO = 2

ERROR_NAMES = {1: "OverflowError", 2: "ZeroDivisionError", 3: "ValueError"}

#: `(opcode, name)` of every number operation; a test compares this table with the one inside the extension.
OPS = (
    (1, "ISQRT"),
    (2, "GCD"),
    (3, "LCM"),
    (4, "BIT_LENGTH"),
    (5, "FLOAT_OF_INT"),
    (6, "FLOAT_OF_FRACTION"),
    (7, "MATH_SQRT_INT"),
    (8, "RAT_NEW"),
    (9, "RAT_ADD"),
    (10, "RAT_SUB"),
    (11, "RAT_MUL"),
    (12, "RAT_DIV"),
    (13, "RAT_NEG"),
    (14, "RAT_CMP"),
    (20, "SUM_RATIONAL"),
    (21, "SUM_ADD"),
    (22, "SUM_SUB"),
    (23, "SUM_NEG"),
    (24, "SUM_SCALED"),
    (25, "SUM_SCALED_DIFFERENCE"),
    (26, "SUM_DIFFERENCE_IS_ZERO"),
    (27, "SUM_MUL"),
    (28, "SUM_IS_ZERO"),
    (29, "SUM_IS_RATIONAL"),
    (30, "SUM_AS_RATIONAL"),
    (31, "SUM_ENCLOSURE"),
    (32, "SUM_CERTIFIED_SIGN"),
    (33, "SUM_SIGN_PREFILTER"),
    (34, "INTEGER_FORM"),
    (35, "INTEGER_ENCLOSURE"),
    (36, "INTEGER_CERTIFIED_SIGN"),
    (37, "SCALED_DIFFERENCE_PARTS"),
    (38, "MULTIPLY_INTEGER_ITEMS"),
    (39, "REDUCED_FORM"),
    (40, "SCALED_BY_RECIPROCAL"),
    (41, "FILTERED_SIGN"),
    (42, "DIFFERENCE_FILTERED_SIGN"),
    (50, "ORIENTED_SUM"),
    (51, "PRODUCT_ADDED"),
    (52, "SUM_OF_PRODUCTS"),
    (60, "FF_CENTRE_AND_BOUND"),
    (61, "FF_ORIENTATION_SIGN"),
    (62, "FF_LINE_ESTIMATE"),
    (63, "FF_POLYGON_SIGN"),
    (64, "FF_AFFINE_MAP_VIOLATED"),
)
OPCODES = {name: code for code, name in OPS}


class CodecError(ValueError):
    """A value the format cannot carry, or a buffer that is not in the format."""


class NativeError:
    """A named failure the native side returned for one operation (the oracle raised this exception class)."""

    __slots__ = ("code",)

    def __init__(self, code: int) -> None:
        self.code = code

    @property
    def name(self) -> str:
        return ERROR_NAMES.get(self.code, f"error {self.code}")

    def __eq__(self, other: object) -> bool:
        return type(other) is NativeError and other.code == self.code

    def __hash__(self) -> int:
        return hash(("NativeError", self.code))

    def __repr__(self) -> str:
        return f"NativeError({self.name})"


_SUM_TYPE: list = []


def sqrt_sum_type() -> type:
    """`SqrtSumV1` of the kernel on `sys.path`, resolved on first use."""

    if not _SUM_TYPE:
        from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

        _SUM_TYPE.append(SqrtSumV1)
    return _SUM_TYPE[0]


# --------------------------------------------------------------------------
# encoding
# --------------------------------------------------------------------------


def put_uint(out: bytearray, value: int) -> None:
    while value >= 0x80:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)


def put_int(out: bytearray, value: int) -> None:
    negative = value < 0
    magnitude = -value if negative else value
    count = (magnitude.bit_length() + 7) >> 3
    header = (count << 1) | negative
    if header < 0x80:
        out.append(header)
    else:
        put_uint(out, header)
    if count:
        out += magnitude.to_bytes(count, "little")


def put_unsigned(out: bytearray, value: int) -> None:
    if value < 0:
        raise CodecError(f"a negative number where a radicand or denominator is expected: {value}")
    put_int(out, value)


def put_coef(out: bytearray, coefficient) -> None:
    kind = type(coefficient)
    if kind is int:
        out.append(COEF_INT)
        put_int(out, coefficient)
    elif kind is Fraction:
        denominator = coefficient._denominator
        if denominator == 1:
            out.append(COEF_FRACTION_INTEGRAL)
            put_int(out, coefficient._numerator)
        else:
            out.append(COEF_FRACTION)
            put_int(out, coefficient._numerator)
            put_int(out, denominator)
    else:
        raise CodecError(f"a coefficient must be an int or a Fraction, not {kind.__name__}")


def put_sum(out: bytearray, value) -> None:
    terms = value.terms
    put_uint(out, len(terms))
    for radicand, coefficient in terms:
        put_unsigned(out, radicand)
        put_coef(out, coefficient)


def put_value(out: bytearray, value) -> None:
    kind = type(value)
    if kind is int:
        out.append(TAG_INT)
        put_int(out, value)
    elif kind is Fraction:
        out.append(TAG_FRAC)
        put_int(out, value._numerator)
        put_unsigned(out, value._denominator)
    elif kind is float:
        out.append(TAG_FLOAT)
        out += struct.pack("<d", value)
    elif kind is bool:
        out.append(TAG_TRUE if value else TAG_FALSE)
    elif value is None:
        out.append(TAG_NONE)
    elif kind is list or kind is tuple:
        out.append(TAG_LIST)
        put_uint(out, len(value))
        for item in value:
            put_value(out, item)
    elif isinstance(value, sqrt_sum_type()):
        out.append(TAG_SUM)
        put_sum(out, value)
    else:
        raise CodecError(f"cannot encode a {kind.__name__}")


def encode_value(value) -> bytes:
    out = bytearray()
    put_value(out, value)
    return bytes(out)


def encode_request(ops, *, strict: bool = True, memo: bool = True) -> bytes:
    """A number script: `ops` is a sequence of `(operation name or opcode, arguments)`."""

    out = bytearray(MAGIC)
    out.append((FLAG_STRICT if strict else 0) | (0 if memo else FLAG_NO_MEMO))
    put_uint(out, len(ops))
    for operation, arguments in ops:
        code = OPCODES[operation] if isinstance(operation, str) else operation
        out.append(code)
        put_uint(out, len(arguments))
        for argument in arguments:
            put_value(out, argument)
    return bytes(out)


# --------------------------------------------------------------------------
# decoding
# --------------------------------------------------------------------------

_new = object.__new__


def make_fraction(numerator: int, denominator: int) -> Fraction:
    """A `Fraction` from a pair already in lowest terms: no gcd, the slots are set directly."""

    value = _new(Fraction)
    value._numerator = numerator
    value._denominator = denominator
    return value


def get_uint(buffer: bytes, position: int) -> tuple[int, int]:
    byte = buffer[position]
    if byte < 0x80:
        return byte, position + 1
    value = byte & 0x7F
    shift = 7
    position += 1
    while True:
        byte = buffer[position]
        position += 1
        value |= (byte & 0x7F) << shift
        if byte < 0x80:
            return value, position
        shift += 7


def get_int(buffer: bytes, position: int) -> tuple[int, int]:
    header = buffer[position]
    if header < 0x80:
        position += 1
    else:
        header, position = get_uint(buffer, position)
    count = header >> 1
    if not count:
        return 0, position
    end = position + count
    value = int.from_bytes(buffer[position:end], "little")
    return (-value if header & 1 else value), end


def get_coef(buffer: bytes, position: int):
    flag = buffer[position]
    numerator, position = get_int(buffer, position + 1)
    if flag == COEF_INT:
        return numerator, position
    if flag == COEF_FRACTION_INTEGRAL:
        return make_fraction(numerator, 1), position
    if flag == COEF_FRACTION:
        denominator, position = get_int(buffer, position)
        return make_fraction(numerator, denominator), position
    raise CodecError(f"unknown coefficient flag {flag}")


def get_sum(buffer: bytes, position: int):
    count, position = get_uint(buffer, position)
    terms = []
    for _ in range(count):
        radicand, position = get_int(buffer, position)
        coefficient, position = get_coef(buffer, position)
        terms.append((radicand, coefficient))
    return sqrt_sum_type()(tuple(terms)), position


def get_value(buffer: bytes, position: int):
    tag = buffer[position]
    position += 1
    if tag == TAG_INT:
        return get_int(buffer, position)
    if tag == TAG_SUM:
        return get_sum(buffer, position)
    if tag == TAG_FRAC:
        numerator, position = get_int(buffer, position)
        denominator, position = get_int(buffer, position)
        return make_fraction(numerator, denominator), position
    if tag == TAG_LIST:
        count, position = get_uint(buffer, position)
        items = []
        for _ in range(count):
            item, position = get_value(buffer, position)
            items.append(item)
        return items, position
    if tag == TAG_NONE:
        return None, position
    if tag == TAG_TRUE:
        return True, position
    if tag == TAG_FALSE:
        return False, position
    if tag == TAG_FLOAT:
        return struct.unpack_from("<d", buffer, position)[0], position + 8
    if tag == TAG_ERROR:
        return NativeError(buffer[position]), position + 1
    raise CodecError(f"unknown value tag {tag}")


def decode_value(buffer: bytes):
    value, position = get_value(buffer, 0)
    if position != len(buffer):
        raise CodecError("trailing bytes after the value")
    return value


def decode_response(buffer: bytes) -> list:
    count, position = get_uint(buffer, 0)
    results = []
    for _ in range(count):
        value, position = get_value(buffer, position)
        results.append(value)
    if position != len(buffer):
        raise CodecError("trailing bytes after the last result")
    return results
