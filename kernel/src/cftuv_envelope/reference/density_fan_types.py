"""Именованные отказы рациональной власти Density-веера."""

from __future__ import annotations


class DensityRationalAuthorityExhausted(ValueError):
    """Именованный отказ только при превышении доказанного resource cap."""


class AdaptiveDensityFanInvalid(ValueError):
    """Запись V2 не следует из production metric/ideal facts."""


class DensityWindowChartUnrepresentable(AdaptiveDensityFanInvalid):
    """Допустимое projective-окно не представимо принятым chart-контрактом."""


class DensityTerminationBoxesExhausted(AdaptiveDensityFanInvalid):
    """Ящик вокруг ideal не нашёл осуществимого веера за `_BOX_REFINEMENT_CAP`.

    Это ИМЕННО нехватка уточнения, а не неразрешимость предиката: предел
    подшага с иррациональным лучом (допустимая область — точка) отсекается
    раньше, точным признаком `ideal_is_exact_limit_with_irrational_direction`,
    и сюда не доходит. Сюда доходит положительный запас, который не уместился
    в ящик шириной `2^-24 ... 2^-120` от окна, и, редкий случай, предел с
    рациональным лучом evaluation-геометрии, привязанный из-за иррационального
    исходного: сообщение называет, стоит ли веер ровно на пределе.
    """
