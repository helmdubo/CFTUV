"""Малый точный корпус весов и стен для MOTORCYCLE↔EXHAUSTIVE.

Здесь нет замороженных дайджестов результата: до слияния P0-2 абсолютная
запись узла намеренно не является властью.  Корпус задаёт только входы;
сверка двух режимов живёт в ``test_wavefront_weighted_wall_differential``.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from cftuv_envelope.wavefront.polygon import (
    PolygonV1,
    with_edge_speeds,
)

from chamfered_standard import with_right_angle_fans
from wavefront_cases import (
    axis_rectangle,
    cross,
    double_notch,
    ell,
    holes_grid,
    staircase,
)


Q_FACTORS = (
    Fraction(0),
    Fraction(1, 4),
    Fraction(1),
    Fraction(4),
    Fraction(65, 64),
)


@dataclass(frozen=True, slots=True)
class WeightedWallCase:
    name: str
    polygon: PolygonV1
    features: tuple[str, ...]


def _weighted(
    polygon: PolygonV1,
    factor: Fraction,
    *,
    source_and_walls: bool,
) -> PolygonV1:
    """Ставит абсолютное q: всем источникам либо двум источникам среди стен."""

    rows = []
    edges = tuple(polygon.edges())
    probe = min(2, len(edges) - 1)
    for index, (start, end, _) in enumerate(edges):
        if source_and_walls:
            value = factor if index in {0, probe} else Fraction(0)
        else:
            value = factor
        rows.append((start, end, value))
    return with_edge_speeds(polygon, tuple(rows))


def all_stationary(polygon: PolygonV1) -> PolygonV1:
    """Недостижимая ячейка q=0 × full-source для именованного отказа."""

    return with_edge_speeds(
        polygon,
        tuple((start, end, Fraction(0)) for start, end, _ in polygon.edges()),
    )


def _uniform_weight(polygon: PolygonV1, factor: Fraction) -> PolygonV1:
    return with_edge_speeds(
        polygon,
        tuple(
            (start, end, factor)
            for start, end, _ in polygon.edges()
        ),
    )


def _collinear_moving_source_and_wall() -> PolygonV1:
    """Одна прямая y=4 несёт moving source справа и wall слева."""

    polygon = cross(wide=6, tall=4)
    rows = []
    for start, end, _ in polygon.edges():
        is_moving = start[1] == end[1] == 4 and min(start[0], end[0]) >= 10
        rows.append((start, end, Fraction(65, 64) if is_moving else Fraction(0)))
    return with_edge_speeds(polygon, tuple(rows))


def _reflected(polygon: PolygonV1) -> PolygonV1:
    """Отражает петли и переносит q по неориентированному вхождению ребра."""

    def point(item: tuple[int, int]) -> tuple[int, int]:
        return (-item[0], item[1])

    loops = tuple(tuple(point(item) for item in loop.points) for loop in polygon.loops)
    reflected = PolygonV1.build(loops[0], loops[1:])
    speeds = {
        frozenset((point(start), point(end))): value
        for start, end, value in polygon.edges()
    }
    return with_edge_speeds(
        reflected,
        tuple(
            (start, end, speeds[frozenset((start, end))])
            for start, end, _ in reflected.edges()
        ),
    )


def weighted_wall_differential_corpus() -> tuple[WeightedWallCase, ...]:
    """23 входа: достижимая q-сетка и все особые семейства карточки."""

    cases: list[WeightedWallCase] = []
    grid = cross(wide=6, tall=4)
    for factor in Q_FACTORS[1:]:
        cases.append(
            WeightedWallCase(
                f"cross_full_q_{factor}",
                _weighted(grid, factor, source_and_walls=False),
                ("q_grid", "full_source", "collinear"),
            )
        )
    for factor in Q_FACTORS[1:]:
        cases.append(
            WeightedWallCase(
                f"cross_source_and_walls_q_{factor}",
                _weighted(grid, factor, source_and_walls=True),
                ("q_grid", "source_and_walls", "collinear"),
            )
        )

    for name, polygon in (
        ("rect", axis_rectangle(12, 8)),
        ("ell", ell(12)),
        ("staircase", staircase()),
    ):
        for partial in (False, True):
            cases.append(
                WeightedWallCase(
                    f"{name}_{'source_and_walls' if partial else 'full'}_q_65_64",
                    _weighted(
                        polygon,
                        Fraction(65, 64),
                        source_and_walls=partial,
                    ),
                    (name, "source_and_walls" if partial else "full_source"),
                )
            )

    fan = with_right_angle_fans(
        _weighted(ell(12), Fraction(65, 64), source_and_walls=False)
    )
    cases.append(WeightedWallCase("weighted_vertex_fan", fan, ("fan", "zero_edge")))
    for count, factor in ((1, Fraction(4)), (2, Fraction(65, 64))):
        cases.append(
            WeightedWallCase(
                f"weighted_holes_{count}",
                _weighted(
                    holes_grid(1, count),
                    factor,
                    source_and_walls=False,
                ),
                ("holes", "weighted"),
            )
        )

    direct = _weighted(
        double_notch(), Fraction(1, 4), source_and_walls=False
    )
    cases.extend(
        (
            WeightedWallCase("mirror_direct", direct, ("mirror", "weighted")),
            WeightedWallCase(
                "mirror_reflected", _reflected(direct), ("mirror", "weighted")
            ),
        )
    )
    cases.append(
        WeightedWallCase(
            "several_collinear_incidences",
            _weighted(
                cross(wide=5, tall=3, right=17, top=19),
                Fraction(65, 64),
                source_and_walls=False,
            ),
            ("collinear", "weighted"),
        )
    )
    cases.append(
        WeightedWallCase(
            "search_pruning_witness",
            _uniform_weight(cross(wide=4, tall=4), Fraction(4)),
            ("strict_pruning", "weighted"),
        )
    )
    cases.append(
        WeightedWallCase(
            "collinear_moving_source_and_wall",
            _collinear_moving_source_and_wall(),
            ("collinear", "moving_source_and_wall"),
        )
    )
    cases.append(
        WeightedWallCase(
            "same_time_weighted_collapse",
            _uniform_weight(
                cross(wide=4, tall=4, arm=4, right=12, top=12),
                Fraction(4),
            ),
            ("same_time", "weighted"),
        )
    )
    return tuple(cases)
