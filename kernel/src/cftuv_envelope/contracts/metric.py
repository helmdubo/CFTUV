"""Public planar metric contracts for exact reference and filtered runtime."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
from math import gcd, isfinite

from ..numeric import CertifiedDecimalIntervalV1
from ..ids import (
    ChainUseId,
    ReferenceMetricId,
    LineageId,
    PatchDomainId,
    PhysicalEdgeId,
    PlanarityCertificateId,
    RuntimeMetricId,
    SourceFaceId,
    SourceRevision,
    SourceVertexId,
    SurfaceTriangleId,
)


REFERENCE_PLANAR_METRIC_SCHEMA_V2 = (
    "cftuv.envelope.rational_affine_planar_metric.v2"
)
RUNTIME_PLANAR_METRIC_SCHEMA_V1 = "cftuv.envelope.runtime_planar_metric.v1"
EMBEDDING_CERTIFIED_RATIONAL_AFFINE_PLANAR_METRIC_SCHEMA_V1 = (
    "cftuv.envelope.embedding_certified_rational_affine_planar_metric.v1"
)


class PlanarityAdmissionLawV1(str, Enum):
    EXACT_SOURCE_PLANE_V1 = "EXACT_SOURCE_PLANE_V1"
    # Источник не компланарен побитово, но укладывается в объявленный бюджет
    # невязки. Вершины проецируются на плоскость ТОЧНО, в рациональных числах:
    # ниже по конвейеру арифметика остаётся точной, приблизительным является
    # только выбор входа, и он записан в сертификате.
    NEAR_PLANAR_PROJECTION_V1 = "NEAR_PLANAR_PROJECTION_V1"
    # Источник не плоский и не near-planar, но он РАЗВЁРТЫВАЕТСЯ: карта — не
    # проекция на плоскость, а привязанная к решётке шарнирная развёртка по
    # дереву смежности, и судит её растяжение (`DevelopableStretchCertificateV1`).
    DEVELOPABLE_UNFOLD_V1 = "DEVELOPABLE_UNFOLD_V1"
    # Источник не развёртывается ЦЕЛИКОМ (именованный отказ ступени выше), но развёртывается
    # его ПОЛОСА вокруг выбранных цепей: карта — развёртка носителя из треугольников в
    # пределах досягаемости запроса, а власть — запас до стены досягаемости в сертификате.
    DEVELOPABLE_BAND_CHART_V1 = "DEVELOPABLE_BAND_CHART_V1"


class GridSnappingLawV1(str, Enum):
    """Как координаты источника попали в эту метрику.

    Форма взята с `PlanarityAdmissionLawV1`: закон существует в контракте
    отдельно от того, какое значение объявляет хост, поэтому переключение
    политики видно в сертификате, а не только в поведении.

    Привязок в конвейере ДВЕ, и они независимы: вершины ИСТОЧНИКА (до базиса,
    механизм детерминированный) и точки КОНСТРУКЦИЙ (`offset_support_g`,
    `segment_intersections` — механизм вероятностный, карточка R1b сама числит
    его лотереей). Поэтому и законов три, а не два: разрез между ними
    измерен — на `building.002` привязка одного источника даёт `EXACT` и ту же
    топологию, а привязка конструкций поверх неё сливает три вычисленные точки
    и оставляет две висячие полурёбра.
    """

    # Нынешнее поведение: координаты binary64 читаются как точные дроби и
    # больше ничем не трогаются.
    UNSNAPPED_EXACT_V1 = "UNSNAPPED_EXACT_V1"
    # Вершины источника привязаны к целочисленной решётке ДО построения
    # базиса, конструкции — НЕ привязаны и остаются точными. Восстановленное
    # задуманное отношение живёт в метрике, а вырождение вычисленных точек
    # по-прежнему не сливается.
    SOURCE_ONLY_GRID_SNAP_V1 = "SOURCE_ONLY_GRID_SNAP_V1"
    # Вершины источника привязаны к целочисленной решётке ДО построения
    # базиса, поэтому базис, матрица Грама и все углы считаются уже от
    # привязанных координат; сверх того привязаны и конструкции.
    INTEGER_GRID_SNAP_V1 = "INTEGER_GRID_SNAP_V1"

    @property
    def snaps_source(self) -> bool:
        """Двигает ли закон вершины источника до построения базиса."""

        return self is not GridSnappingLawV1.UNSNAPPED_EXACT_V1

    @property
    def snaps_constructions(self) -> bool:
        """Двигает ли закон вычисленные точки (`offset_support_g` и пересечения).

        Отдельным свойством, а не сравнением с именем в четырёх местах: разрез
        между двумя привязками — это то, чем закон `SOURCE_ONLY_GRID_SNAP_V1`
        отличается от `INTEGER_GRID_SNAP_V1`, и он обязан быть назван один раз.
        """

        return self is GridSnappingLawV1.INTEGER_GRID_SNAP_V1


class GridWindowOutcomeV1(str, Enum):
    """Существует ли шаг решётки, который чинит вход и не съедает деталь.

    Границ у окна две, поэтому и исходов закрытия два: «границы разошлись» и
    «между границами нет степени двойки» лечатся разным, и сливать их в один
    отказ значило бы потерять причину.
    """

    WINDOW_AVAILABLE = "WINDOW_AVAILABLE"
    GRID_WINDOW_CLOSED = "GRID_WINDOW_CLOSED"
    NO_POWER_OF_TWO_STEP_IN_WINDOW = "NO_POWER_OF_TWO_STEP_IN_WINDOW"


class GridScaleSearchOrderV1(str, Enum):
    """С какого конца окна перебираются степени двойки.

    Порядок — часть закона, а не деталь реализации: он однозначно определяет,
    какой масштаб будет выбран, поэтому обязан быть записан рядом с выбором.
    Два конца окна названы оба, потому что оба осмысленны и замерены; какой из
    них объявляет ядро — сказано в `source_grid.GRID_SCALE_SEARCH_ORDER`.
    """

    # От самого МЕЛКОГО допустимого шага (наибольший масштаб) к крупному.
    FINEST_ADMISSIBLE_FIRST_V1 = "FINEST_ADMISSIBLE_FIRST_V1"
    # От самого КРУПНОГО допустимого шага (наименьший масштаб) к мелкому.
    COARSEST_ADMISSIBLE_FIRST_V1 = "COARSEST_ADMISSIBLE_FIRST_V1"


class GridScaleTrialOutcomeV1(str, Enum):
    """Чем кончилась проба одного масштаба. Третьего исхода нет."""

    RELATIONS_RESTORED = "RELATIONS_RESTORED"
    RELATIONS_NOT_RESTORED = "RELATIONS_NOT_RESTORED"


@dataclass(frozen=True, slots=True)
class GridScaleTrialV1:
    """Одна проба перебора: что пробовали и что из этого вышло.

    Записывается каждая проба, а не только победившая. Иначе выбор
    невоспроизводим: по картинке в Blender видно, какая геометрия получилась,
    но не видно, почему выбран именно этот шаг, — а он выбран потому, что
    предыдущие пробы отказали, и отказ каждой из них здесь назван числом.
    """

    scale: int
    step: ExactRationalV1
    restored_right_corners: int
    outcome: GridScaleTrialOutcomeV1

    def __post_init__(self) -> None:
        if self.scale <= 0 or self.scale & (self.scale - 1):
            raise ValueError("масштаб пробы — положительная степень двойки")
        if self.restored_right_corners < 0:
            raise ValueError("счётчик восстановленных углов неотрицателен")
        if self.step != ExactRationalV1(1, self.scale):
            raise ValueError("шаг пробы обязан быть обратным её масштабу")


@dataclass(frozen=True, slots=True)
class IntegerGridCertificateV1:
    """Решётка домена: обе границы окна, весь перебор и доказательство.

    Записывается ВСЕГДА, в том числе когда привязки не было: тот же вход с
    другим шагом даёт другой результат, и дайджест обязан их различать.

    `intended_right_corners` — сколько углов патча объявленная авторская
    ошибка числит задуманно прямыми, не будучи таковыми точно.
    `restored_right_corners` — сколько из них дают точно рациональную долю π
    в тех координатах, которые метрика реально несёт. Две названные величины —
    два поля: расхождение между числом названного и числом измеренного само по
    себе дефект.

    `window_step` и `source_scale` у всякого закона, который двигает источник
    (`SOURCE_ONLY_GRID_SNAP_V1`, `INTEGER_GRID_SNAP_V1`), — ВЫБРАННЫЙ
    перебором шаг, то есть последняя проба в `scale_trials`. При
    `UNSNAPPED_EXACT_V1` перебора не было, и это первый кандидат окна: шаг,
    который окно предлагает у своей нижней границы.

    Привязку конструкций сертификат не описывает и описывать не обязан: она
    происходит ниже, в карте, и её решётка выводится из `window_step`
    (`source_grid.chart_grid_for`). Два закона со снятой привязкой источника
    здесь неразличимы намеренно — различие между ними живёт там, где оно
    действует.
    """

    snapping_law: GridSnappingLawV1
    window_outcome: GridWindowOutcomeV1
    patch_extent: ExactRationalV1
    author_angular_error: ExactRationalV1
    decal_detail: ExactRationalV1
    window_lower_bound: ExactRationalV1
    window_upper_bound: ExactRationalV1
    window_step: ExactRationalV1 | None
    source_scale: int | None
    magnitude_bound: int | None
    intended_right_corners: int
    restored_right_corners: int
    search_order: GridScaleSearchOrderV1
    scale_trials: tuple[GridScaleTrialV1, ...]

    def __post_init__(self) -> None:
        if self.intended_right_corners < 0 or self.restored_right_corners < 0:
            raise ValueError("угловые счётчики решётки неотрицательны")
        if self.restored_right_corners > self.intended_right_corners:
            raise ValueError(
                "восстановлено не может быть больше, чем задумано прямыми"
            )
        for trial in self.scale_trials:
            if trial.restored_right_corners > self.intended_right_corners:
                raise ValueError(
                    "проба восстановила больше углов, чем задумано прямыми"
                )
            restored = (
                trial.restored_right_corners == self.intended_right_corners
            )
            if restored is not (
                trial.outcome is GridScaleTrialOutcomeV1.RELATIONS_RESTORED
            ):
                raise ValueError(
                    "исход пробы расходится с её же числом восстановленных"
                )
        # Проверки ниже принадлежат привязке ИСТОЧНИКА, а не одному закону:
        # окно, перебор и доказательство восстановления одинаковы у
        # `SOURCE_ONLY_GRID_SNAP_V1` и `INTEGER_GRID_SNAP_V1`, потому что
        # источник они двигают одинаково, а различаются ниже — привязкой
        # конструкций, которой в этом сертификате нет.
        if not self.snapping_law.snaps_source:
            if self.scale_trials:
                raise ValueError(
                    "UNSNAPPED_EXACT_V1 не перебирает масштабы: проб быть не может"
                )
            return
        if self.window_outcome is not GridWindowOutcomeV1.WINDOW_AVAILABLE:
            raise ValueError(
                f"{self.snapping_law.value} требует открытого окна шага"
            )
        if self.window_step is None or self.source_scale is None:
            raise ValueError("привязка без объявленного шага невозможна")
        # Сертификат, объявляющий восстановление, обязан ему удовлетворять.
        # Иначе «привязка сработала» существует как заявление, которого никто
        # не проверил, — а проверять его больше негде: дальше по конвейеру
        # исходные координаты уже недоступны.
        if self.restored_right_corners != self.intended_right_corners:
            raise ValueError(
                f"{self.snapping_law.value} не восстановил все задуманно "
                "прямые углы"
            )
        self._check_trials()

    def _check_trials(self) -> None:
        """Перебор обязан быть тем самым, который объявлен законом.

        Проверяется не «что-то записано», а четыре свойства объявленного
        закона: перебор непуст, победил ПЕРВЫЙ прошедший, победитель — это и
        есть выбранный шаг, и все пробы шли объявленным порядком внутри окна.
        Без этих проверок запись перебора была бы украшением, а не
        доказательством выбора.
        """

        if not self.scale_trials:
            raise ValueError("привязка без записанного перебора невозможна")
        winner = self.scale_trials[-1]
        if winner.outcome is not GridScaleTrialOutcomeV1.RELATIONS_RESTORED:
            raise ValueError("последняя проба перебора обязана быть прошедшей")
        if winner.scale != self.source_scale or winner.step != self.window_step:
            raise ValueError("выбранный шаг не совпадает с прошедшей пробой")
        if any(
            trial.outcome is GridScaleTrialOutcomeV1.RELATIONS_RESTORED
            for trial in self.scale_trials[:-1]
        ):
            raise ValueError(
                "закон берёт ПЕРВЫЙ прошедший масштаб: прошедшая проба до "
                "победителя означает, что выбран не он"
            )
        scales = [trial.scale for trial in self.scale_trials]
        descending = (
            self.search_order is GridScaleSearchOrderV1.FINEST_ADMISSIBLE_FIRST_V1
        )
        expected = sorted(scales, reverse=descending)
        if scales != expected or len(set(scales)) != len(scales):
            raise ValueError("перебор идёт не тем порядком, который объявлен")
        for trial in self.scale_trials:
            if not (
                self.window_lower_bound.numerator
                * trial.step.denominator
                <= trial.step.numerator * self.window_lower_bound.denominator
                and trial.step.numerator * self.window_upper_bound.denominator
                <= self.window_upper_bound.numerator * trial.step.denominator
            ):
                raise ValueError("проба перебора вышла за границы окна")


class NearPlanarResidualBudgetLawV1(str, Enum):
    """Чем меряется отклонение источника от собственной плоскости.

    Членов ДВА, потому что величин две, и они принадлежат разным законам.
    Записью от 2026-07-31 бюджет берётся у того закона, который источник
    ДВИГАЕТ: у привязывающего это ячейка выбранного шага, у
    `UNSNAPPED_EXACT_V1` — прежний шум представления. Пока член был один,
    контракт лгал о власти: `_admission_budget` уже брала ячейку у всякого
    снапнутого домена (полевой default), а сертификат безусловно писал
    `RELATIVE_EXTENT_OR_ULP_V1`. Число и закон расходились на два порядка —
    6.10e-05 против 3.31e-07 на полевом скате, — и прочитать по сертификату,
    чем судили домен, было нельзя. Это класс отказа «декоративная власть/DTO»:
    поле объявляет закон, которому не подчиняется.
    """

    # max(relative_extent_factor * max(planar_extent, minimum_extent),
    #     coordinate_ulp_multiplier * max_coordinate_ulp)
    # Исследовано в M-R0 как RUNTIME_PLANAR_RESIDUAL_BUDGET_CANDIDATE_V1.
    RELATIVE_EXTENT_OR_ULP_V1 = "RELATIVE_EXTENT_OR_ULP_V1"
    # Ячейка выбранного шага решётки: `IntegerGridCertificateV1.window_step`.
    # Осевая решётка сама вносит непланарность — наклонную плоскость она рвёт,
    # каждая координата садится в свой узел независимо, — и внесённое ею
    # отклонение ограничено ячейкой шага.
    GRID_STEP_CELL_V1 = "GRID_STEP_CELL_V1"
    # АБСОЛЮТНЫЙ ПРОДУКТОВЫЙ ДОПУСК ЮБКИ — решение владельца от 2026-08-01.
    # Величина принадлежит не решётке и не представлению, а ПРОДУКТУ: цель —
    # юбка декалей вдоль выбранных seam chains, и планарность домена сама по
    # себе не является целью. Число живёт в ядре
    # (`planar_metric.PRODUCT_SKIRT_ABSOLUTE_BUDGET`), сертификат его
    # повторяет, валидатор сверяет — закон владеет числом, запись его не
    # выбирает. Два закона выше остаются членами перечисления и остаются
    # пересчитываемыми: они история и красные контроли, а не действующий
    # допуск.
    PRODUCT_SKIRT_ABSOLUTE_V1 = "PRODUCT_SKIRT_ABSOLUTE_V1"


PRODUCT_SKIRT_ABSOLUTE_BUDGET = Fraction(1, 80)
"""Допуск кривизны near-planar: 1/80 единицы сцены = 1.25 см. Точная дробь.

Число живёт рядом со своим ЗАКОНОМ, а не у построителя: им пользуются оба
конца — построитель, чтобы судить источник, и валидатор, чтобы пересчитать
записанное. Положить его в `planar_metric` не выйдет и технически: валидатор
импортировал бы построитель через цикл
(`planar_metric` -> `source_grid` -> `reference` -> `validation`).

Решение ВЛАДЕЛЬЦА от 2026-08-01, три основания дословно по смыслу:

1. Цель продукта — ЮБКА ДЕКАЛЕЙ вдоль выбранных seam chains. Планарность
   домена сама по себе не цель и никогда ею не была: она была условием, при
   котором умеет считать нынешний алгоритм, и это условие приняли за цель.
2. Кривая или планарная геометрия домена НЕ ВАЖНА для этой цели. Кривые крыши
   `building.004` отклоняются на 0.15 мм – 1.2 см и обязаны СТРОИТЬСЯ; при
   прежних допусках (ячейка 6.10e-05, представление 3.31e-07) они отвергались
   честно и бесполезно — отказ был верен закону и бесполезен продукту.
3. Допуск поднят СПЕЦИАЛЬНО ВЫШЕ нынешней потребности (1.25 см против
   наблюдаемых 1.2 см), чтобы искать алгоритм, который ляжет на криволинейные
   поверхности. Это не послабление ради зелёного цвета, а расширение области,
   в которой решение ищется.

Число ДРОБЬЮ, а не float: бюджет сравнивается с точным квадратом невязки, и
двоичное приближение 0.0125 внесло бы в саму границу допуска шум, которого в
решении владельца нет. 1/80 представимо точно и двоично, и десятично.

Величина АБСОЛЮТНАЯ — в единицах сцены, не доля габарита. Доля означала бы,
что большой домен вправе быть кривее маленького, а юбке декали габарит патча
безразличен: допуск на отклонение поверхности от плоскости есть свойство
поверхности, а не её размера.

Контроль, что допуск не стал дырой: прогиб 0.1035 (10 см) на `building.002`
patch 10 остаётся отвергнутым — он в 8.3 раза больше этой границы.
"""


class AffineFrameSelectionLawV1(str, Enum):
    CANONICAL_SOURCE_VERTEX_BASIS_V1 = (
        "CANONICAL_SOURCE_VERTEX_BASIS_V1"
    )
    # Репер near-planar домена со ПРИВЕДЁННЫМ ЦЕЛОЧИСЛЕННЫМ базисом плоскости:
    # `A = w1 / S`, `B = w2 / S`, где `S` — масштаб решётки источника, а
    # `(w1, w2)` — приведённый (Лагранж—Гаусс) базис целочисленной решётки
    # `{w ∈ Z³ : w·n = 0}` плоскости с примитивной нормалью `n`. Репер от
    # разностей СПРОЕЦИРОВАННЫХ вершин (закон выше) наследует знаменатели
    # проекции (деление на `n·n`): матрица Грама получает огромные знаменатели, и
    # квадраты длин рёбер решётки — радиканды `SqrtSumV1` — вырастают до 227–264
    # бит (поле, `building`) с простыми делителями до 104 бит. Здесь Грам — целые
    # порядка `|n|`, делённые на `S²` (радиканды 138–145 бит). Начало и координаты
    # вершин те же: репер лежит в той же
    # плоскости, и `origin + u·A + v·B` по-прежнему ТОЧНО восстанавливает
    # спроецированную позицию — меняется только базис, вершины не двигаются.
    # Закон записывается ТОЛЬКО у near-planar домена со спроецированными
    # вершинами; у точной плоскости остаётся закон выше, и её байты не двигаются.
    REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1 = (
        "REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1"
    )
    # Репер КАРТЫ развёртки: начало нуль, `A = e_x / S'`, `B = e_y / S'`, Грам
    # `I / S'^2`, где `S'` — масштаб карты (`DevelopableUnfoldCertificateV1.
    # chart_scale`). Это обычный `RationalAffinePlanarMetricV2` над плоскостью КАРТЫ
    # (`z = 0`): `origin + u·A + v·B` восстанавливает точку карты, а не точку
    # источника — источник развёрнут, а не расположен в этой плоскости. Закон
    # записывается ТОЛЬКО у домена с сертификатом развёртки.
    UNFOLDED_DEVELOPMENT_FRAME_V1 = "UNFOLDED_DEVELOPMENT_FRAME_V1"


class NearPlanarFramePolicyV1(str, Enum):
    """Каким репером просят описывать near-planar карту (политика ВЫЗЫВАЮЩЕГО).

    Политика не записывается в метрику: записывается закон, который применён
    (`AffineFrameSelectionLawV1`). Приведённый базис применяется только к домену
    со спроецированными вершинами; точной плоскости политика ничего не меняет.

    ЧЕСТНО О ПРЕДЕЛЕ. Работа подготовки, исходный репер -> приведённый, на полевых
    near-planar доменах: `building` 106/109/120/121 на d1 — 0.89x, 1.96x, 61.5x,
    2.26x (на d2 у 106 и 109 — 39.6x и 4.2x), `building.004` 1/4/6/7 на d0 — 115x,
    «отказ по капу» -> EXACT, 22.6x, 5x. Лучше везде, кроме `building` 106 на d1
    (там хуже на 12 %), и не ВСЕГДА лучше вообще: на синтетическом
    домене, где только одна вершина выведена из плоскости, а остальной репер —
    малые целые, исходный репер дешевле (570 единиц работы против 3913). Выбор по
    размеру Грама это не ловит (замер: Грам приведённого базиса там даже проще, а
    радиканды длиннее — их держат знаменатели КООРДИНАТ карты, а не только Грам),
    поэтому политика явная, а не «умная».
    """

    CANONICAL_ONLY_V1 = "CANONICAL_ONLY_V1"
    REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1 = (
        "REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1"
    )


class ProjectionAnchorSelectionLawV1(str, Enum):
    """Basis identity for the additive projection-embedding certificate.

    `CANONICAL_SOURCE_VERTEX_BASIS_ON_RESOLVED_PLANE_V1` назвало ОДИН выбор
    базиса и записало его в оба поля сертификата. Сравнение двух копий одного
    числа не может упасть: закон был самодоказательным.

    `EXACT_SOURCE_3D_BASIS_AND_RESOLVED_PLANE_BASIS_V2` называет ДВА закона:
    сторона источника — канонический базис по точным трёхмерным координатам
    (до проекции), сторона проекции — канонический базис карты разрешённой
    плоскости. Входы разные, предикаты разные, расхождение достижимо.

    Старое значение оставлено читаемым: записи, выпущенные под ним, остаются
    разбираемыми, а какой закон применялся, видно по полю.
    """

    CANONICAL_SOURCE_VERTEX_BASIS_ON_RESOLVED_PLANE_V1 = (
        "CANONICAL_SOURCE_VERTEX_BASIS_ON_RESOLVED_PLANE_V1"
    )
    EXACT_SOURCE_3D_BASIS_AND_RESOLVED_PLANE_BASIS_V2 = (
        "EXACT_SOURCE_3D_BASIS_AND_RESOLVED_PLANE_BASIS_V2"
    )


class ProjectionInteriorInjectivityLawV1(str, Enum):
    """Как доказана глобальная инъективность интерьера карты проекции.

    Предпосылочный путь (простая граница, положительная ориентация, согласованные
    инцидентности, ожидаемая топология носителя) выводит инъективность из
    теоремы. Этот закон её не выводит, а ПРОВЕРЯЕТ: каждый полигон грани
    триангулируется отсечением ушей точными предикатами, и все пары
    треугольников проходят AABB-префильтр и точный тест разделяющей оси.
    Коробка домена 10-20 м и десятки-сотни треугольников делают O(n^2) дешёвым.
    """

    EAR_CLIPPED_TRIANGLE_PAIR_EXACT_OVERLAP_V1 = (
        "EAR_CLIPPED_TRIANGLE_PAIR_EXACT_OVERLAP_V1"
    )


class AffineReconstructionLawV1(str, Enum):
    O_PLUS_U_A_PLUS_V_B_V1 = "O_PLUS_U_A_PLUS_V_B_V1"


class AffineChartOrientationV1(str, Enum):
    COORDINATE_CCW_MATCHES_OWNER_PATCH = (
        "COORDINATE_CCW_MATCHES_OWNER_PATCH"
    )
    COORDINATE_CW_MATCHES_OWNER_PATCH = (
        "COORDINATE_CW_MATCHES_OWNER_PATCH"
    )


class RuntimePredicateFilterLawV1(str, Enum):
    BINARY64_OUTWARD_INTERVAL_V1 = "BINARY64_OUTWARD_INTERVAL_V1"


class RuntimePredicateResultV1(str, Enum):
    CERTIFIED_NEGATIVE = "CERTIFIED_NEGATIVE"
    CERTIFIED_ZERO = "CERTIFIED_ZERO"
    CERTIFIED_POSITIVE = "CERTIFIED_POSITIVE"
    EXACT_FALLBACK_REQUIRED = "EXACT_FALLBACK_REQUIRED"


class RuntimeMetricFallbackLawV1(str, Enum):
    AUTHORITATIVE_REFERENCE_METRIC_V2 = (
        "AUTHORITATIVE_REFERENCE_METRIC_V2"
    )


class MetricSemanticIdentityLawV1(str, Enum):
    EXACT_CONSTRUCTION_CERTIFICATES_ONLY = (
        "EXACT_CONSTRUCTION_CERTIFICATES_ONLY"
    )


@dataclass(frozen=True, slots=True)
class ExactRationalV1:
    numerator: int
    denominator: int

    def __post_init__(self) -> None:
        if type(self.numerator) is not int or type(self.denominator) is not int:
            raise TypeError("ExactRationalV1 requires integer components")
        if self.denominator <= 0:
            raise ValueError("ExactRationalV1 denominator must be positive")
        if gcd(abs(self.numerator), self.denominator) != 1:
            raise ValueError("ExactRationalV1 must be reduced")


@dataclass(frozen=True, slots=True)
class ExactPoint2V1:
    x: ExactRationalV1
    y: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactVector2V1:
    x: ExactRationalV1
    y: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactPoint3V1:
    x: ExactRationalV1
    y: ExactRationalV1
    z: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactVector3V1:
    x: ExactRationalV1
    y: ExactRationalV1
    z: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactMatrix2V1:
    m00: ExactRationalV1
    m01: ExactRationalV1
    m10: ExactRationalV1
    m11: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ExactSourceVertexCoordinateV2:
    source_vertex_id: SourceVertexId
    domain_coordinate: ExactPoint2V1


@dataclass(frozen=True, slots=True)
class CertifiedAffineSupportDirectionV2:
    direction_in_domain: ExactVector2V1
    reference_metric_id: ReferenceMetricId


@dataclass(frozen=True, slots=True)
class ExactSourcePlaneCertificateV1:
    certificate_id: PlanarityCertificateId
    patch_domain_id: PatchDomainId
    admission_law: PlanarityAdmissionLawV1
    exact: bool
    exact_plane_normal: ExactVector3V1
    source_vertex_ids: frozenset[SourceVertexId]
    reconstruction_law: AffineReconstructionLawV1

    def __post_init__(self) -> None:
        if not self.exact:
            raise ValueError(
                "ExactSourcePlaneCertificateV1 must be exact"
            )


@dataclass(frozen=True, slots=True)
class SourceSnapEmbeddingCertificateV1:
    """Exact evidence that source-grid snapping preserved the source embedding.

    Counts are deliberately recorded instead of one aggregate ``valid`` bit:
    the validator recomputes every count from the source positions, face
    cycles, and declared grid law.  A zero therefore names the exact property
    proved by the record and cannot hide a different failed predicate.
    """

    snapping_law: GridSnappingLawV1
    source_vertex_ids: tuple[SourceVertexId, ...]
    source_vertex_count: int
    source_edge_count: int
    intended_right_corner_count: int
    newly_coincident_vertex_pair_count: int
    collapsed_nonzero_source_edge_count: int
    new_nonadjacent_edge_intersection_count: int
    unclassifiable_source_corner_count: int
    unchanged_unclassifiable_source_corner_count: int
    degenerated_intended_right_corner_count: int
    exact_pair_test_count: int

    def __post_init__(self) -> None:
        counts = (
            self.source_vertex_count,
            self.source_edge_count,
            self.intended_right_corner_count,
            self.newly_coincident_vertex_pair_count,
            self.collapsed_nonzero_source_edge_count,
            self.new_nonadjacent_edge_intersection_count,
            self.unclassifiable_source_corner_count,
            self.unchanged_unclassifiable_source_corner_count,
            self.degenerated_intended_right_corner_count,
            self.exact_pair_test_count,
        )
        if any(item < 0 for item in counts):
            raise ValueError("source-snap embedding counts must be non-negative")
        if (
            self.unchanged_unclassifiable_source_corner_count
            > self.unclassifiable_source_corner_count
        ):
            raise ValueError(
                "unchanged unclassifiable corners are a subset of all "
                "unclassifiable source corners"
            )
        if self.source_vertex_count != len(self.source_vertex_ids):
            raise ValueError("source-snap vertex count disagrees with its IDs")


@dataclass(frozen=True, slots=True)
class NearPlanarProjectionEmbeddingCertificateV1:
    """Exact evidence that projection retained the patch embedding.

    `boundary_cyclic_order_sha256` — ОДНОСТОРОННИЙ отпечаток комбинаторики
    границы источника, а не двустороннее клеймо. Прежде здесь стояла пара
    `source_`/`projected_`, и обе половины считались из одной и той же
    последовательности `PhysicalEdgeId`: проекция в расчёт не входила, и
    сравнение не могло упасть ни на каком входе. Отпечаток оставлен, потому
    что валидатор его пересчитывает — подделанная запись отвергается, — но
    доказательством сохранения порядка он больше не притворяется.

    `source_anchor_vertex_ids` и `projected_anchor_vertex_ids` теперь
    считаются РАЗНЫМИ законами из РАЗНЫХ входов (см.
    `ProjectionAnchorSelectionLawV1`). Их РАВЕНСТВА сертификат не требует:
    измерено на 61 живом вызове проекции — 18 из них законно выбирают разную
    третью вершину базиса, потому что near-planar проекция имеет право
    сделать почти коллинеарную тройку точно коллинеарной. Отказом остаётся
    расхождение ПРЕФИКСА (origin, first): его может изменить только
    схлопывание двух различных вершин источника в одну точку карты.

    Поля `*_triangle_*` несут прямую власть по инъективности интерьера:
    предпосылки теоремы (простая граница, ориентация, вложенность, система
    вращения) записаны отдельными счётчиками выше, а перекрытие интерьеров
    доказывается перебором пар, а не выводится из них.
    """

    source_vertex_ids: tuple[SourceVertexId, ...]
    source_chart_dropped_axis: int
    source_chart_first_axis_negated: bool
    source_boundary_occurrence_count: int
    source_boundary_component_count: int
    projected_boundary_component_count: int
    coincident_boundary_occurrence_pair_count: int
    collapsed_boundary_edge_occurrence_count: int
    new_nonadjacent_edge_intersection_count: int
    new_nonadjacent_collinear_overlap_count: int
    source_face_orientation_signs: tuple[int, ...]
    projected_face_orientation_signs: tuple[int, ...]
    source_boundary_loop_orientation_signs: tuple[int, ...]
    projected_boundary_loop_orientation_signs: tuple[int, ...]
    source_boundary_loop_nesting_depths: tuple[int, ...]
    projected_boundary_loop_nesting_depths: tuple[int, ...]
    orientation_mismatch_count: int
    nesting_mismatch_count: int
    boundary_cyclic_order_sha256: str
    anchor_selection_law: ProjectionAnchorSelectionLawV1
    source_anchor_vertex_ids: tuple[SourceVertexId, ...]
    projected_anchor_vertex_ids: tuple[SourceVertexId, ...]
    resolved_plane_basis_unavailable_count: int
    source_fan_identity_sha256: str
    projected_fan_identity_sha256: str
    source_fan_ambiguity_count: int
    projected_fan_ambiguity_count: int
    interior_injectivity_law: ProjectionInteriorInjectivityLawV1
    coincident_projected_vertex_pair_count: int
    nonsimple_projected_face_count: int
    projected_face_triangle_count: int
    overlapping_projected_triangle_pair_count: int
    triangle_pair_broadphase_test_count: int
    triangle_pair_exact_test_count: int
    exact_pair_test_count: int

    def __post_init__(self) -> None:
        counts = (
            self.source_boundary_occurrence_count,
            self.source_boundary_component_count,
            self.projected_boundary_component_count,
            self.coincident_boundary_occurrence_pair_count,
            self.collapsed_boundary_edge_occurrence_count,
            self.new_nonadjacent_edge_intersection_count,
            self.new_nonadjacent_collinear_overlap_count,
            self.orientation_mismatch_count,
            self.nesting_mismatch_count,
            self.resolved_plane_basis_unavailable_count,
            self.source_fan_ambiguity_count,
            self.projected_fan_ambiguity_count,
            self.coincident_projected_vertex_pair_count,
            self.nonsimple_projected_face_count,
            self.projected_face_triangle_count,
            self.overlapping_projected_triangle_pair_count,
            self.triangle_pair_broadphase_test_count,
            self.triangle_pair_exact_test_count,
            self.exact_pair_test_count,
        )
        if any(item < 0 for item in counts):
            raise ValueError("projection embedding counts must be non-negative")
        if self.source_chart_dropped_axis not in (0, 1, 2):
            raise ValueError("source chart dropped axis must be 0, 1, or 2")
        source_loop_count = len(self.source_boundary_loop_orientation_signs)
        projected_loop_count = len(self.projected_boundary_loop_orientation_signs)
        if source_loop_count != len(self.source_boundary_loop_nesting_depths):
            raise ValueError("source loop signs and nesting depths disagree")
        if projected_loop_count != len(self.projected_boundary_loop_nesting_depths):
            raise ValueError("projected loop signs and nesting depths disagree")
        signs = (
            *self.source_face_orientation_signs,
            *self.projected_face_orientation_signs,
            *self.source_boundary_loop_orientation_signs,
            *self.projected_boundary_loop_orientation_signs,
        )
        if any(item not in (-1, 0, 1) for item in signs):
            raise ValueError("embedding orientation signs must be -1, 0, or 1")


class NearPlanarWidthDistortionLawV1(str, Enum):
    """Чем меряется искажение ширины при проекции на плоскость карты.

    Проекция треугольника T на плоскость с нормалью `n` — линейное отображение
    с сингулярными числами `1` и `cos θ_T`, где `θ_T` — угол между нормалью
    треугольника и `n`. Длина вдоль поверхности относится к длине на карте как
    число из `[1, 1/cos θ_T]`: декаль, заданная шириной на карте, на поверхности
    не шире, чем в `1/cos θ_T` раз. `cos² θ_T` — РАЦИОНАЛЬНОЕ число
    (`(n_T·n)² / ((n_T·n_T)(n·n))`), поэтому закон считается точно, без корня
    и без допуска вычисления; допуск один, и он назван — относительная ширина.
    """

    INTRINSIC_WIDTH_RELATIVE_V1 = "INTRINSIC_WIDTH_RELATIVE_V1"


class NearPlanarLiftLawV1(str, Enum):
    """На какую поверхность ложится меш near-planar домена.

    Ось закона: одна и та же метрика (карта домена на сертифицированной
    плоскости) допускает два разных ответа на вопрос «где в 3D лежит точка
    карты».

    * `CERTIFIED_PLANE_V1` — на сертифицированную плоскость: `origin + a·x +
      b·y`. Поверхность источника при этом не трогается, расстояние до неё —
      невязка сертификата, и судит её абсолютный бюджет юбки.
    * `SOURCE_TRIANGLES_V1` — на треугольники источника: точка карты находится
      в проекции треугольника точно, подъём — барицентрический по его ПРИВЯЗАННЫМ
      3D-вершинам. Расстояние до поверхности по построению нуль, абсолютная
      невязка плоскости перестаёт судить и становится записанной диагностикой;
      судят искажение ширины (`NearPlanarWidthDistortionCertificateV1`) и
      сертификат вложения проекции.

    * `SOURCE_TRIANGLES_CLIPPED_V1` — то же, что `SOURCE_TRIANGLES_V1`, плюс
      РЕЗКА граней меша по рёбрам треугольников источника: на каждом внутреннем
      ребре источника, которое пересекает грань, встаёт вершина, и каждый кусок
      грани лежит в ОДНОМ замкнутом треугольнике источника (доказано точно). Это
      закон подъёма, а не закон топологии: он добавляет вершины `clip:<k>`, а
      суд над доменом (σ, вложение, сертификат) остаётся прежним, поэтому
      сертификат метрики пишет `SOURCE_TRIANGLES_V1` (`judged_as`).
    * `SOURCE_FACES_CLIPPED_V1` — то же, но режет ТОЛЬКО по настоящим рёбрам меша: ребро источника, общее
      у треугольников РАЗНЫХ граней. Диагональ четырёхгранья — ребро триангуляции хоста, а не меша, и
      по ней грань не режется: кусок лежит в одной ЗАМКНУТОЙ грани источника (объединение её
      треугольников, выпуклое на карте). Грань, непланарная больше допуска хорды
      (`materialize.clip_cells.CLIP_DIAGONAL_CHORD_BUDGET`), режется по своим треугольникам, как под
      `SOURCE_TRIANGLES_CLIPPED_V1`, под названным счётчиком. Суд тот же (`judged_as`).

    Точно планарный домен закон не затрагивает: его сертифицированная
    плоскость и есть его поверхность.
    """

    CERTIFIED_PLANE_V1 = "CERTIFIED_PLANE_V1"
    SOURCE_TRIANGLES_V1 = "SOURCE_TRIANGLES_V1"
    SOURCE_TRIANGLES_CLIPPED_V1 = "SOURCE_TRIANGLES_CLIPPED_V1"
    SOURCE_FACES_CLIPPED_V1 = "SOURCE_FACES_CLIPPED_V1"

    @property
    def onto_surface(self) -> bool:
        """Меш кладётся на треугольники источника (резка либо нет)."""

        return self is not NearPlanarLiftLawV1.CERTIFIED_PLANE_V1

    @property
    def clips(self) -> bool:
        """Грани меша режутся рёбрами источника (по треугольникам либо по граням)."""

        return self in (
            NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1,
            NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        )

    @property
    def clips_by_faces(self) -> bool:
        """Резка идёт по рёбрам МЕШ-граней источника, а не по рёбрам его триангуляции."""

        return self is NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1

    @property
    def judged_as(self) -> "NearPlanarLiftLawV1":
        """Закон, под которым ДОМЕН ПРИНЯТ: резка суда не меняет, сертификат пишет его."""

        if self.clips:
            return NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1
        return self


NEAR_PLANAR_WIDTH_BUDGET = Fraction(1, 50)
"""Допуск искажения ширины near-planar: 2 % относительно. Точная дробь.

Решение ВЛАДЕЛЬЦА (`DECISIONS.md`, 2026-10-02, «КРИВИЗНА, ПЕРВАЯ СТУПЕНЬ»):
порог относительный, не в сантиметрах — допуск на искажение свойства
поверхности («насколько декаль шире на поверхности, чем на карте») не должен
зависеть от размера патча. Условие приёма: `min cos² θ_T ≥ 1/(1+b)²`, то есть
`b = 1/50` даёт `cos² ≥ 2500/2601`, наклон не круче ~11.5° у худшего
треугольника.

Допуск владеет ЯДРО: его читают и построитель (судить), и валидатор
(пересчитать), и записывает сертификат — как `PRODUCT_SKIRT_ABSOLUTE_BUDGET`.
Абсолютная невязка плоскости (1.25 см) при укладке на треугольники источника
перестаёт судить и становится записанной диагностикой: поверхность, на которую
ложится декаль, — не плоскость, и расстояние до плоскости ей безразлично.
"""


@dataclass(frozen=True, slots=True)
class SnappedSourcePositionV1:
    """Позиция вершины источника после привязки к решётке, ДО проекции."""

    source_vertex_id: SourceVertexId
    position: ExactPoint3V1


@dataclass(frozen=True, slots=True)
class NearPlanarWidthDistortionCertificateV1:
    """Искажение ширины по треугольникам источника владельца: точное, записанное.

    `min_cos_squared` — наименьший `cos² θ_T` по измеренным (невырожденным)
    треугольникам; `worst_triangle_id` — тот, кто его даёт (первый по имени при
    равенстве). Вырожденный после привязки треугольник (нулевая нормаль) не
    измерим и не молчит: он считается в `degenerate_triangle_count`, называется
    `first_degenerate_triangle_id`, и судья обязан отказать
    `NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE`.

    `folded_triangle_count` — сколько треугольников источника ПРОЕКЦИЯ перевернула
    (обход проекции треугольника против обхода проекции его грани: знак `n_T·n`
    против `n_F·n`, точно); `first_folded_triangle_id` и `first_folded_face_id`
    называют первый по имени. Перевёрнутый треугольник накрывает соседа, и точка
    карты принадлежала бы двум плоскостям, поэтому судья обязан отказать
    `NEAR_PLANAR_SOURCE_TRIANGLE_FOLDED`. Квадрат `cos²` знак прячет, а вложение
    P0-4 проверяет полигоны граней, а не треугольники укладки.

    `snapped_source_positions` — позиции, от которых считан сертификат, то есть
    привязанные, но не спроецированные. Это то же 3D, на которое ложится
    декаль при укладке на треугольники источника, поэтому запись
    самодостаточна для укладки и пересчитываема валидатором из снапшота.

    Сертификат НЕ судит сам: это запись. Судит `width_distortion_violations`
    (`_width_distortion`), и когда и кого он судит, решает закон укладки.
    """

    certificate_id: PlanarityCertificateId
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    law: NearPlanarWidthDistortionLawV1
    width_budget: ExactRationalV1
    min_cos_squared: ExactRationalV1
    worst_triangle_id: SurfaceTriangleId | None
    worst_face_id: SourceFaceId | None
    triangles_measured: int
    degenerate_triangle_count: int
    first_degenerate_triangle_id: SurfaceTriangleId | None
    folded_triangle_count: int
    first_folded_triangle_id: SurfaceTriangleId | None
    first_folded_face_id: SourceFaceId | None
    snapped_source_positions: frozenset[SnappedSourcePositionV1]

    def __post_init__(self) -> None:
        if (
            self.triangles_measured < 0
            or self.degenerate_triangle_count < 0
            or self.folded_triangle_count < 0
        ):
            raise ValueError("width-distortion counts must be non-negative")
        if (self.first_folded_triangle_id is None) != (
            self.folded_triangle_count == 0
        ) or (self.first_folded_triangle_id is None) != (
            self.first_folded_face_id is None
        ):
            raise ValueError(
                "a folded triangle and its face are named exactly when one was counted"
            )
        if self.folded_triangle_count > self.triangles_measured:
            raise ValueError("a folded triangle is a measured triangle")
        if (self.worst_triangle_id is None) != (self.triangles_measured == 0):
            raise ValueError(
                "the worst triangle is named exactly when a triangle was measured"
            )
        if (self.worst_triangle_id is None) != (self.worst_face_id is None):
            raise ValueError("the worst triangle and its face are named together")
        if (self.first_degenerate_triangle_id is None) != (
            self.degenerate_triangle_count == 0
        ):
            raise ValueError(
                "a degenerate triangle is named exactly when one was counted"
            )
        low = Fraction(self.min_cos_squared.numerator, self.min_cos_squared.denominator)
        if not 0 <= low <= 1:
            raise ValueError("cos-squared lies in [0, 1]")
        if self.width_budget.numerator <= 0:
            raise ValueError("the width budget is positive")
        if self.triangles_measured + self.degenerate_triangle_count == 0:
            raise ValueError("a width-distortion certificate saw no triangle")


@dataclass(frozen=True, slots=True)
class NearPlanarProjectionCertificateV1:
    """Запись о том, что вход был спроецирован, и на сколько он отклонялся.

    Карта N0 требует: ни один солвер не сглаживает и не проецирует молча, и
    каждая неточная политика записывает масштаб, метод, бюджет невязки и
    ревизию источника. Проекция здесь точная (рациональная); приблизителен
    выбор плоскости, и именно он зафиксирован этими полями.

    `max_residual_squared` назван КВАДРАТОМ, потому что квадратом и является:
    невязка сравнивается с бюджетом в квадрате, чтобы не вводить корень, а
    корень из рационального в общем случае не представим точно и хранить его
    было бы нечем. Прежнее имя `max_residual` лгало о величине — читающий
    сравнивал его с `residual_budget` и получал разницу в квадрат.

    `max_coordinate_ulp` записан затем, чтобы проверяющий мог ПЕРЕСЧИТАТЬ
    `residual_budget` по объявленному закону, ничего не принимая на веру. У
    закона ячейки для этого хватает `IntegerGridCertificateV1.window_step`
    метрики, а закон представления берёт максимум из двух членов, и второй из
    них — ULP координат ИСТОЧНИКА (до проекции). По самой метрике он
    невосстановим: карта несёт координаты уже спроецированных позиций.
    Без этого поля записанное число не проверялось бы вовсе, и «бюджет»
    остался бы заявлением.
    """

    certificate_id: PlanarityCertificateId
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    admission_law: PlanarityAdmissionLawV1
    exact: bool
    exact_plane_normal: ExactVector3V1
    source_vertex_ids: frozenset[SourceVertexId]
    reconstruction_law: AffineReconstructionLawV1
    residual_budget_law: NearPlanarResidualBudgetLawV1
    relative_extent_factor: ExactRationalV1
    minimum_extent: ExactRationalV1
    coordinate_ulp_multiplier: int
    max_coordinate_ulp: ExactRationalV1
    planar_extent: ExactRationalV1
    residual_budget: ExactRationalV1
    max_residual_squared: ExactRationalV1
    projected_source_vertex_ids: frozenset[SourceVertexId]
    # Искажение ширины по треугольникам источника (ступень NEAR_PLANAR V2).
    # `None` — «не измерялось»: вызвавший построитель не дал треугольников.
    width_distortion: NearPlanarWidthDistortionCertificateV1 | None = None
    # Закон укладки, под которым домен ПРИНЯТ. При `SOURCE_TRIANGLES_V1` судят
    # искажение ширины и вложение проекции, а абсолютная невязка плоскости
    # (`max_residual_squared` против `residual_budget`) — записанная диагностика,
    # она может быть больше бюджета; при `CERTIFIED_PLANE_V1` судит невязка.
    lift_law: NearPlanarLiftLawV1 = NearPlanarLiftLawV1.CERTIFIED_PLANE_V1

    def __post_init__(self) -> None:
        if self.exact:
            raise ValueError(
                "NearPlanarProjectionCertificateV1 describes a non-exact plane"
            )
        if self.admission_law is not PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1:
            raise ValueError(
                "NearPlanarProjectionCertificateV1 requires "
                "NEAR_PLANAR_PROJECTION_V1"
            )
        if not self.projected_source_vertex_ids:
            raise ValueError(
                "near-planar certificate must name the projected vertices"
            )


class CurvatureLadderPolicyV1(str, Enum):
    """Лестница метрики по кривизне: что пробуют ПОСЛЕ именованного отказа near-planar.

    Политика ВЫЗЫВАЮЩЕГО, как политика укладки: в метрику не пишется, пишется
    закон, который применён (`planarity_certificate`). Лестница одна и
    однонаправленная: EXACT -> NEAR_PLANAR -> DEVELOPABLE. Развёртка пробуется
    ТОЛЬКО после отказа near-planar по ширине, перевороту треугольника или
    вложению проекции, поэтому домен, принятый сегодня, не перемаршрутизируется
    и его байты прежние.
    """

    NEAR_PLANAR_ONLY_V1 = "NEAR_PLANAR_ONLY_V1"
    NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1 = "NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1"


class DevelopableUnfoldTreeLawV1(str, Enum):
    """Какое дерево смежности разворачивается: корень и обход названы законом.

    Корень — треугольник владельца с наименьшим значением `SurfaceTriangleId`;
    обход — в ширину, соседи берутся по порядку номеров сторон `0, 1, 2`
    (нумерация ядра: сторона `i` — пара `(v[i], v[i+1])`). Шарнир по стороне
    родителя кладёт третью вершину ребёнка по другую сторону стороны.
    """

    CANONICAL_BFS_SMALLEST_TRIANGLE_ID_V1 = "CANONICAL_BFS_SMALLEST_TRIANGLE_ID_V1"


class DevelopableProposalLawV1(str, Enum):
    """Чем считается ПРЕДЛОЖЕНИЕ карты. Предложение — не власть: судит растяжение.

    Шарнирная развёртка в binary64 с фиксированным порядком операций над
    рациональными квадратами длин: вершина получает ОДНУ позицию при первом
    достижении (сварка), затем все позиции привязываются к решётке карты.
    Невязка веера недевелопабельной вершины и шум привязки попадают в растяжение
    треугольников, а не молча исчезают.

    `ARAP_LOCAL_GLOBAL_80_BINARY64_V1` — ВТОРОЕ предложение, и пробуется оно ТОЛЬКО
    после именованного отказа шарнира (растяжение за бюджетом, переворот, самонакрытие
    границы): шарнир кладёт треугольник за треугольником и сваливает весь угловой дефект
    вершин в последние, ARAP (local/global, котангенсовы веса, 80 итераций, одна
    пришпиленная вершина, Холецкий по огибающей, binary64 с фиксированным порядком
    операций) размазывает его по всей карте. Число итераций — часть закона и его имени.
    Старт ARAP — положения шарнира, а имя его отказа записано в `previous_refusals`
    сертификата. Власть та же: точный сертификат растяжения и простота границы.

    `ARAP_CONE_RELIEF_80_BINARY64_V1` — тот же ARAP с теми же 80 итерациями, но с ЦЕЛЬЮ,
    смещённой по углу у граничных вершин, чей разомкнутый веер не помещается в оборот
    (`_cone_relief`, закон `CONE_RELIEF_NLERP_V1`). Пробуется ТОЛЬКО после отказа шарнира
    `DEVELOPABLE_CHART_SELF_OVERLAP` при развёртке в бюджете (изометрия накрывает себя сама, ARAP
    её не лечит), и последняя запись `previous_refusals` у него — именно этот отказ.
    """

    BINARY64_HINGE_V1 = "BINARY64_HINGE_V1"
    ARAP_LOCAL_GLOBAL_80_BINARY64_V1 = "ARAP_LOCAL_GLOBAL_80_BINARY64_V1"
    ARAP_CONE_RELIEF_80_BINARY64_V1 = "ARAP_CONE_RELIEF_80_BINARY64_V1"


class DevelopableProposalSelectionLawV1(str, Enum):
    """КАК выбрано предложение, давшее карту: шарнир или ARAP, и чем это записано.

    Карта шарнира, принятая в бюджете запроса, раньше уходила в сертификат, даже если ARAP
    растянул бы её меньше (при бюджете 20 % шарнир в 15 % побеждал ARAP в 2 %). Закон
    «лучшее предложение»: если сертифицированное растяжение карты шарнира выше
    `DEVELOPABLE_ISOMETRIC_ENOUGH`, ARAP тоже строит карту (тот же суд, тот же бюджет, та же
    решётка), и остаётся карта с МЕНЬШИМ сертифицированным растяжением; равенство решает
    шарнир. Имя записывает победителя и его причину, а числа обоих лежат рядом:

    * `HINGE_ISOMETRIC_ENOUGH_V1` — шарнир не растянут выше порога, ARAP не пробовался;
    * `ARAP_AFTER_HINGE_REFUSED_V1` — шарнир отказан именем ДО привязки, карту дал ARAP либо
      ARAP с запасом угла у конуса (какой из двух — называет `proposal_law`; последняя запись
      `previous_refusals` — отказ шарнира);
    * `BEST_HINGE_WON_V1` / `BEST_ARAP_WON_V1` — обе карты приняты, выбрана меньшая;
    * `HINGE_KEPT_ARAP_UNAVAILABLE_V1` — ARAP не получил положений (потолок работы, матрица
      не положительна), остаётся карта шарнира;
    * `HINGE_KEPT_ARAP_REFUSED_V1` — карта ARAP названно отказана (`arap_refusal`), остаётся
      карта шарнира.
    """

    HINGE_ISOMETRIC_ENOUGH_V1 = "HINGE_ISOMETRIC_ENOUGH_V1"
    ARAP_AFTER_HINGE_REFUSED_V1 = "ARAP_AFTER_HINGE_REFUSED_V1"
    BEST_HINGE_WON_V1 = "BEST_HINGE_WON_V1"
    BEST_ARAP_WON_V1 = "BEST_ARAP_WON_V1"
    HINGE_KEPT_ARAP_UNAVAILABLE_V1 = "HINGE_KEPT_ARAP_UNAVAILABLE_V1"
    HINGE_KEPT_ARAP_REFUSED_V1 = "HINGE_KEPT_ARAP_REFUSED_V1"


class DevelopableStretchLawV1(str, Enum):
    """Чем судится растяжение: сингулярные числа `G_s^-1 G_c` без корней.

    Для треугольника с рациональным Грамом источника `G_s` и целочисленным
    Грамом карты `G_c` линейное отображение источник -> карта имеет квадраты
    сингулярных чисел, равные корням `q(λ) = det(G_c - λ G_s)`. Корни лежат в
    `[1/(1+b)^2, (1+b)^2]` тогда и только тогда, когда `q(l) >= 0`, `q(u) >= 0`
    и `l <= tr(G_s^-1 G_c)/2 <= u`: три знака рациональных чисел, ни корня, ни
    допуска вычисления.
    """

    EXACT_GRAM_SINGULAR_VALUE_BAND_V1 = "EXACT_GRAM_SINGULAR_VALUE_BAND_V1"


class DevelopableLiftLawV1(str, Enum):
    """Куда ложится меш развёрнутого домена: только на треугольники источника."""

    UNFOLDED_SOURCE_TRIANGLES_V1 = "UNFOLDED_SOURCE_TRIANGLES_V1"


class DevelopableFanClosureLawV1(str, Enum):
    """Чем получен ярлык вершины: классификация, а не суд.

    `EXACT_PLANAR_CLOSED_FAN_V1` — веер замкнут и ТОЧНО компланарен: сумма
    углов равна `2π` по построению. `CERTIFIED_INTERVAL_ENCLOSURE_V1` —
    сертифицированная оболочка суммы отделена от `2π`. `EXACT_FAN_CLOSURE_SQRT_SUM_V1`
    — оболочка содержит `2π`, замкнутость решает точный знак: `Π(P_i + i√H_i)`,
    `P_i = A_i + B_i - C_i`, `H_i = 4 A_i B_i - P_i^2`, вещественно и положительно
    тогда и только тогда, когда сумма углов `≡ 0 (mod 2π)`; мнимая часть — элемент
    `SqrtSumV1`, и ярлык ставится по его оболочке в 256 бит либо по каноническому
    нулю. `FAN_CLOSURE_UNDECIDED_V1` — веер длиннее объявленного либо оболочка
    не разделила знак, либо бюджет точной работы кончился; ярлык
    `UNDECIDED_WORK_BUDGET`, домен при этом судит растяжение, а не ярлык.
    """

    EXACT_PLANAR_CLOSED_FAN_V1 = "EXACT_PLANAR_CLOSED_FAN_V1"
    CERTIFIED_INTERVAL_ENCLOSURE_V1 = "CERTIFIED_INTERVAL_ENCLOSURE_V1"
    EXACT_FAN_CLOSURE_SQRT_SUM_V1 = "EXACT_FAN_CLOSURE_SQRT_SUM_V1"
    FAN_CLOSURE_UNDECIDED_V1 = "FAN_CLOSURE_UNDECIDED_V1"


class VertexDevelopabilityClassV1(str, Enum):
    """Ярлык внутренней вершины развёрнутого домена. Судит растяжение, не ярлык.

    Решение владельца (2026-10-03): внутренняя вершина с ДОКАЗАННЫМ `≠ 2π`,
    растяжение которой в бюджете, принимается с ярлыком `NEAR_DEVELOPABLE` —
    ровно как near-planar принимает наклон в бюджете.
    """

    EXACT_DEVELOPABLE = "EXACT_DEVELOPABLE"
    NEAR_DEVELOPABLE = "NEAR_DEVELOPABLE"
    UNDECIDED_WORK_BUDGET = "UNDECIDED_WORK_BUDGET"


class DevelopableStraightChainLawV1(str, Enum):
    """Как объявленная ПРЯМОЙ цепь кладётся на карту развёртки.

    Хост объявляет прямой цепь, у которой нет углов (открытая, больше двух вершин), а
    очередь требует, чтобы её вершины лежали на одной прямой карты ТОЧНО. Независимая
    привязка каждой вершины к решётке этого не даёт (прямая вне осей решётки), поэтому
    цепь, чьи узлы после привязки не лежат на хорде между концами строго по порядку,
    получает внутренние вершины ровно на хорде: проекция положения вершины в предложении
    развёртки на хорду, округлённая до двоичной дроби (сдвиг вдоль хорды меньше восьмой
    доли узла, строгий порядок сохранён). Коллинеарность — по построению, а сдвиг вершины
    (её расстояние до хорды) судит судья растяжения, как любой другой сдвиг карты.
    Цепь, уже лежащая на хорде, остаётся на своих узлах. Угловое отклонение боковой
    стороны от `π` (цепь по внутренней геометрии искривлена) пишется записью и несёт
    имя отказа `DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT`, когда выпрямление не по карману.
    """

    INTERIOR_NODES_ON_ENDPOINT_SEGMENT_V1 = "INTERIOR_NODES_ON_ENDPOINT_SEGMENT_V1"


# ВНИМАНИЕ: это значение — СМЫСЛ каждого запроса, где поле `developable_stretch_budget` опущено (на проводе
# умолчание не пишется, `schema.wire_default_field`). Изменение числа молча меняет смысл всех хранимых запросов
# без поля (60 фикстур, сохранённые прогоны): его можно менять только вместе с новой версией схемы запроса
# (`DECAL_REQUEST_SCHEMA_V1` -> V2) и явной миграцией, а не правкой константы.
DEFAULT_DEVELOPABLE_STRETCH_BUDGET = Fraction(1, 5)
"""Допуск растяжения развёртки ПО УМОЛЧАНИЮ: 20 % относительно. Точная дробь.

Решение ВЛАДЕЛЬЦА (`DECISIONS.md`, 2026-10-03, «КРИВИЗНА, СТУПЕНЬ 2: S1» — бюджет был
`1/50`; тот же день, «Устраивают растяжения до 20%» — стал `1/5`): относительное
свойство поверхности, а не сантиметры. Бюджет near-planar по ширине
(`NEAR_PLANAR_WIDTH_BUDGET`) остаётся `1/50`: near-planar, которому он отказал,
по-прежнему падает на ступень развёртки, а она судит уже по этому допуску.
Условие приёма: все квадраты сингулярных чисел
отображения источник -> карта лежат в `[1/(1+b)^2, (1+b)^2]`, то есть длина вдоль
поверхности относится к длине на карте как число из `[1/(1+b), 1+b]` (двусторонне:
развёртка и сжимает, и растягивает, в отличие от проекции).

Допуск — политика ЗАПРОСА: `DecalRequestV1.developable_stretch_budget` (точная дробь из
`(0, MAX_DEVELOPABLE_STRETCH_BUDGET]`), а это значение несёт запрос без собственного поля —
старые запросы и тесты. Его читает построитель (судить), перечитывает валидатор (сверяя запись
с ЗАПРОСОМ, а не с константой) и записывает сертификат
(`DevelopableStretchCertificateV1.stretch_budget`).
"""

MAX_DEVELOPABLE_STRETCH_BUDGET = Fraction(1, 2)
"""Наибольший допуск растяжения, который вправе назвать запрос: 50 %. Точная дробь.

Выше этой границы «развёртка» перестаёт быть развёрткой поверхности (сингулярные числа
расходятся в полтора раза), и запрос с таким допуском получает именованный отказ
(`POLICY_MISMATCH` на `developable_stretch_budget`), а не карту, судимую ничем.
"""

DEVELOPABLE_ISOMETRIC_ENOUGH = Fraction(1, 50)
"""Порог «достаточно изометрично»: 2 % относительно. Точная дробь.

Карта шарнира, чьё сертифицированное растяжение не больше этого порога, ЛУЧШЕЙ не ищется:
ARAP не пробуется, и байты такой карты прежние. Выше порога ARAP тоже строит карту, и
побеждает та, у которой растяжение меньше (равенство — шарнир). Это не допуск приёма (приём
судит бюджет запроса), а порог цены поиска: он решает, стоит ли тратить второе предложение.
Прежний бюджет приёма развёртки (`1/50` до решения владельца «до 20 %»).
"""


def developable_stretch_budget_is_lawful(budget: Fraction) -> bool:
    """Допуск растяжения законен: положителен и не выше `MAX_DEVELOPABLE_STRETCH_BUDGET`."""

    return 0 < budget <= MAX_DEVELOPABLE_STRETCH_BUDGET


DEFAULT_DEVELOPABLE_STRETCH_BUDGET_V1 = ExactRationalV1(
    DEFAULT_DEVELOPABLE_STRETCH_BUDGET.numerator,
    DEFAULT_DEVELOPABLE_STRETCH_BUDGET.denominator,
)
"""Допуск по умолчанию в проводной форме: значение поля запроса, когда оно не названо."""

# ВНИМАНИЕ: как и допуск растяжения, это СМЫСЛ каждого запроса без поля `chart_reach_cap` (на проводе умолчание
# опущено): менять число можно только вместе с новой версией схемы запроса.
DEFAULT_CHART_REACH_CAP = Fraction(1, 2)
"""Досягаемость полосовой карты ПО УМОЛЧАНИЮ: полметра. Точная дробь, метры.

Политика ЗАПРОСА (`DecalRequestV1.chart_reach_cap`): наибольшая alpha, для которой карта-полоса вокруг выбранных
цепей вправе быть единственной картой домена. Полметра покрывает типичные ширины декалей на стенах и сводах (решение
оркестратора, владелец делегировал технический выбор): ползунок alpha ниже полуметра полосу не пересобирает, а выше —
получает именованный отказ `REQUEST_ALPHA_EXCEEDS_CHART_REACH`, а не молчаливо усечённую полосу.
"""

MAX_CHART_REACH_CAP = Fraction(100)
"""Наибольшая досягаемость, которую вправе назвать запрос: сто метров. Точная дробь.

Выше этого «полоса» перестаёт быть полосой (носитель карты — весь патч), и запрос получает именованный отказ
(`POLICY_MISMATCH` на `chart_reach_cap`), а не карту без стены досягаемости.
"""


def chart_reach_cap_is_lawful(cap: Fraction) -> bool:
    """Досягаемость законна: положительна и не выше `MAX_CHART_REACH_CAP`."""

    return 0 < cap <= MAX_CHART_REACH_CAP


DEFAULT_CHART_REACH_CAP_V1 = ExactRationalV1(
    DEFAULT_CHART_REACH_CAP.numerator,
    DEFAULT_CHART_REACH_CAP.denominator,
)
"""Досягаемость по умолчанию в проводной форме: значение поля запроса, когда оно не названо."""


@dataclass(frozen=True, slots=True)
class DevelopableStretchCertificateV1:
    """Растяжение треугольников источника в карту: точный суд, числа записаны.

    `triangles_outside_budget` — сколько невырожденных в карте треугольников
    не прошли точное условие приёма, `first_outside_triangle_id` — первый по
    имени. `worst_triangle_id` и `worst_band_squared_upper` называют худший
    треугольник и СЕРТИФИЦИРОВАННУЮ верхнюю границу `max(λ_max, 1/λ_min)` — число,
    которое читается, но не судит: суд — точный предикат, граница считается
    целочисленным корнем. Треугольник карты с нулевой площадью не имеет конечной
    границы, поэтому считается отдельно (`chart_degenerate_triangle_count`) и
    всегда вне бюджета. `chart_flipped_triangle_count` — треугольники с обратным
    обходом карты (знак площади против обхода источника, точно на целых).
    """

    law: DevelopableStretchLawV1
    stretch_budget: ExactRationalV1
    triangles_measured: int
    triangles_outside_budget: int
    first_outside_triangle_id: SurfaceTriangleId | None
    worst_triangle_id: SurfaceTriangleId | None
    worst_band_squared_upper: ExactRationalV1
    chart_degenerate_triangle_count: int
    chart_flipped_triangle_count: int
    first_flipped_triangle_id: SurfaceTriangleId | None

    def __post_init__(self) -> None:
        counts = (
            self.triangles_measured,
            self.triangles_outside_budget,
            self.chart_degenerate_triangle_count,
            self.chart_flipped_triangle_count,
        )
        if any(item < 0 for item in counts):
            raise ValueError("stretch counts must be non-negative")
        if self.triangles_outside_budget > self.triangles_measured:
            raise ValueError("an outside-budget triangle is a measured triangle")
        if self.chart_flipped_triangle_count > self.triangles_measured:
            raise ValueError("a flipped triangle is a measured triangle")
        if (self.first_outside_triangle_id is None) != (
            self.triangles_outside_budget == 0
        ):
            raise ValueError(
                "an outside-budget triangle is named exactly when one was counted"
            )
        if (self.first_flipped_triangle_id is None) != (
            self.chart_flipped_triangle_count == 0
        ):
            raise ValueError(
                "a flipped triangle is named exactly when one was counted"
            )
        if self.stretch_budget.numerator <= 0:
            raise ValueError("the stretch budget is positive")
        band = Fraction(
            self.worst_band_squared_upper.numerator,
            self.worst_band_squared_upper.denominator,
        )
        if band < 1:
            raise ValueError("the worst squared band is at least one")


@dataclass(frozen=True, slots=True)
class DevelopableVertexClassV1:
    """Ярлык внутренней вершины: оболочка суммы углов, закон и размер веера."""

    vertex_id: SourceVertexId
    developability_class: VertexDevelopabilityClassV1
    closure_law: DevelopableFanClosureLawV1
    angle_sum_enclosure: CertifiedDecimalIntervalV1
    fan_triangle_count: int

    def __post_init__(self) -> None:
        if self.fan_triangle_count < 3:
            raise ValueError("a closed fan has at least three triangles")
        undecided = (
            self.developability_class
            is VertexDevelopabilityClassV1.UNDECIDED_WORK_BUDGET
        )
        law_undecided = (
            self.closure_law is DevelopableFanClosureLawV1.FAN_CLOSURE_UNDECIDED_V1
        )
        if undecided != law_undecided:
            raise ValueError(
                "the undecided class and the undecided closure law come together"
            )


@dataclass(frozen=True, slots=True)
class DevelopableDeclaredChainV1:
    """Объявленная прямой цепь развёртки: вершины и доказательство боковой стороны.

    `side_angle_enclosure` — сертифицированная оболочка суммы углов веера на стороне патча у
    худшей внутренней вершины `worst_vertex_id` (прямая в карте — это ровно `π`);
    `defect_proven` — оболочка ОТДЕЛЕНА от `π`: цепь по своей внутренней геометрии
    искривлена, и прямой в карте она стала ценой растяжения соседних треугольников
    (его судит `DevelopableStretchCertificateV1`). Без доказательства (веер не стягивается
    к двум рёбрам цепи) оба поля пусты.
    """

    vertex_ids: tuple[SourceVertexId, ...]
    worst_vertex_id: SourceVertexId | None
    side_angle_enclosure: CertifiedDecimalIntervalV1 | None
    defect_proven: bool

    def __post_init__(self) -> None:
        if len(self.vertex_ids) < 3 or len(set(self.vertex_ids)) != len(self.vertex_ids):
            raise ValueError("a declared straight chain has three or more distinct vertices")
        if (self.worst_vertex_id is None) != (self.side_angle_enclosure is None):
            raise ValueError("the worst vertex and its side-angle enclosure come together")
        if self.defect_proven and self.worst_vertex_id is None:
            raise ValueError("a proven defect names its vertex")
        if self.worst_vertex_id is not None and self.worst_vertex_id not in self.vertex_ids[1:-1]:
            raise ValueError("the worst vertex is an interior vertex of the chain")


def _check_unfold_record(record, name: str, law: PlanarityAdmissionLawV1) -> None:
    """Общие условия записи карты развёртки: у целого патча и у полосы они одни."""

    if record.exact:
        raise ValueError(f"{name} describes a non-exact chart")
    if record.admission_law is not law:
        raise ValueError(f"{name} requires {law.value}")
    if record.chart_scale <= 0 or record.chart_scale_trials <= 0:
        raise ValueError("the chart scale and its trial count are positive")
    if (
        record.snapped_vertex_count < 0
        or record.boundary_loop_count < 0
        or record.chart_boundary_overlap_count < 0
    ):
        raise ValueError("unfold counts must be non-negative")
    if record.snapped_vertex_count > len(record.source_vertex_ids):
        raise ValueError("a snapped vertex is a source vertex")
    classified = {item.vertex_id for item in record.vertex_classes}
    if len(classified) != len(record.vertex_classes):
        raise ValueError("a vertex is classified once")
    if not classified <= record.source_vertex_ids:
        raise ValueError("a classified vertex is a source vertex")


@dataclass(frozen=True, slots=True)
class DevelopableUnfoldCertificateV1:
    """Запись о том, что домен РАЗВЁРНУТ: дерево, предложение, растяжение, ярлыки.

    Карта домена — обычный `RationalAffinePlanarMetricV2` с репером развёртки
    (`AffineFrameSelectionLawV1.UNFOLDED_DEVELOPMENT_FRAME_V1`): начало нуль,
    `A = e_x/S'`, `B = e_y/S'`. Координаты вершин — целые точки решётки карты
    (`chart_scale = S'`, решётка очереди — единица), так что очередь видит
    только граничные петли и семена в этих координатах, а поверхность источника
    видит только материализатор, через `snapped_source_positions`.

    `exact_plane_normal` — нормаль плоскости КАРТЫ (`(0, 0, 1)`), а не какой-либо
    плоскости источника: у развёрнутого источника плоскости нет. Поле оставлено,
    чтобы запись читалась теми же проводными проверками, что и два других
    сертификата.

    `previous_refusals` — след лестницы: имя отказа near-planar, после которого
    пробовалась развёртка, и (только у `ARAP_LOCAL_GLOBAL_80_BINARY64_V1` и
    `ARAP_CONE_RELIEF_80_BINARY64_V1`) имя отказа шарнирного предложения, после
    которого пробовалось второе предложение: запись говорит, ПОЧЕМУ
    домен здесь, а не на ступень ниже. `snapped_vertex_count` и `snap_residual` — сколько вершин
    сдвинула привязка карты к решётке и наибольшее смещение по оси в единицах
    решётки.

    `proposal_selection_law` — КАК выбрано предложение (`DevelopableProposalSelectionLawV1`);
    `hinge_chart_worst_band_squared_upper` и `arap_chart_worst_band_squared_upper` —
    сертифицированные верхние границы квадрата растяжения КАРТ двух предложений (`None`:
    предложение карты не дало либо не пробовалось), `arap_refusal` — имя отказа карты ARAP при
    `HINGE_KEPT_ARAP_REFUSED_V1` (иначе пусто). Число победителя равно
    `stretch.worst_band_squared_upper`.
    """

    certificate_id: PlanarityCertificateId
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    admission_law: PlanarityAdmissionLawV1
    exact: bool
    exact_plane_normal: ExactVector3V1
    source_vertex_ids: frozenset[SourceVertexId]
    reconstruction_law: AffineReconstructionLawV1
    tree_law: DevelopableUnfoldTreeLawV1
    proposal_law: DevelopableProposalLawV1
    lift_law: DevelopableLiftLawV1
    root_triangle_id: SurfaceTriangleId
    chart_scale: int
    chart_scale_trials: int
    stretch: DevelopableStretchCertificateV1
    proposal_worst_band_squared_upper: ExactRationalV1
    snapped_vertex_count: int
    snap_residual: ExactRationalV1
    vertex_classes: frozenset[DevelopableVertexClassV1]
    boundary_loop_count: int
    chart_boundary_overlap_count: int
    previous_refusals: tuple[str, ...]
    snapped_source_positions: frozenset[SnappedSourcePositionV1]
    straight_chain_law: DevelopableStraightChainLawV1
    declared_straight_chains: tuple[DevelopableDeclaredChainV1, ...]
    proposal_selection_law: DevelopableProposalSelectionLawV1
    hinge_chart_worst_band_squared_upper: ExactRationalV1 | None
    arap_chart_worst_band_squared_upper: ExactRationalV1 | None
    arap_refusal: str

    def __post_init__(self) -> None:
        _check_unfold_record(
            self, "DevelopableUnfoldCertificateV1", PlanarityAdmissionLawV1.DEVELOPABLE_UNFOLD_V1
        )


class BandSupportLawV1(str, Enum):
    """Каким законом выбран НОСИТЕЛЬ полосовой карты. Предложение — не власть: власть — запас в сертификате.

    `FACES_WITHIN_EUCLIDEAN_REACH_V1`: грань источника входит в носитель, если хоть одна её вершина лежит не дальше
    `D = (1 + b) * cap` (евклидово расстояние в 3D по привязанным позициям, точная дробь) от рёбер выбранных цепей;
    носитель — связная компонента таких граней (по общим рёбрам), содержащая выбранные цепи. Целыми гранями, а не
    треугольниками: полигон грани, примыкающей к ободу, обязан иметь координаты всех своих вершин. `b` — допуск
    растяжения запроса: по сертификату растяжения путь длиной `alpha` на карте — не длиннее `(1 + b) * alpha` на
    поверхности, поэтому грань дальше `D` от обода при `alpha <= cap` в карту не попадает.
    """

    FACES_WITHIN_EUCLIDEAN_REACH_V1 = "FACES_WITHIN_EUCLIDEAN_REACH_V1"


class BandBoundaryRoleV1(str, Enum):
    """Чем является сторона границы носителя: ободом, куском исходной границы или стеной досягаемости.

    `RIM` — сторона выбранной цепи (из неё растёт фронт); `ORIGINAL_BOUNDARY` — сторона прочей цепи границы патча
    (стена, как и без полосы); `REACH_WALL` — ребро, где носитель обрезан по досягаемости: границы патча там нет,
    стена искусственная, и сертификат доказывает, что фронт `alpha <= cap` до неё не доходит.
    """

    RIM = "RIM"
    ORIGINAL_BOUNDARY = "ORIGINAL_BOUNDARY"
    REACH_WALL = "REACH_WALL"


@dataclass(frozen=True, slots=True)
class BandBoundarySideV1:
    """Направленная сторона границы носителя (внутренность носителя слева, обход петли на карте против часовой).

    Ребро и концы названы по источнику; `chain_use_id` — использование цепи границы патча (обод либо прочая
    цепь), у стены досягаемости его нет.
    """

    role: BandBoundaryRoleV1
    physical_edge_id: PhysicalEdgeId
    start_vertex_id: SourceVertexId
    end_vertex_id: SourceVertexId
    chain_use_id: ChainUseId | None

    def __post_init__(self) -> None:
        if (self.role is BandBoundaryRoleV1.REACH_WALL) != (self.chain_use_id is None):
            raise ValueError("a reach wall side carries no ChainUse, every other side carries one")
        if self.start_vertex_id == self.end_vertex_id:
            raise ValueError("a boundary side joins two distinct vertices")


@dataclass(frozen=True, slots=True)
class DevelopableBandChartCertificateV1:
    """Запись о том, что ПОЛОСА вокруг выбранных цепей РАЗВЁРНУТА: носитель, стена досягаемости, запас до неё.

    Домен не развёртывается целиком (именованный отказ целого патча лежит в `previous_refusals`), но
    развёртывается носитель из граней в пределах досягаемости запроса: карта — обычная привязанная к решётке
    развёртка носителя (все поля до `arap_refusal` те же, что у `DevelopableUnfoldCertificateV1`, и судит их тот же
    суд растяжения, но по треугольникам носителя), а к ней записаны:

    * `selected_chain_use_ids`, `reach_cap`, `support_law`, `support_reach` — вход и закон выбора носителя;
    * `support_triangle_ids` — треугольники носителя (из них подъём берёт позиции и треугольники), и сколько
      треугольников патча в носитель не вошло: `excluded_triangle_count`, первый по имени;
    * `strip_boundary` — граница носителя по сторонам: обод, куски исходной границы, стена досягаемости;
    * `chart_reach_margin_squared` — ВЛАСТЬ: наименьший квадрат расстояния на карте (метры) между ободом и стеной
      досягаемости. Он не меньше `reach_cap^2`: фронт ширины `alpha <= reach_cap` растёт внутри `alpha`-окрестности
      обода на карте и до стены не доходит, поэтому усечённый носитель отвечает так же, как отвечал бы целый патч.

    Поля до `arap_refusal` повторяют `DevelopableUnfoldCertificateV1` НАМЕРЕННО: запись самостоятельная (разбор по
    вайр-идентичности, не по наследованию: кодек выбирает члены объединения по `issubclass`), а тест закрепляет
    совпадение имён, типов и порядка. `source_vertex_ids` — вершины носителя; `previous_refusals` кончается отказом
    целого патча (а у ARAP после него — отказом шарнира).
    """

    certificate_id: PlanarityCertificateId
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    admission_law: PlanarityAdmissionLawV1
    exact: bool
    exact_plane_normal: ExactVector3V1
    source_vertex_ids: frozenset[SourceVertexId]
    reconstruction_law: AffineReconstructionLawV1
    tree_law: DevelopableUnfoldTreeLawV1
    proposal_law: DevelopableProposalLawV1
    lift_law: DevelopableLiftLawV1
    root_triangle_id: SurfaceTriangleId
    chart_scale: int
    chart_scale_trials: int
    stretch: DevelopableStretchCertificateV1
    proposal_worst_band_squared_upper: ExactRationalV1
    snapped_vertex_count: int
    snap_residual: ExactRationalV1
    vertex_classes: frozenset[DevelopableVertexClassV1]
    boundary_loop_count: int
    chart_boundary_overlap_count: int
    previous_refusals: tuple[str, ...]
    snapped_source_positions: frozenset[SnappedSourcePositionV1]
    straight_chain_law: DevelopableStraightChainLawV1
    declared_straight_chains: tuple[DevelopableDeclaredChainV1, ...]
    proposal_selection_law: DevelopableProposalSelectionLawV1
    hinge_chart_worst_band_squared_upper: ExactRationalV1 | None
    arap_chart_worst_band_squared_upper: ExactRationalV1 | None
    arap_refusal: str
    selected_chain_use_ids: frozenset[ChainUseId]
    reach_cap: ExactRationalV1
    support_law: BandSupportLawV1
    support_reach: ExactRationalV1
    support_triangle_ids: frozenset[SurfaceTriangleId]
    excluded_triangle_count: int
    first_excluded_triangle_id: SurfaceTriangleId | None
    strip_boundary: tuple[BandBoundarySideV1, ...]
    chart_reach_margin_squared: ExactRationalV1

    def __post_init__(self) -> None:
        _check_unfold_record(
            self, "DevelopableBandChartCertificateV1", PlanarityAdmissionLawV1.DEVELOPABLE_BAND_CHART_V1
        )
        if not self.selected_chain_use_ids or not self.support_triangle_ids or not self.previous_refusals:
            raise ValueError("a band chart names its rim, its support and the refusal that led to it")
        if self.excluded_triangle_count < 0 or (self.first_excluded_triangle_id is None) != (
            self.excluded_triangle_count == 0
        ):
            raise ValueError("an excluded triangle is named exactly when one was counted")
        if self.reach_cap.numerator <= 0 or self.support_reach.numerator <= 0:
            raise ValueError("the reach cap and the support reach are positive")
        if not self.strip_boundary or any(
            (item.role is BandBoundaryRoleV1.RIM) != (item.chain_use_id in self.selected_chain_use_ids)
            for item in self.strip_boundary
            if item.chain_use_id is not None
        ):
            raise ValueError("a rim side is a side of a selected ChainUse and no other side is")
        if not any(item.role is BandBoundaryRoleV1.REACH_WALL for item in self.strip_boundary):
            raise ValueError("a band chart is cut by a reach wall: without one it is the whole patch")
        margin = Fraction(
            self.chart_reach_margin_squared.numerator, self.chart_reach_margin_squared.denominator
        )
        cap = Fraction(self.reach_cap.numerator, self.reach_cap.denominator)
        if margin < cap * cap:
            raise ValueError("the chart distance from the rim to the reach wall is at least the reach cap")


UNFOLDED_CERTIFICATE_TYPES = (DevelopableUnfoldCertificateV1, DevelopableBandChartCertificateV1)
"""Сертификаты карты развёртки (целого патча и полосы): разбор по вайр-типу, не по наследованию."""


def is_unfolded_certificate(certificate: object) -> bool:
    """Карта домена — развёртка: целого патча либо её полосы."""

    return type(certificate) in UNFOLDED_CERTIFICATE_TYPES


@dataclass(frozen=True, slots=True)
class RationalAffinePlanarMetricV2:
    reference_metric_id: ReferenceMetricId
    patch_domain_id: PatchDomainId
    source_revision: SourceRevision
    exact_origin: ExactPoint3V1
    exact_basis_a: ExactVector3V1
    exact_basis_b: ExactVector3V1
    exact_gram_matrix: ExactMatrix2V1
    exact_inverse_gram_matrix: ExactMatrix2V1
    exact_source_vertex_coordinates: frozenset[
        ExactSourceVertexCoordinateV2
    ]
    chart_orientation: AffineChartOrientationV1
    frame_selection_law: AffineFrameSelectionLawV1
    planarity_certificate: (
        ExactSourcePlaneCertificateV1
        | NearPlanarProjectionCertificateV1
        | DevelopableUnfoldCertificateV1
        | DevelopableBandChartCertificateV1
    )
    source_lineage: frozenset[LineageId]
    grid_certificate: IntegerGridCertificateV1


@dataclass(frozen=True, slots=True)
class EmbeddingCertifiedRationalAffinePlanarMetricV1:
    """Additive carrier for an unchanged V2 metric and embedding evidence."""

    metric: RationalAffinePlanarMetricV2
    source_snap_embedding_certificate: SourceSnapEmbeddingCertificateV1
    near_planar_projection_embedding_certificate: (
        NearPlanarProjectionEmbeddingCertificateV1 | None
    )

    def __post_init__(self) -> None:
        snap = self.source_snap_embedding_certificate
        if snap.snapping_law is not self.metric.grid_certificate.snapping_law:
            raise ValueError("snap embedding law differs from the V2 metric")
        source_ids = tuple(
            sorted(
                self.metric.planarity_certificate.source_vertex_ids,
                key=lambda item: item.value,
            )
        )
        # Привязка источника и её доказательство вложения - факты ЦЕЛОГО патча (одна решётка на все выделения), а
        # карта полосы покрывает только носитель: её вершины входят в доказательство, но не исчерпывают его.
        if type(self.metric.planarity_certificate) is DevelopableBandChartCertificateV1:
            if not set(source_ids) <= set(snap.source_vertex_ids):
                raise ValueError("a band chart vertex is outside the snap embedding vertices")
        elif snap.source_vertex_ids != source_ids:
            raise ValueError("snap embedding vertices differ from the V2 metric")
        projection = self.near_planar_projection_embedding_certificate
        planarity = self.metric.planarity_certificate
        projected = (
            planarity.projected_source_vertex_ids
            if type(planarity) is NearPlanarProjectionCertificateV1
            else frozenset()
        )
        if bool(projected) is (projection is None):
            raise ValueError(
                "projection embedding is required exactly when vertices moved"
            )
        if projection is not None and projection.source_vertex_ids != source_ids:
            raise ValueError(
                "projection embedding vertices differ from the V2 metric"
            )


@dataclass(frozen=True, slots=True)
class Binary64Point2V1:
    x: float
    y: float

    def __post_init__(self) -> None:
        if not all(isfinite(item) for item in (self.x, self.y)):
            raise ValueError("Binary64Point2V1 requires finite coordinates")


@dataclass(frozen=True, slots=True)
class Binary64Point3V1:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(item) for item in (self.x, self.y, self.z)):
            raise ValueError("Binary64Point3V1 requires finite coordinates")


@dataclass(frozen=True, slots=True)
class Binary64Vector3V1:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(item) for item in (self.x, self.y, self.z)):
            raise ValueError("Binary64Vector3V1 requires finite components")


@dataclass(frozen=True, slots=True)
class Binary64Matrix2V1:
    m00: float
    m01: float
    m10: float
    m11: float

    def __post_init__(self) -> None:
        if not all(
            isfinite(item)
            for item in (self.m00, self.m01, self.m10, self.m11)
        ):
            raise ValueError("Binary64Matrix2V1 requires finite components")


@dataclass(frozen=True, slots=True)
class Binary64SourceVertexCoordinateV1:
    source_vertex_id: SourceVertexId
    domain_coordinate: Binary64Point2V1


@dataclass(frozen=True, slots=True)
class DerivedBinary64AffineViewV1:
    origin: Binary64Point3V1
    basis_a: Binary64Vector3V1
    basis_b: Binary64Vector3V1
    gram_matrix: Binary64Matrix2V1
    inverse_gram_matrix: Binary64Matrix2V1
    source_vertex_coordinates: frozenset[
        Binary64SourceVertexCoordinateV1
    ]


@dataclass(frozen=True, slots=True)
class RuntimePredicateFilterContractV1:
    filter_law: RuntimePredicateFilterLawV1
    uncertain_result: RuntimePredicateResultV1
    exact_zero_requires_fallback: bool
    semantic_identity_law: MetricSemanticIdentityLawV1

    def __post_init__(self) -> None:
        if (
            self.uncertain_result
            is not RuntimePredicateResultV1.EXACT_FALLBACK_REQUIRED
        ):
            raise ValueError(
                "uncertain runtime predicates must require exact fallback"
            )
        if not self.exact_zero_requires_fallback:
            raise ValueError(
                "binary64 zero cannot be semantic authority"
            )


@dataclass(frozen=True, slots=True)
class RuntimeMetricFallbackContractV1:
    fallback_law: RuntimeMetricFallbackLawV1
    authoritative_reference_metric_id: ReferenceMetricId
    rounded_runtime_reconstruction_forbidden: bool

    def __post_init__(self) -> None:
        if not self.rounded_runtime_reconstruction_forbidden:
            raise ValueError(
                "runtime fallback cannot reconstruct exact facts from floats"
            )


@dataclass(frozen=True, slots=True)
class RuntimePlanarMetricV1:
    runtime_metric_id: RuntimeMetricId
    reference_metric_id: ReferenceMetricId
    derived_binary64_view: DerivedBinary64AffineViewV1
    predicate_filter_contract: RuntimePredicateFilterContractV1
    fallback_contract: RuntimeMetricFallbackContractV1

    def __post_init__(self) -> None:
        if (
            self.fallback_contract.authoritative_reference_metric_id
            != self.reference_metric_id
        ):
            raise ValueError(
                "RuntimePlanarMetricV1 must fallback to its reference metric"
            )
