"""План станций цепей (`CHAIN_STATION_PLAN_V1`): какие вершины цепи декаль оставляет.

Запись плана — решение ОДИН РАЗ НА ВЕРШИНУ ЦЕПИ, принятое при компиляции по фактам снапшота. Решение не зависит от выбора цепей
(плотности, ширины, соседей по запросу): оно — функция поверхности источника вокруг вершины, поэтому оба домена общей цепи читают
одну запись и швов с лишней вершиной с одной стороны (`ADAPTER_SEAM_T_JUNCTIONS`) не бывает по построению. Закон, числа и причины —
`cftuv_envelope._chain_station`; проверяющий плана пересчитывает каждую запись по сырому снапшоту (`validation_chain_station`).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import PhysicalChainId, SourceFaceId, SourceVertexId
from ..numeric import ExactRatioV1


class ChainStationLawV1(str, Enum):
    CHAIN_STATION_PLAN_V1 = "CHAIN_STATION_PLAN_V1"


class ChainStationKindV1(str, Enum):
    """Где вершина стоит на цепи: внутри куска физической цепи либо в СТЫКЕ двух кусков одной цепи (хост режет цепь в каждом точном изломе)."""

    INTERIOR_VERTEX = "INTERIOR_VERTEX"
    CHAIN_JOINT = "CHAIN_JOINT"


class ChainStationDispositionV1(str, Enum):
    """`FREE` — декаль вершины не несёт; `REQUIRED` — несёт (причина названа)."""

    FREE = "FREE"
    REQUIRED = "REQUIRED"


class ChainStationReasonV1(str, Enum):
    """Почему вершина получила своё решение. Ровно одна причина на вершину."""

    #: FREE: цепь в вершине прямая в пределах художественного допуска, хорда в допуске, каждое поперечное ребро источника в обоих патчах
    #: инертно (грани по обе стороны ребра лежат в допуске хорды одна от плоскости другой), и грани, которые эти рёбра склеивают, плоски вместе.
    TRANSVERSE_EDGES_INERT = "TRANSVERSE_EDGES_INERT"
    #: FREE при карте домена, которая не плоскость (развёртка): UV вдоль выпрямленного ребра совпадает с прежней в пределах
    #: растяжения карты, а не точно. Решение то же; имя говорит, чем оно оплачено.
    FREE_UNDER_CHART = "FREE_UNDER_CHART"
    #: REQUIRED: поперечное ребро складывает поверхность свыше допуска хорды.
    FOLD = "FOLD"
    #: REQUIRED: рёбра по отдельности инертны, но грани, которые они склеивают, вместе глубже допуска хорды (дрейф цепочки).
    GROUP_NOT_FLAT = "GROUP_NOT_FLAT"
    #: REQUIRED: излом цепи в вершине больше художественного допуска (`CANONICAL_RESTORATION_ARTIST_ERROR`) либо цепь заворачивает назад.
    BEND_BEYOND_STRAIGHT = "BEND_BEYOND_STRAIGHT"
    #: REQUIRED: вершина стоит от прямой между соседями по цепи дальше допуска хорды (`CLIP_DIAGONAL_CHORD_BUDGET`).
    CHORD_BEYOND_BUDGET = "CHORD_BEYOND_BUDGET"
    #: REQUIRED: каждая вершина в допуске, но подряд идущие `FREE`-вершины линии цепей (куски, соединённые стыками) вместе уводят ломаную от
    #: прямой между несомыми вершинами дальше допуска хорды: дрейф малых изломов. Окно по линии жадное, слева направо.
    RUN_CHORD_BEYOND_BUDGET = "RUN_CHORD_BEYOND_BUDGET"
    #: REQUIRED: в вершине сходится другой шов или край меша, либо она вершина угла либо стыка в отношениях снапшота.
    JUNCTION = "JUNCTION"
    #: REQUIRED: поверхность другого патча цепи в вершине снапшоту неизвестна (хост её не выгрузил).
    NEIGHBOUR_SIDE_UNKNOWN = "NEIGHBOUR_SIDE_UNKNOWN"
    #: REQUIRED: ребро или вершина не многообразны (три грани на ребре, вершина дважды в грани, грань без площади, ребро цепи нулевой длины).
    NOT_MANIFOLD = "NOT_MANIFOLD"
    #: REQUIRED: положений (или граней у вершины) нет, например координатно-свободная выгрузка: сравнивать поверхности нечем.
    POSITIONS_UNAVAILABLE = "POSITIONS_UNAVAILABLE"
    #: REQUIRED: цепь замкнута — у неё нет внутренности, и разрез замкнутой цепи решает не этот закон.
    CLOSED_CHAIN = "CLOSED_CHAIN"


@dataclass(frozen=True, slots=True)
class ChainStationV1:
    """Решение по одной вершине цепи.

    `inert_face_pairs` — пары граней источника по обе стороны инертных поперечных рёбер вершины (у `FREE`; у `REQUIRED` пусто):
    резка домена не режет по этим рёбрам. Пары отсортированы по значению идентичности.
    """

    source_vertex_id: SourceVertexId
    ordinal: int
    kind: ChainStationKindV1
    disposition: ChainStationDispositionV1
    reason: ChainStationReasonV1
    inert_face_pairs: tuple[tuple[SourceFaceId, SourceFaceId], ...]


@dataclass(frozen=True, slots=True)
class ChainStationPlanV1:
    """План одной физической цепи: решение по каждой внутренней вершине и каждому стыку, по порядку вершин цепи.

    Стык двух кусков одной цепи записан в планах ОБОИХ кусков (порядковый номер 0 либо последний). `flatness_budget` — допуск хорды закона
    (метры, запись реестра `CLIP_DIAGONAL_CHORD_DEPTH_V1`), по которому решено: проверяющий сверяет его с реестром, а не принимает на слово.
    """

    law: ChainStationLawV1
    physical_chain_id: PhysicalChainId
    flatness_budget: ExactRatioV1
    stations: tuple[ChainStationV1, ...]
