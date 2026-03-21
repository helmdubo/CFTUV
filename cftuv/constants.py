from mathutils import Vector


WORLD_UP = Vector((0.0, 0.0, 1.0))

# Patch classification thresholds
FLOOR_THRESHOLD = 0.9
WALL_THRESHOLD = 0.3

# Boundary chain neighbor types
NB_MESH_BORDER = -1
NB_SEAM_SELF = -2

# Frame classification
FRAME_ALIGNMENT_THRESHOLD_H = 0.02
FRAME_ALIGNMENT_THRESHOLD_V = 0.04
# Backward-compatible alias for code paths that still expect one generic threshold.
FRAME_ALIGNMENT_THRESHOLD = FRAME_ALIGNMENT_THRESHOLD_V

# Corner detection
CORNER_ANGLE_THRESHOLD_DEG = 30.0

# Debug
GP_DEBUG_PREFIX = "CFTUV_Debug_"

# ============================================================
# Scoring weights — задокументированы в Phase 3.5
# Подобраны эмпирически на production мешах (архитектурные ассеты).
# Изменение любого веса влияет на порядок scaffold placement.
# ============================================================

# --- Root patch certainty scoring ---
# Определяет приоритет patch как root для quilt.
# Сумма весов = 1.0 (+ semantic bonus сверху).
ROOT_WEIGHT_AREA = 0.30          # Крупные patches — более стабильный seed
ROOT_WEIGHT_FRAME = 0.30         # Patches с H+V frame — надёжный scaffold каркас
ROOT_WEIGHT_FREE_RATIO = 0.20    # Меньше FREE chains = более предсказуемый layout
ROOT_WEIGHT_HOLES = 0.10         # Patches без holes — проще для conformal
ROOT_WEIGHT_BASE = 0.10          # Базовый score для любого valid patch

# --- Attachment candidate scoring ---
# Определяет силу связи patch→patch через seam.
# Сумма весов = 1.0 (минус penalties).
ATTACH_WEIGHT_SEAM = 0.25       # Длина shared seam нормализованная по max
ATTACH_WEIGHT_PAIR = 0.40       # Сила лучшей chain pair через seam (доминирует)
ATTACH_WEIGHT_TARGET = 0.20     # Root certainty целевого patch
ATTACH_WEIGHT_OWNER = 0.15      # Root certainty текущего patch

# --- Chain pair strength ---
# Определяет силу связи между двумя chains через seam.
PAIR_WEIGHT_FRAME_CONT = 0.40   # Совпадение frame role (H→H, V→V) — основной сигнал
PAIR_WEIGHT_ENDPOINT = 0.25     # Качество endpoint bridge (shared corners/verts)
PAIR_WEIGHT_CORNER = 0.10       # Сила corner anchors

# Corner detection
CORNER_ANGLE_THRESHOLD_DEG = 30.0

# Debug
GP_DEBUG_PREFIX = "CFTUV_Debug_"

# ============================================================
# Scoring weights — задокументированы в Phase 3.5
# Подобраны эмпирически на production мешах (архитектурные ассеты).
# Изменение любого веса влияет на порядок scaffold placement.
# ============================================================

# --- Root patch certainty scoring ---
# Определяет приоритет patch как root для quilt.
# Сумма весов = 1.0 (+ semantic bonus сверху).
ROOT_WEIGHT_AREA = 0.30          # Крупные patches — более стабильный seed
ROOT_WEIGHT_FRAME = 0.30         # Patches с H+V frame — надёжный scaffold каркас
ROOT_WEIGHT_FREE_RATIO = 0.20    # Меньше FREE chains = более предсказуемый layout
ROOT_WEIGHT_HOLES = 0.10         # Patches без holes — проще для conformal
ROOT_WEIGHT_BASE = 0.10          # Базовый score для любого valid patch

# --- Attachment candidate scoring ---
# Определяет силу связи patch→patch через seam.
# Сумма весов = 1.0 (минус penalties).
ATTACH_WEIGHT_SEAM = 0.25       # Длина shared seam нормализованная по max
ATTACH_WEIGHT_PAIR = 0.40       # Сила лучшей chain pair через seam (доминирует)
ATTACH_WEIGHT_TARGET = 0.20     # Root certainty целевого patch
ATTACH_WEIGHT_OWNER = 0.15      # Root certainty текущего patch

# --- Chain pair strength ---
# Определяет силу связи между двумя chains через seam.
PAIR_WEIGHT_FRAME_CONT = 0.40   # Совпадение frame role (H→H, V→V) — основной сигнал
PAIR_WEIGHT_ENDPOINT = 0.25     # Качество endpoint bridge (shared corners/verts)
PAIR_WEIGHT_CORNER = 0.10       # Сила corner anchors
PAIR_WEIGHT_SEMANTIC = 0.10     # Совпадение semantic key
PAIR_WEIGHT_EP_STRENGTH = 0.10  # Endpoint strength отдельных chains
PAIR_WEIGHT_LOOP = 0.05         # Совпадение loop kind (OUTER↔OUTER, HOLE↔HOLE)

# --- Chain frontier thresholds ---
# Контролируют когда frontier builder останавливается.
FRONTIER_PROPAGATE_THRESHOLD = 0.45  # Score выше → уверенно propagate через seam
FRONTIER_WEAK_THRESHOLD = 0.25       # Score ниже → skip (слишком слабая связь)
FRONTIER_MINIMUM_SCORE = 0.30        # Минимальный score для placement chain в frontier

# --- Continuous scoring factors (P5) ---
SCORE_FREE_LENGTH_SCALE = 0.1
SCORE_FREE_LENGTH_CAP = 0.15
SCORE_DOWNSTREAM_SCALE = 0.05
SCORE_DOWNSTREAM_CAP = 0.20
SCORE_ISOLATED_HV_PENALTY = 0.40
SCORE_FREE_STRIP_CONNECTOR = 0.10
SCORE_FREE_FRAME_NEIGHBOR = 0.05
