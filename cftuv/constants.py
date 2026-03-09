from mathutils import Vector


WORLD_UP = Vector((0.0, 0.0, 1.0))

# Patch classification thresholds
FLOOR_THRESHOLD = 0.9
WALL_THRESHOLD = 0.3

# Boundary chain neighbor types
NB_MESH_BORDER = -1
NB_SEAM_SELF = -2

# Frame classification
FRAME_ALIGNMENT_THRESHOLD = 0.08

# Corner detection
CORNER_ANGLE_THRESHOLD_DEG = 30.0

# Debug
GP_DEBUG_PREFIX = "CFTUV_Debug_"
