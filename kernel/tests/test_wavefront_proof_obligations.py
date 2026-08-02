from __future__ import annotations

import pytest

from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.skeleton import build_skeleton
from test_wavefront_motorcycle_graph import FROZEN_DIGESTS
from test_wavefront_partial_source import (
    FIGURES_WHERE_THE_QUEUE_MATCHES_THE_STANDARD,
    FIGURES_WHERE_THE_QUEUE_REFUSES_BY_NAME,
    FIGURES_WHERE_THE_STANDARD_REFUSES_BY_NAME,
)
from wavefront_cases import named_corpus, partial_source_corpus


_FROZEN_DIGESTS_ORACLE = {
    "comb": "b454deaf48ae78496745f5609623e8cc60231a806eb4932db9b84a73d3263ff1",
    "diamond": "9ea7f5e065760f256c71e23f79b9b9a0eda3c0fc8e341da1c743af7d4712641e",
    "ell": "1c329d51eff1a96a127b7c5ebab0ea035b9be023e8ad1d557812c49c5506573e",
    "hole": "b631437d083548ec634a40e84c91d015800150a43f996d6b34a951d2f79140a1",
    "rectangle": "881d8d89ae7490bad41c6bb012e2a4acf78516a6d97e62830f4efff64e4c0808",
    "square": "05db9afe7cdbb59ca7fd4840410047c80fcd7e4e4a38fb30014517e41dbff9d8",
    "triangle": "dfdcbeed3283fab0acc04169a198b024d4cefe1776c5f03d4e9f5d21d4a6e8a3",
    "two_holes": "b5e81ff224868236ff22a164eb7ff44d0f76bd3d3a47cd12fc1baaeffe721e04",
}

# Полный pre-P0 набор: новые proof-счётчики намеренно не входят в projection.
_LEGACY_COUNTER_NAMES = ('coincident_split_targets', 'discarded_stale_candidates', 'edge_events', 'motorcycle_march_steps', 'motorcycle_trace_crashes', 'motorcycle_trace_pairs', 'motorcycle_traces', 'motorcycle_unbounded_traces', 'motorcycle_wall_crashes', 'motorcycle_wall_tests', 'multi_participant_nodes', 'peaks', 'refused_filter_beyond_trace', 'refused_filter_edge_is_own', 'refused_filter_event_in_the_past', 'refused_filter_point_outside_front', 'refused_filter_solo_vertex', 'refused_filter_span_does_not_collapse', 'refused_filter_span_is_born_zero', 'refused_filter_triple_never_concurrent', 'refused_no_rule_joint_is_antiparallel', 'refused_no_rule_joint_is_codirectional', 'refused_no_rule_joint_is_codirectional_at_different_speeds', 'refused_no_rule_meeting_not_reconnectable', 'refused_no_rule_span_vanished', 'refused_no_rule_triple_always_concurrent', 'resting_steiner_vertices', 'ridges', 'split_candidates_beyond_trace', 'split_candidates_examined', 'split_events', 'split_search_exhaustive_segments', 'split_search_exhaustive_vertices', 'start_events', 'switch_events', 'vertex_meeting_events')

_GEOMETRY_ORACLE = {
    'named::axis_square': ('EXACT', '05db9afe7cdbb59ca7fd4840410047c80fcd7e4e4a38fb30014517e41dbff9d8', (0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    'named::axis_rectangle': ('EXACT', '881d8d89ae7490bad41c6bb012e2a4acf78516a6d97e62830f4efff64e4c0808', (0, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'named::right_triangle': ('EXACT', 'dfdcbeed3283fab0acc04169a198b024d4cefe1776c5f03d4e9f5d21d4a6e8a3', (0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    'named::diamond': ('EXACT', '9ea7f5e065760f256c71e23f79b9b9a0eda3c0fc8e341da1c743af7d4712641e', (0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    'named::ell': ('EXACT', '1c329d51eff1a96a127b7c5ebab0ea035b9be023e8ad1d557812c49c5506573e', (0, 4, 2, 2, 0, 0, 1, 0, 1, 8, 1, 0, 0, 2, 0, 0, 0, 0, 0, 4, 2, 0, 0, 0, 0, 4, 0, 2, 0, 4, 0, 0, 0, 0, 0, 1)),
    'named::comb_2': ('EXACT', '9a1dffb281c70512a809e6428f31e01cdbd58ad0497f5b7512b5685aec8070c2', (0, 5, 4, 2, 0, 0, 2, 0, 2, 10, 2, 2, 1, 4, 3, 0, 0, 0, 0, 6, 1, 0, 0, 0, 0, 2, 0, 1, 1, 9, 2, 0, 0, 0, 0, 0)),
    'named::comb_4': ('EXACT', '0ef329cb92721b956d7d919a206122967e5d2d777ddcb7951d6a890d866ad286', (0, 5, 6, 4, 0, 0, 4, 0, 4, 20, 4, 4, 1, 8, 5, 0, 0, 0, 0, 18, 1, 0, 0, 0, 0, 2, 0, 1, 1, 21, 4, 0, 0, 0, 0, 0)),
    'named::hole_1': ('EXACT', '2d0cccbced1e398ba31bf65a95fbfeded79c800e9d67c59d9c8dd2ee26b7e026', (0, 8, 0, 4, 0, 0, 4, 0, 4, 28, 4, 0, 0, 8, 12, 0, 0, 0, 0, 8, 4, 0, 0, 0, 0, 8, 0, 4, 0, 24, 0, 0, 0, 0, 0, 4)),
    'named::holes_2': ('EXACT', '5dc87460d9ef2f34ff733bdc9188b0a4beb429fb1bc0aa0d539d5407ed335df1', (0, 16, 2, 8, 0, 2, 8, 0, 8, 52, 4, 0, 2, 16, 32, 0, 0, 0, 0, 24, 6, 0, 0, 0, 0, 24, 0, 7, 2, 72, 4, 0, 0, 0, 0, 4)),
    'named::cross': ('EXACT', 'a5205821ef102efb12752307e917214b535e60a8379f938d481fc19aadbc74cf', (0, 8, 4, 8, 4, 4, 4, 0, 4, 72, 3, 0, 12, 12, 8, 10, 0, 0, 0, 14, 4, 0, 0, 0, 0, 16, 4, 4, 12, 52, 0, 0, 2, 0, 0, 3)),
    'named::staircase': ('EXACT', '6ff77e1481648aa6a0bd02c7c328cc91ccf7bab50c4e8a7cf63519fff350f9b7', (0, 5, 3, 2, 0, 0, 2, 0, 2, 14, 1, 1, 0, 4, 0, 3, 0, 0, 0, 12, 2, 0, 0, 0, 0, 4, 0, 2, 0, 14, 2, 0, 0, 0, 0, 0)),
    'named::u_shape': ('EXACT', 'aa1bd005e64bc4209d29db5052bffe68a9ce6eab2b44ba2e8d98e3c75a027b2e', (0, 6, 2, 4, 0, 0, 2, 0, 2, 16, 2, 0, 0, 4, 3, 0, 0, 0, 0, 5, 3, 0, 0, 0, 0, 6, 0, 3, 0, 9, 0, 0, 0, 0, 0, 2)),
    'named::double_notch': ('EXACT', '3bd4165324e74884ab51f8934e1fdf2257f4ea175a2745555c7c109fbce47857', (2, 15, 6, 4, 1, 1, 4, 0, 4, 29, 0, 0, 3, 8, 6, 3, 0, 0, 0, 28, 4, 0, 0, 0, 0, 10, 1, 5, 3, 42, 4, 0, 0, 0, 0, 0)),
    'named::field_building_002_scale_64': ('EXACT', '0822d28b997b15b656d3fc7530cafea2237165a3c53e8fdc15b8670b486adbd0', (1, 17, 6, 11, 1, 1, 4, 0, 4, 52, 3, 3, 14, 12, 28, 18, 0, 0, 0, 0, 0, 0, 0, 3, 0, 11, 1, 2, 14, 71, 4, 0, 0, 0, 0, 0)),
    'named::field_building_002_scale_256': ('EXACT', '2f52bd3d83486d3e78e36d7e568c2e59ba726eba310030e725548151f939cc59', (0, 16, 6, 10, 1, 1, 4, 0, 4, 50, 3, 3, 11, 12, 29, 18, 0, 0, 0, 0, 0, 0, 0, 2, 0, 11, 1, 2, 11, 67, 4, 0, 0, 0, 0, 0)),
    'named::field_building_002_scale_1024': ('EXACT', '36e0bb35570a7f96d8733a885f16c822acad00b6fbbdda0ab2ed7b000a0faafd', (0, 16, 6, 9, 1, 1, 4, 0, 4, 46, 3, 3, 17, 12, 23, 18, 0, 0, 0, 0, 0, 0, 0, 2, 0, 11, 1, 2, 17, 67, 4, 0, 0, 0, 0, 0)),
    'named::star_9_seed_0': ('EXACT', 'fbfce27b0ddf640fbf24bb3fee2bab49fc7640de1ae02e1652b0bcec2903e0ea', (0, 15, 6, 5, 0, 0, 1, 0, 1, 26, 1, 1, 0, 4, 2, 9, 0, 0, 0, 0, 0, 0, 0, 1, 0, 5, 0, 1, 0, 17, 1, 0, 0, 0, 0, 0)),
    'named::star_9_seed_1': ('EXACT', '43d0280e3801b9cb279b9eac3289ce98b5d61f59cde691e7a8b2e25bbb77bb64', (0, 11, 6, 3, 0, 0, 1, 0, 1, 14, 2, 2, 0, 2, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 1, 0, 0, 0, 0, 0)),
    'named::star_9_seed_2': ('EXACT', '1da5c7f79326ba27d35ad36ee448ab6629b953cddaaa3e729a90d638b226e337', (0, 12, 7, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    'named::star_9_seed_3': ('EXACT', '9da7ec390199ae1b4af466cd76c9a55e6b3ee1bb9ffebf825e0774aa3c0bc854', (0, 12, 7, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    'named::star_9_seed_4': ('EXACT', 'ebbdd1d8ad75723d07d2fe586b1c67373871c3b7f82b68a9f831c344b6418785', (0, 21, 4, 13, 2, 3, 3, 0, 3, 71, 1, 1, 8, 12, 6, 25, 0, 0, 0, 0, 0, 0, 0, 3, 0, 18, 2, 3, 8, 60, 3, 0, 0, 0, 0, 0)),
    'named::star_9_seed_5': ('EXACT', '5fdb90742c4629a1f2027d3efe1301f56c935a714627714124780a141284f8c9', (0, 14, 6, 5, 0, 0, 1, 0, 1, 25, 1, 1, 0, 4, 2, 10, 0, 0, 0, 0, 0, 0, 0, 1, 0, 5, 0, 1, 0, 17, 1, 0, 0, 0, 0, 0)),
    'partial_source::rect_12x8_source_edge_0': ('EXACT', 'c135a7e6a9146b68a2db65b5979ee98ce77a433f1053662b12149d1cd24e6544', (0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 1, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'partial_source::rect_12x8_source_edge_1': ('EXACT', 'ff795e17753fa6e8786cef33709f93306474aa09bac97a73b51eb0bc1b5811cc', (0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 1, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'partial_source::rect_12x8_source_edge_2': ('EXACT', '92bf9fc28735c26c6ca7a84478aee3a5b6d8f769bdd2c0d9c669272b8b5cc6cc', (0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'partial_source::rect_12x8_source_edge_3': ('EXACT', 'b94653bef992c9589a5929920e626b706c1c4487e8ba0864c76bb01809207b46', (0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'partial_source::rect_12x8_source_edges_0_1': ('EXACT', '73236dcedd31c47a1302ca55b4ede4b0b5dad0017432ba44470f6b6ba6d5fe50', (0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'partial_source::rect_12x8_source_edges_1_2': ('EXACT', 'fe2c6da3c56a51bbbc777156ecb70ae00c4b88b2ff1a5457a1a19a71140061d9', (0, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'partial_source::rect_12x8_source_edges_2_3': ('EXACT', '1a4aed8db8190a8532adffab5bacdeb48aad55f765fefb3984f8da5061cc35c6', (0, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'partial_source::rect_12x8_source_edges_3_0': ('EXACT', 'cf8704aa4a0d967b343094324f86e1c885ef95b6521621f319fe6b0085384072', (0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'partial_source::ell_12_source_edge_0': ('EXACT', '9534363ec7c7a7dcce91f024d40060d68a12dd0e5a1316f8f0aac970220ca85a', (0, 0, 3, 16, 0, 0, 1, 1, 0, 64, 0, 0, 0, 2, 0, 0, 0, 2, 0, 8, 2, 0, 0, 0, 0, 4, 0, 2, 0, 4, 1, 0, 1, 0, 0, 0)),
    'partial_source::ell_12_source_edge_1': ('WAVEFRONT_LEFT_UNRESOLVED', '39f1189cfec5b84f6c8f83f6b30a55344ebc11e9b4005508b013288ece1a8ff4', (0, 3, 1, 17, 0, 0, 1, 1, 0, 68, 0, 0, 0, 4, 1, 0, 0, 1, 0, 11, 0, 0, 0, 1, 0, 5, 0, 1, 0, 11, 1, 0, 2, 0, 0, 0)),
    'partial_source::ell_12_source_edge_2': ('WAVEFRONT_LEFT_UNRESOLVED', 'b6fa3eaa25ff6acd400f605084325fd8417ea7ba1bd4b24b8aa9084834f5e2c3', (0, 1, 1, 1, 0, 0, 1, 0, 1, 4, 0, 0, 0, 2, 2, 0, 0, 1, 0, 8, 0, 0, 0, 0, 0, 2, 0, 1, 0, 4, 1, 0, 0, 0, 0, 0)),
    'partial_source::ell_12_source_edge_3': ('WAVEFRONT_LEFT_UNRESOLVED', '673833a9c663d0235d33de374eaf6066d0df0666280cae9416cce003c2ba4917', (0, 0, 1, 1, 0, 0, 1, 0, 1, 4, 0, 0, 0, 2, 2, 0, 0, 1, 0, 8, 1, 0, 0, 0, 0, 2, 0, 1, 0, 4, 1, 0, 0, 0, 0, 0)),
    'partial_source::ell_12_source_edge_4': ('WAVEFRONT_LEFT_UNRESOLVED', 'ac61d405de5364475c5464d246f943644c38b15f52695ebf812f420c31006ff7', (0, 2, 1, 17, 0, 0, 1, 1, 0, 68, 0, 0, 0, 4, 1, 0, 0, 2, 0, 9, 0, 0, 1, 1, 0, 6, 0, 1, 0, 11, 1, 0, 2, 0, 0, 0)),
    'partial_source::ell_12_source_edge_5': ('EXACT', '495caa2e543f62ee18c82b4c2aeca8dad6f0dc1e72420e68a39d0f8e13487fd6', (0, 2, 3, 16, 0, 0, 1, 1, 0, 64, 0, 0, 0, 2, 0, 0, 0, 2, 0, 8, 0, 0, 0, 0, 0, 4, 0, 2, 0, 4, 1, 0, 1, 0, 0, 0)),
    'partial_source::ell_12_source_edges_0_1': ('EXACT', '7377c3aa10b5d74b1e5fac0791fda8d63841577fdc5d1c80edd503de67f5c0bc', (1, 3, 3, 16, 0, 0, 1, 1, 0, 64, 0, 0, 0, 2, 0, 0, 0, 1, 0, 5, 2, 0, 0, 1, 0, 4, 0, 2, 0, 4, 1, 0, 1, 0, 0, 0)),
    'partial_source::ell_12_source_edges_1_2': ('WAVEFRONT_LEFT_UNRESOLVED', 'd824374aa42ec989f8bf7ca676207a2ef214ac3c7a0fbc0f3a7ea3e0c57643d0', (1, 3, 1, 1, 0, 0, 1, 0, 1, 4, 0, 0, 0, 2, 2, 0, 0, 1, 0, 5, 0, 0, 0, 1, 0, 2, 0, 1, 0, 4, 1, 0, 0, 0, 0, 0)),
    'partial_source::ell_12_source_edges_2_3': ('EXACT', '60b2a39172741262e6b3afede51c3d864fccce62dd6043d6d5c66a3e1aea6c04', (0, 1, 2, 2, 0, 0, 1, 0, 1, 8, 1, 0, 0, 2, 4, 0, 0, 2, 0, 2, 1, 0, 0, 0, 0, 4, 0, 2, 0, 4, 0, 0, 0, 0, 0, 1)),
    'partial_source::ell_12_source_edges_3_4': ('WAVEFRONT_LEFT_UNRESOLVED', '04207bb2a1cb57fcf945b43b61513e219415557287108bc528a49c41b69e9719', (1, 2, 1, 1, 0, 0, 1, 0, 1, 4, 0, 0, 0, 2, 2, 0, 0, 1, 0, 5, 1, 0, 0, 2, 0, 2, 0, 1, 0, 4, 1, 0, 0, 0, 0, 0)),
    'partial_source::ell_12_source_edges_4_5': ('EXACT', '80c172adfa5e3e457450f1910cb6f15cbe7adfac22ef5096f0a2c5a3ebb010b3', (1, 4, 3, 16, 0, 0, 1, 1, 0, 64, 0, 0, 0, 2, 0, 0, 0, 1, 0, 5, 1, 0, 0, 2, 0, 4, 0, 2, 0, 4, 1, 0, 1, 0, 0, 0)),
    'partial_source::ell_12_source_edges_5_0': ('EXACT', '9096be36264c4e5b00231dcd3d22f6a8ed66e9460fa43d54bf4709254b468722', (0, 3, 2, 16, 0, 0, 1, 1, 0, 64, 1, 0, 0, 2, 0, 0, 0, 2, 0, 4, 1, 0, 0, 0, 0, 4, 0, 2, 0, 4, 0, 0, 1, 0, 0, 1)),
    'partial_source::staircase_source_edge_0': ('EXACT', '63f204632a794fdb8bf22959ffda78820561713489359947f4500a2c8bf00a59', (0, 2, 4, 32, 0, 0, 2, 2, 0, 224, 0, 0, 0, 4, 0, 0, 0, 3, 0, 18, 3, 0, 0, 0, 0, 6, 0, 3, 0, 14, 2, 0, 2, 0, 0, 0)),
    'partial_source::staircase_source_edge_1': ('WAVEFRONT_LEFT_UNRESOLVED', '441de8b426b59e180bd6c76a2bb9bd17e1422b0bb2c189c905de2230c80fe696', (0, 4, 1, 33, 0, 0, 2, 2, 0, 231, 0, 0, 0, 6, 1, 3, 0, 1, 0, 21, 0, 0, 0, 1, 0, 5, 0, 1, 0, 23, 1, 0, 3, 0, 0, 0)),
    'partial_source::staircase_source_edge_2': ('WAVEFRONT_LEFT_UNRESOLVED', '5ae64f0f07ce5eb06e7499809e5d2a35f0c63c7ccb5649a3256774dbefeaf00a', (0, 1, 1, 17, 0, 0, 2, 1, 1, 119, 0, 0, 0, 4, 4, 0, 0, 1, 0, 18, 0, 0, 0, 0, 0, 2, 0, 1, 0, 14, 1, 0, 1, 0, 0, 0)),
    'partial_source::staircase_source_edge_3': ('WAVEFRONT_LEFT_UNRESOLVED', '3bb19a96f2f327eb5b71a0e0f0e72b921354bc7b9c593e538a3f1dce0c744279', (0, 3, 1, 34, 0, 0, 2, 1, 1, 222, 0, 0, 0, 8, 6, 1, 0, 1, 0, 24, 0, 0, 0, 1, 0, 7, 0, 1, 0, 30, 1, 0, 2, 0, 0, 0)),
    'partial_source::staircase_source_edge_4': ('WAVEFRONT_LEFT_UNRESOLVED', 'd2e0ac6d5c1609311988b3f53cfedcf0d63c8e83570e6d4d7689b41190557f6c', (0, 2, 0, 18, 0, 0, 2, 1, 1, 126, 0, 0, 0, 6, 4, 1, 0, 2, 0, 19, 0, 0, 1, 1, 0, 8, 0, 1, 0, 23, 2, 0, 1, 0, 0, 0)),
    'partial_source::staircase_source_edge_5': ('WAVEFRONT_LEFT_UNRESOLVED', 'e9fd3ef992c49d4b057daac7d51325017c87e317daaf5efbd5d11c763c678a3b', (0, 0, 1, 17, 0, 0, 2, 1, 1, 119, 0, 0, 0, 4, 4, 0, 0, 1, 0, 18, 1, 0, 0, 0, 0, 2, 0, 1, 0, 14, 1, 0, 1, 0, 0, 0)),
    'partial_source::staircase_source_edge_6': ('WAVEFRONT_LEFT_UNRESOLVED', 'e50fb591e8ac8fb43cbe618f0b6f8bf6af9fd0334e59ce2e8b9a6b99d565c23e', (0, 2, 1, 34, 0, 0, 2, 2, 0, 238, 0, 0, 0, 8, 5, 3, 0, 4, 0, 24, 0, 0, 2, 1, 0, 11, 0, 1, 0, 33, 3, 0, 4, 0, 0, 0)),
    'partial_source::staircase_source_edge_7': ('EXACT', '81500913c8b3ccab32f0b53998020ce04094dd13be988e9e0e94d4f18aade289', (1, 4, 4, 32, 0, 0, 2, 2, 0, 224, 0, 0, 0, 4, 0, 0, 0, 3, 0, 18, 0, 0, 0, 0, 0, 6, 0, 3, 0, 14, 2, 0, 2, 0, 0, 0)),
    'partial_source::staircase_source_edges_0_1': ('EXACT', '5f766e3f3480440ca3b15ae375c83a4a68bf45b70aef83722caa434ac9ac15c4', (1, 5, 4, 32, 0, 0, 2, 2, 0, 224, 0, 0, 0, 4, 0, 1, 0, 2, 0, 14, 3, 0, 0, 1, 0, 6, 0, 3, 0, 14, 2, 0, 2, 0, 0, 0)),
    'partial_source::staircase_source_edges_1_2': ('WAVEFRONT_LEFT_UNRESOLVED', 'cc1af9b6e43ad2248d4af955beb5294b2b9a82dd358002a54e900375f43ee08f', (1, 3, 1, 17, 0, 0, 2, 1, 1, 119, 0, 0, 0, 4, 4, 1, 0, 1, 0, 14, 0, 0, 0, 1, 0, 2, 0, 1, 0, 14, 1, 0, 1, 0, 0, 0)),
    'partial_source::staircase_source_edges_2_3': ('WAVEFRONT_LEFT_UNRESOLVED', 'd262c1ba312a066f237d392b4d001ab430ffb76db7c9ed8f99e0f4bfd3217527', (0, 4, 2, 18, 0, 0, 2, 1, 1, 126, 0, 0, 1, 6, 9, 1, 0, 2, 0, 16, 0, 0, 0, 1, 0, 7, 0, 2, 1, 24, 2, 0, 2, 0, 0, 0)),
    'partial_source::staircase_source_edges_3_4': ('WAVEFRONT_LEFT_UNRESOLVED', 'f1898203cfa6f3eec9b84b31afb94885979272e90f4695fdc4abba66b5970b2a', (2, 4, 0, 19, 2, 1, 2, 0, 2, 116, 0, 0, 2, 8, 13, 0, 0, 1, 0, 14, 0, 0, 1, 2, 0, 12, 2, 2, 2, 35, 2, 0, 2, 0, 0, 0)),
    'partial_source::staircase_source_edges_4_5': ('WAVEFRONT_LEFT_UNRESOLVED', 'ef979ffc412ffdd01c2b3935ce1fbd78485380b3faf05a37f9dbf4b79f7ea7b5', (0, 2, 2, 18, 0, 0, 2, 1, 1, 126, 0, 0, 1, 6, 9, 1, 0, 3, 0, 14, 1, 0, 1, 1, 0, 8, 0, 2, 1, 24, 2, 0, 2, 0, 0, 0)),
    'partial_source::staircase_source_edges_5_6': ('WAVEFRONT_LEFT_UNRESOLVED', 'a24c1db7cc3c32b532edc8885385ae1ee795e6d2b7a3aeb4028df5f516a9c528', (1, 2, 1, 17, 0, 0, 2, 1, 1, 119, 0, 0, 0, 4, 4, 1, 0, 1, 0, 14, 1, 0, 0, 2, 0, 2, 0, 1, 0, 14, 1, 0, 1, 0, 0, 0)),
    'partial_source::staircase_source_edges_6_7': ('EXACT', '19d1070085299e67474e40a46ef60ceeea616ef78e715aed053b3aa0193b87ec', (2, 6, 4, 32, 0, 0, 2, 2, 0, 224, 0, 0, 0, 4, 0, 1, 0, 2, 0, 14, 1, 0, 0, 2, 0, 6, 0, 3, 0, 14, 2, 0, 2, 0, 0, 0)),
    'partial_source::staircase_source_edges_7_0': ('EXACT', '7b2b0cd18ba7d7969acae259534be5417170f4d2398f423baee05c185f9f3a93', (0, 4, 3, 32, 0, 0, 2, 2, 0, 224, 1, 1, 0, 4, 0, 3, 0, 2, 0, 12, 1, 0, 0, 0, 0, 4, 0, 2, 0, 14, 2, 0, 2, 0, 0, 0)),
    'partial_source::ell_12_source_without_the_reflex_pair': ('EXACT', '2704f577047d526992c512c65f4856ef66345bac78e0b57a0a5ff541314deb91', (3, 6, 1, 17, 0, 0, 1, 1, 0, 69, 1, 0, 0, 4, 4, 0, 0, 0, 0, 0, 1, 0, 0, 2, 0, 5, 0, 2, 0, 9, 1, 0, 2, 0, 0, 0)),
    'partial_source::rect_12x8_source_bottom_at_double_speed': ('EXACT', 'a25c938cd0bbad73fb4772684c331b8e3977bff82d54e0521f09add18fbd2568', (0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 1, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0)),
    'partial_source::ell_12_all_sources_bottom_at_double_speed': ('EXACT', 'd1b03a48d0cafdb3712403c0812b1d7a33544f50bd406c5d3657f2bcb2a5a5be', (0, 5, 3, 2, 0, 0, 1, 0, 1, 8, 0, 0, 0, 2, 0, 1, 0, 0, 0, 4, 2, 0, 0, 0, 0, 4, 0, 2, 0, 4, 1, 0, 0, 0, 0, 0)),
    'partial_source::staircase_all_sources_bottom_at_double_speed': ('EXACT', '6c0a61fc52ba3aa9d0b694086b80fa87b849c8b2cdec2017a943939ac25d35cf', (0, 8, 4, 2, 0, 0, 2, 0, 2, 14, 0, 0, 0, 4, 0, 3, 0, 0, 0, 12, 3, 0, 0, 0, 0, 6, 0, 3, 0, 14, 2, 0, 0, 0, 0, 0)),
    'partial_source::ell_12_all_sources_mixed_speeds_at_the_reflex_vertex': ('EXACT', '139f3a89c2a40fa9a0eebcdd738fb93818547dffe06e422863c8a6a2dfba8064', (0, 4, 2, 2, 0, 0, 1, 0, 1, 8, 1, 0, 0, 2, 2, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 4, 0, 2, 0, 4, 0, 0, 0, 0, 0, 1)),
}

_CASES = (
    *((f"named::{name}", polygon) for name, polygon in named_corpus()),
    *((f"partial_source::{name}", polygon) for name, polygon in partial_source_corpus()),
)


@pytest.mark.parametrize(
    "case_id,polygon",
    _CASES,
    ids=tuple(case_id for case_id, _ in _CASES),
)
def test_p0_geometry_and_existing_counter_oracle(case_id, polygon):
    expected_outcome, expected_digest, expected_counter_values = _GEOMETRY_ORACLE[case_id]
    skeleton = build_skeleton(polygon)
    counter_map = dict(skeleton.counters)
    assert all(name in counter_map for name in _LEGACY_COUNTER_NAMES)
    assert skeleton.outcome.value == expected_outcome
    assert semantic_digest(skeleton) == expected_digest
    assert tuple(counter_map[name] for name in _LEGACY_COUNTER_NAMES) == expected_counter_values


def test_p0_oracle_covers_all_63_cases_exactly():
    case_ids = tuple(case_id for case_id, _ in _CASES)
    assert len(case_ids) == 63
    assert len(set(case_ids)) == 63
    assert set(case_ids) == set(_GEOMETRY_ORACLE)
    assert len(_LEGACY_COUNTER_NAMES) == 36
    assert len(set(_LEGACY_COUNTER_NAMES)) == 36


def test_p0_frozen_digest_and_partial_source_oracles_are_unchanged():
    assert FROZEN_DIGESTS == _FROZEN_DIGESTS_ORACLE
    assert (
        FIGURES_WHERE_THE_QUEUE_MATCHES_THE_STANDARD,
        FIGURES_WHERE_THE_QUEUE_REFUSES_BY_NAME,
        FIGURES_WHERE_THE_STANDARD_REFUSES_BY_NAME,
    ) == (23, 17, 1)

