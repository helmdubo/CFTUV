from __future__ import annotations

from types import SimpleNamespace

from mathutils import Vector

from cftuv import debug


class _FakeFrames(list):
    def new(self, frame_number):
        frame = SimpleNamespace(frame_number=frame_number)
        self.append(frame)
        return frame


class _FakeLayer:
    def __init__(self, name):
        self.name = name
        self.frames = _FakeFrames()

    def clear(self):
        self.frames.clear()


class _FakeLayers(list):
    def new(self, name, set_active=False):
        del set_active
        layer = _FakeLayer(name)
        self.append(layer)
        return layer


def test_rail_visualization_reads_immutable_ledger_and_uses_canonical_colors(monkeypatch):
    gp_data = SimpleNamespace(layers=_FakeLayers())
    gp_obj = SimpleNamespace(data=gp_data)
    materials = {}
    strokes = []

    monkeypatch.setattr(debug, '_get_or_create_decal_rail_gp_object', lambda source: gp_obj)

    def ensure_material(_gp_data, name, color):
        materials[name] = color
        return len(materials) - 1

    def add_stroke(frame, points, material, line_width=4, cyclic=False):
        strokes.append(
            (
                next(
                    layer.name
                    for layer in gp_data.layers
                    if any(item is frame for item in layer.frames)
                ),
                tuple(tuple(point) for point in points),
                material,
                line_width,
                cyclic,
            )
        )

    monkeypatch.setattr(debug, '_ensure_gp_material', ensure_material)
    monkeypatch.setattr(debug, '_add_gp_stroke', add_stroke)

    plan = SimpleNamespace(
        ledger=SimpleNamespace(
            vertices=(
                SimpleNamespace(vertex_id=10, position=(0.0, 0.0, 0.0)),
                SimpleNamespace(vertex_id=11, position=(0.0, 2.0, 0.0)),
                SimpleNamespace(vertex_id=12, position=(3.0, 1.0, 0.0)),
            ),
            edges=(SimpleNamespace(edge_id=20, vertex_ids=(10, 11)),),
            routes=(
                SimpleNamespace(
                    route_id=30,
                    stations=(
                        SimpleNamespace(station_index=0, kind='START', source_vertex_id=10),
                        SimpleNamespace(
                            station_index=1,
                            kind='REGULAR',
                            source_edge_id=20,
                            edge_parameter=0.5,
                        ),
                        SimpleNamespace(station_index=2, kind='END', source_vertex_id=11),
                    )
                ),
            ),
            events=(
                SimpleNamespace(kind='DAM', source_edge_id=20, edge_parameter=0.5),
                SimpleNamespace(kind='POLE', source_vertex_id=11),
                SimpleNamespace(kind='MERGE', source_vertex_id=10),
                SimpleNamespace(kind='RAIL_MARK', route_id=30, station_index=1),
            ),
        )
    )

    assert debug.create_decal_rail_visualization(plan, SimpleNamespace()) is gp_obj

    assert [layer.name for layer in gp_data.layers] == [
        'Rails',
        'Dams',
        'Poles',
        'Merges',
        'Marks',
    ]
    assert materials == {
        'CFTUV_DecalRail_Rails': (0.0, 1.0, 1.0, 1.0),
        'CFTUV_DecalRail_Dams': (1.0, 0.0, 0.0, 1.0),
        'CFTUV_DecalRail_Poles': (1.0, 0.0, 1.0, 1.0),
        'CFTUV_DecalRail_Merges': (0.0, 1.0, 0.0, 1.0),
        'CFTUV_DecalRail_Marks': (1.0, 1.0, 0.0, 1.0),
    }
    assert sum(layer_name == 'Rails' for layer_name, *_rest in strokes) == 1
    assert sum(layer_name == 'Dams' for layer_name, *_rest in strokes) == 3
    assert sum(layer_name == 'Poles' for layer_name, *_rest in strokes) == 3
    assert sum(layer_name == 'Merges' for layer_name, *_rest in strokes) == 3
    assert sum(layer_name == 'Marks' for layer_name, *_rest in strokes) == 3
    rail_stroke = next(stroke for stroke in strokes if stroke[0] == 'Rails')
    assert rail_stroke[1] == (
        (0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 2.0, 0.0),
    )


class _NamedStore(dict):
    def __iter__(self):
        return iter(self.values())

    def remove(self, item, **_kwargs):
        self.pop(item.name, None)


def test_clear_rail_visualization_preserves_analyze_overlay(monkeypatch):
    source = SimpleNamespace(name='Wall')
    analyze_name = debug._get_gp_debug_name(source)
    rail_name = debug._get_decal_rail_gp_name(source)
    analyze_data = SimpleNamespace(name=analyze_name)
    rail_data = SimpleNamespace(name=rail_name)
    analyze_obj = SimpleNamespace(name=analyze_name, data=analyze_data)
    rail_obj = SimpleNamespace(name=rail_name, data=rail_data)
    objects = _NamedStore({analyze_name: analyze_obj, rail_name: rail_obj})
    gp_data = _NamedStore({analyze_name: analyze_data, rail_name: rail_data})
    fake_bpy = SimpleNamespace(
        data=SimpleNamespace(
            objects=objects,
            grease_pencils_v3=gp_data,
            grease_pencils=None,
            materials=_NamedStore(),
        )
    )
    monkeypatch.setattr(debug, 'bpy', fake_bpy)

    debug.clear_decal_rail_visualization(source)

    assert rail_name not in objects
    assert rail_name not in gp_data
    assert analyze_name in objects
    assert analyze_name in gp_data


def test_none_plan_only_clears_existing_rail_preview(monkeypatch):
    source = SimpleNamespace(name='Wall')
    cleared = []
    monkeypatch.setattr(debug, 'clear_decal_rail_visualization', cleared.append)
    monkeypatch.setattr(
        debug,
        '_get_or_create_decal_rail_gp_object',
        lambda _source: (_ for _ in ()).throw(AssertionError('GP object must not be created')),
    )

    assert debug.create_decal_rail_visualization(None, source) is None
    assert cleared == [source]


def test_rail_item_points_supports_station_vectors_and_plain_tuples():
    station_rail = SimpleNamespace(
        stations=(
            SimpleNamespace(co=Vector((1.0, 2.0, 3.0))),
            SimpleNamespace(co=(4.0, 5.0, 6.0)),
        )
    )
    tuple_rail = SimpleNamespace(points=((7.0, 8.0), (9.0, 10.0)))

    assert [tuple(point) for point in debug._rail_item_points(station_rail)] == [
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    ]
    assert [tuple(point) for point in debug._rail_item_points(tuple_rail)] == [
        (7.0, 8.0, 0.0),
        (9.0, 10.0, 0.0),
    ]


class _FakeLegacyPoints(list):
    def add(self, count):
        self.extend(SimpleNamespace() for _index in range(count))


def test_gp_stroke_creation_keeps_legacy_and_v3_compatibility():
    legacy_stroke = SimpleNamespace(points=_FakeLegacyPoints())
    legacy_strokes = SimpleNamespace(new=lambda: legacy_stroke)
    legacy_frame = SimpleNamespace(strokes=legacy_strokes)
    assert debug._new_gp_stroke(legacy_frame, 2) is legacy_stroke
    assert len(legacy_stroke.points) == 2

    class Drawing:
        def __init__(self):
            self.strokes = []

        def add_strokes(self, sizes):
            self.strokes.extend(
                SimpleNamespace(points=[SimpleNamespace() for _index in range(size)])
                for size in sizes
            )

    drawing = Drawing()
    v3_frame = SimpleNamespace(drawing=drawing)
    v3_stroke = debug._new_gp_stroke(v3_frame, 3)
    assert v3_stroke is drawing.strokes[-1]
    assert len(v3_stroke.points) == 3
