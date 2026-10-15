"""Чтение архива задач `task_inputs` (`tools/native_snapshot_check_corpus.read_tasks`): запись до публикации Density не знает поля `density_backend`.

Старая запись эмулируется честно: состояние слот-датакласса - список значений полей в порядке `fields()`, и пикл, сделанный до добавления последнего поля, короче на него;
`__setstate__` тогда оставляет слот пустым, и `dataclasses.replace` падает с `AttributeError` до всякой геометрии. Адаптер читателя восстанавливает только это поле и только `PYTHON`.
"""

from __future__ import annotations

import copyreg
import dataclasses
import io
import lzma
import pickle
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_snapshot_check_corpus as corpus  # noqa: E402

from cftuv.envelope_domain_pool import DomainTaskV1  # noqa: E402


def _task(**extra):
    return DomainTaskV1(1, 2, "domain", None, None, "0", frozenset(), **extra)


def _archive(path, task, drop_last: int):
    """Файл корпуса с задачей, у которой в состоянии нет последних `drop_last` полей."""

    blob = _pickle_without_last(task, drop_last)
    with lzma.open(path, "wb") as handle:
        pickle.dump(("case", blob), handle, protocol=5)
    return path


def _read_one(path):
    ((case, task),) = list(corpus.read_tasks(path))
    assert case == "case"
    return task


def test_a_record_without_the_field_fails_in_replace_before_the_adapter_and_replays_after_it(tmp_path):
    raw = pickle.loads(_pickle_without_last(_task(), 1))
    with pytest.raises(AttributeError):
        dataclasses.replace(raw, cold=None)  # the reproduction: the replay dies before any geometry
    task = _read_one(_archive(tmp_path / "old.tasks.xz", _task(), 1))
    assert task.density_backend == corpus.HISTORICAL_DENSITY_BACKEND == "PYTHON"
    replayed = dataclasses.replace(task, cold=None)
    assert replayed.density_backend == "PYTHON" and replayed.task_id == 1


def _pickle_without_last(task, drop_last):
    buffer = io.BytesIO()

    class _Pickler(pickle.Pickler):
        def reducer_override(self, obj):
            if type(obj) is DomainTaskV1:
                return copyreg.__newobj__, (DomainTaskV1,), obj.__getstate__()[: len(dataclasses.fields(DomainTaskV1)) - drop_last]
            return NotImplemented

    _Pickler(buffer, protocol=5).dump(task)
    return buffer.getvalue()


@pytest.mark.parametrize("explicit", ["NATIVE", "PYTHON"])
def test_a_record_that_has_the_field_keeps_its_value(tmp_path, explicit):
    task = _read_one(_archive(tmp_path / "new.tasks.xz", _task(density_backend=explicit), 0))
    assert task.density_backend == explicit
    assert dataclasses.replace(task, cold=None).density_backend == explicit


def test_the_product_default_of_a_new_task_is_untouched_by_the_reader():
    assert _task().density_backend == "NATIVE"
    assert corpus.restore_historical_density_backend(_task()).density_backend == "NATIVE"


def test_another_missing_field_is_not_defaulted_silently(tmp_path):
    task = _read_one(_archive(tmp_path / "broken.tasks.xz", _task(), 2))  # `embedding_backend` and `density_backend` are absent
    assert task.density_backend == "PYTHON"
    with pytest.raises(AttributeError, match="embedding_backend"):
        dataclasses.replace(task, cold=None)
