"""Native work replay: точный ordered след Python бюджетов, без native extension."""

from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
from types import ModuleType

import pytest

ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT/'kernel/src', ROOT/'kernel/tests'):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from cftuv_envelope import backend
from cftuv_envelope.reference import adaptive_density_atlas as atlas, adaptive_density_fan as fan
from cftuv_envelope.reference import density_oracle_v1 as facts
from cftuv_envelope.reference.density_fan_types import DensityRationalAuthorityExhausted
from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch
from test_density_raw_frontend_v1 import _frontend, _forty_five_degree_raw


@pytest.fixture
def shim(monkeypatch):
    package = ModuleType('_density_work_candidate')
    directory = ROOT/'native/cftuv-python/python/cftuv_native'
    package.__path__ = [str(directory)]
    pin = ModuleType(package.__name__+'.pin')
    pin.NativePortUnsupported = type('NativePortUnsupported', (RuntimeError,), {})
    pin.NativePortStale = type('NativePortStale', (RuntimeError,), {})
    pin.OPERATION_FILES = {}
    monkeypatch.setitem(sys.modules,package.__name__,package)
    monkeypatch.setitem(sys.modules,pin.__name__,pin)
    spec = importlib.util.spec_from_file_location(package.__name__+'.density_op',directory/'density_op.py')
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules,spec.name,module)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def batch():
    template = _frontend()
    raw = _forty_five_degree_raw()
    return facts.RawDensityBatchInputV1((raw,replace(raw,spec_id='spec2')),template.canon_ids,
        replace(template.limits,search_cap=1),'NARROW_ROTATION_BAND')


def _observed(monkeypatch):
    budgets = []
    original = atlas.DensityExactWorkBudget.__init__
    def remember(budget, exhausted, cap):
        original(budget,exhausted,cap)
        budgets.append(budget)
    monkeypatch.setattr(atlas.DensityExactWorkBudget,'__init__',remember)
    return budgets


def _assert_articles(budgets,articles):
    assert [(b.cap,b.counters()) for b in budgets] == [(a.cap,a.counters) for a in articles]
    assert all(b.exhausted is DensityRationalAuthorityExhausted for b in budgets)


def _article(phase='SEARCH',cap=1,shell=2,order=0):
    return facts.DensityPriceArticleV1(phase,cap,(('DENSITY_FAN_SHELL_PROBES',shell),('DENSITY_FAN_ORDER_STEPS',order)))


def test_replay_uses_actual_factory_exhausted_types_and_keeps_overcap_trace(shim,monkeypatch):
    articles=[]
    for phase,factory in (('PREPARATION',fan._preparation_budget),('SEARCH',fan._work_budget)):
        budget=factory(1)
        with pytest.raises(DensityRationalAuthorityExhausted):budget.spend_shell_probes(2)
        assert budget.exhausted is DensityRationalAuthorityExhausted
        articles.append(facts.DensityPriceArticleV1(phase,budget.cap,budget.counters()))
    budgets=_observed(monkeypatch)
    shim._replay_density_work((replace(_empty(),ordered_work_articles=tuple(articles)),))
    _assert_articles(budgets,articles)
    assert [b.spent for b in budgets]==[2,2]


def _empty():
    return facts.RawDensitySpecResultV1('INTERNAL',None,(),(),None,None,(),(),(),())


def test_actual_narrow_refusal_then_voronoi_cold_warm_prices_replayed_once(shim,batch,monkeypatch):
    results=evaluate_raw_density_batch(batch)
    assert [len(r.ordered_work_articles) for r in results]==[3,2]
    assert any(sum(value for _,value in a.counters)>a.cap for r in results for a in r.ordered_work_articles)
    budgets=_observed(monkeypatch)
    core_calls=[]
    def core(held,fraction):
        assert held is batch
        core_calls.append(held)
        return results,None
    assert shim.density_compute(core,batch,candidate=True) is results
    articles=tuple(a for r in results for a in r.ordered_work_articles)
    _assert_articles(budgets,articles)
    assert len(core_calls)==1 and len(budgets)==5


def test_named_failure_prefix_replays_overcap_once_and_stops(shim,batch,monkeypatch):
    failed_batch=replace(batch,limits=replace(batch.limits,preparation_cap=0))
    results=evaluate_raw_density_batch(failed_batch)
    assert len(results)==1 and results[0].failure is not None
    assert sum(v for _,v in results[0].ordered_work_articles[0].counters)>0
    budgets=_observed(monkeypatch)
    assert shim.density_compute(lambda *args:(results,None),failed_batch,candidate=True) is results
    _assert_articles(budgets,results[0].ordered_work_articles)


def test_internal_failure_replays_completed_and_partial_prefix_once_then_propagates(shim,batch,monkeypatch):
    first=evaluate_raw_density_batch(batch)[0]
    partial=replace(_empty(),ordered_work_articles=(_article('PREPARATION',0,1),_article('SEARCH',3,0,4)))
    budgets=_observed(monkeypatch)
    with pytest.raises(shim.DensityNativeInternalErrorV1) as caught:
        shim.density_compute(lambda *args:((first,),(1,'UNREGISTERED_OPAQUE',partial)),batch,candidate=True)
    assert caught.value.completed_results==(first,)
    assert caught.value.partial_effects is partial and caught.value.spec_index==1
    _assert_articles(budgets,(*first.ordered_work_articles,*partial.ordered_work_articles))


@pytest.mark.parametrize('bad',[
    replace(_article(),phase='UNKNOWN'),replace(_article(),cap=True),replace(_article(),cap=-1),
    replace(_article(),counters=(('DENSITY_FAN_ORDER_STEPS',0),('DENSITY_FAN_SHELL_PROBES',0))),
    replace(_article(),counters=(('DENSITY_FAN_SHELL_PROBES',),('DENSITY_FAN_ORDER_STEPS',0))),
    replace(_article(),counters=(None,('DENSITY_FAN_ORDER_STEPS',0))),
    replace(_article(),counters=(('DENSITY_FAN_SHELL_PROBES',True),('DENSITY_FAN_ORDER_STEPS',0))),
])
def test_malformed_final_article_is_runtime_error_without_any_partial_replay(shim,monkeypatch,bad):
    budgets=_observed(monkeypatch)
    values=(replace(_empty(),ordered_work_articles=(_article(),)),replace(_empty(),ordered_work_articles=(bad,)))
    with pytest.raises(RuntimeError,match='malformed recorded Density article'):
        shim._replay_density_work(values)
    assert budgets==[]


def test_malformed_batch_report_does_not_replay_even_a_valid_completed_prefix(shim,batch,monkeypatch):
    first=evaluate_raw_density_batch(batch)[0]
    budgets=_observed(monkeypatch)
    with pytest.raises(RuntimeError,match='incomplete'):
        shim.density_compute(lambda *args:((first,),None),batch,candidate=True)
    assert budgets==[]


def test_backend_named_fallback_computes_python_prices_without_native_replay(batch,monkeypatch):
    monkeypatch.setattr(backend,'_native',lambda:(None,'candidate wheel absent'))
    budgets=_observed(monkeypatch)
    with backend.use_backend('PYTHON',density_backend='NATIVE') as ledger:
        results=backend.density_compute(batch)
    _assert_articles(budgets,tuple(a for r in results for a in r.ordered_work_articles))
    assert len(budgets)==5
    assert ledger.record().density_python_calls==1
    assert ledger.record().density_outcomes==('NATIVE_UNAVAILABLE',)


def test_precomputed_attachment_does_not_replay_work_a_second_time(shim,monkeypatch):
    from test_density_raw_v1 import _case_inputs,_selections,CASES
    from cftuv_envelope.reference.density_raw_v1 import extract_raw_density_spec_input
    from cftuv_envelope.reference.density_attach_v1 import attach_density_results_v1
    source,context=_case_inputs(monkeypatch,*CASES[0])
    raw=tuple(sorted((extract_raw_density_spec_input(source,context,spec,selection)
                     for spec,selection in _selections(context)),key=lambda x:x.spec_id))
    template=_frontend()
    actual=facts.RawDensityBatchInputV1(raw,template.canon_ids,template.limits,'NARROW_ROTATION_BAND')
    results=evaluate_raw_density_batch(actual)
    assert all(r.outcome=='OK' for r in results)
    # Вставляем записанную цену: assembler получает факты, не новую математику.
    results=tuple(replace(r,ordered_work_articles=(_article('PREPARATION',5,1),)) for r in results)
    budgets=_observed(monkeypatch)
    decoded=shim.density_compute(lambda *args:(results,None),actual,candidate=True)
    before=len(budgets)
    attached=attach_density_results_v1(context.compilation,context,source,actual.canon_ids,raw,
                                     tuple((x.spec_id,r) for x,r in zip(raw,decoded)))
    assert len(budgets)==before==len(results)
    assert attached.ordered_work_articles==tuple(a for r in results for a in r.ordered_work_articles)
