# Инструкция владельцу проекта: как управлять AI-agent execution pack

## Ответ в одном абзаце

Пакет нужно **распаковать и хранить в репозитории как версионируемый control plane**. Сам ZIP не должен быть единственным источником инструкций. В каждую новую AI-сессию не нужно вручную переносить весь пакет и всю историю обсуждений. Агенту передаётся только короткий bootstrap prompt, путь к одной активной карточке, принятый integration SHA и handoff её зависимостей. Агент с доступом к репозиторию сам читает разрешённые файлы. Для чата без доступа к репозиторию используется скрипт `tools/build_agent_packet.py`, который собирает один минимальный Markdown-пакет для конкретной карточки.

---

## 1. Куда положить пакет

Рекомендуемый путь внутри CFTUV:

```text
docs/agent_execution/envelope_v1/
```

После установки структура должна выглядеть так:

```text
CFTUV/
├── AGENTS.md
├── docs/
│   ├── architecture_status.json          # создаёт BASE-00
│   ├── agent_execution/
│   │   └── envelope_v1/
│   │       ├── README.md
│   │       ├── OWNER_OPERATING_GUIDE.md
│   │       ├── 00_MASTER_PLAN.md
│   │       ├── 01_GLOBAL_CANON.md
│   │       ├── 02_AGENT_PROTOCOL.md
│   │       ├── 03_DECISION_LOG.md
│   │       ├── SESSION_BOOTSTRAP_TEMPLATE.md
│   │       ├── task_manifest.json
│   │       ├── cards/
│   │       ├── templates/
│   │       ├── references/
│   │       └── tools/
│   └── ...
└── artifacts/
    └── envelope_<slice>/...               # handoff каждого принятого среза
```

Установка из архива:

```bash
unzip CFTUV_AI_AGENT_EXECUTION_PACK_v1_1.zip
mkdir -p docs/agent_execution/envelope_v1
rsync -a CFTUV_AI_AGENT_EXECUTION_PACK_v1_1/ \
  docs/agent_execution/envelope_v1/

git add docs/agent_execution/envelope_v1
git commit -m "docs(envelope): add AI execution control pack v1.1"
```

Не коммитьте только ZIP вместо распакованных файлов. ZIP можно сохранить отдельно как архив поставки, но агенты должны читать обычные Markdown/JSON-файлы из checkout.

---

## 2. На какую ветку установить пакет первый раз

История reviewed Envelope-ветки расходится с `main`, поэтому сначала используйте **временную coordination branch**, основанную непосредственно на reviewed commit:

```bash
git fetch origin --prune
git switch --create coordination/envelope-execution-v1 \
  df587ed166cfb0e0b615148f08c583b4477c5ac4
```

Установите пакет в эту ветку и закоммитьте только документацию execution pack. Затем запустите карточку `BASE-00`.

`BASE-00` отличается от всех следующих карточек:

- `docs/architecture_status.json` ещё не существует;
- стартовой точкой является reviewed commit `df587ed...` или coordination branch, чей родитель — этот commit;
- задача `BASE-00` как раз создаёт canonical integration ref и `docs/architecture_status.json`.

Не вливайте coordination branch в `main` до результата `BASE-00`. После человеческой приёмки BASE-00 execution pack должен присутствовать уже в принятой integration branch.

---

## 3. Что является источником истины

После BASE-00 порядок authority такой:

```text
1. immutable accepted integration SHA
2. docs/architecture_status.json
3. AGENTS.md
4. 01_GLOBAL_CANON.md
5. exact active task card
6. accepted handoff зависимостей карточки
7. implementation/tests в allowlist карточки
```

Не являются authority:

- предыдущий чат;
- память агента;
- весь ZIP как непрочитанный attachment;
- случайная рабочая ветка;
- старый handoff, если более новый gate уже принят;
- downstream-карточки, которые ещё не открыты.

---

## 4. Нужно ли отправлять весь контекст в каждый новый чат

### Агент имеет доступ к GitHub/repository checkout

**Нет.** Передайте только bootstrap prompt и путь к активной карточке. Агент сам прочитает файлы из репозитория.

Минимум, который вы сообщаете:

```text
repository
active card ID and path
accepted integration SHA / branch
accepted dependency handoff paths
PR or worktree, if already created
```

Не прикладывайте все 33 карточки. Это увеличивает риск, что агент начнёт downstream-работу или смешает несколько semantic boundaries.

### Агент не имеет доступа к репозиторию

Для полноценной реализации такой агент всё равно должен получить исходный код. Но для review/contract work можно собрать один минимальный attachment:

```bash
python docs/agent_execution/envelope_v1/tools/build_agent_packet.py \
  --repo-root . \
  --card DOC-00 \
  --mode offline \
  --handoff artifacts/envelope_baseline/session_base_00_handoff.md \
  --output /tmp/DOC-00_AGENT_PACKET.md
```

В новый чат прикладывается **один** файл `DOC-00_AGENT_PACKET.md`, а не весь execution pack.

---

## 5. Жизненный цикл одной карточки

### Шаг 1 — выбрать только READY-карточку

Проверьте `task_manifest.json` и убедитесь, что все `dependencies` имеют принятые handoff.

Не позволяйте агенту самостоятельно выбирать следующую карточку.

### Шаг 2 — определить принятый base

Для всех карточек после BASE-00:

```bash
BASE_SHA=$(python - <<'PY'
import json
print(json.load(open('docs/architecture_status.json'))['accepted_integration_sha'])
PY
)
```

### Шаг 3 — одна карточка, одна ветка

```bash
git switch --create agent/doc-00-authority "$BASE_SHA"
```

Не объединяйте несколько карточек в одну ветку или PR.

### Шаг 4 — открыть свежую AI-сессию

Новая сессия обязательна при смене роли или semantic boundary. Например:

```text
contract author
→ implementation agent
→ independent verifier
```

Это три разных контекста, даже если они относятся к одному большому срезу.

### Шаг 5 — дать агенту bootstrap

Для агента с repo access используйте:

```bash
python docs/agent_execution/envelope_v1/tools/build_agent_packet.py \
  --repo-root . \
  --card DOC-00 \
  --mode repo \
  --handoff artifacts/envelope_baseline/session_base_00_handoff.md
```

Скопируйте полученный короткий prompt в новую сессию.

### Шаг 6 — агент реализует только allowlist карточки

Он не читает весь roadmap и не расширяет scope. Если нужна новая product semantics, агент останавливается с named issue.

### Шаг 7 — обязательный handoff

Агент оставляет:

```text
implementation commit SHA
CI-tested SHA
changed paths
exact test results
contract/schema versions
semantic digest comparison
named unsupported outcomes
source mesh mutation result
legacy paths read: yes/no
next-agent allowlist
```

Handoff живёт в репозитории, а не только в чате.

### Шаг 8 — отдельная verification-сессия

Verifier получает:

```text
active card
accepted base SHA
PR diff
implementation handoff
test/field receipts
```

Verifier не должен получать указание «продолжить реализацию». Его задача — искать нарушение invariants и контрпримеры.

### Шаг 9 — человеческая приёмка

Только после приёмки:

- PR merge/fast-forward в integration branch;
- обновляется `docs/architecture_status.json`;
- карточка получает accepted status;
- фиксируется accepted handoff;
- открывается следующая зависимая карточка.

---

## 6. Короткий bootstrap для repo-connected агента

```text
Work in repository `helmdubo/CFTUV`.

Active task: `<CARD_ID>`
Card path: `docs/agent_execution/envelope_v1/cards/<CARD_FILE>.md`
Accepted base: `<IMMUTABLE_SHA>`
Working branch: `<BRANCH>`
Accepted dependency handoffs:
- `<PATH_1>`
- `<PATH_2>`

Read in this order:
1. `AGENTS.md`
2. `docs/architecture_status.json` (except BASE-00)
3. `docs/agent_execution/envelope_v1/01_GLOBAL_CANON.md`
4. `docs/agent_execution/envelope_v1/02_AGENT_PROTOCOL.md`
5. the exact active card
6. only the dependency handoffs and allowlisted code/tests named by the card

Implement only this card. Do not read legacy decal geometry unless the card explicitly assigns the legacy-curator role. Do not start downstream cards. Leave the mandatory repository handoff and exact test evidence. Stop with a named issue rather than inventing product semantics.
```

Для `BASE-00` замените строку `Accepted base` на:

```text
Reviewed baseline: df587ed166cfb0e0b615148f08c583b4477c5ac4
`docs/architecture_status.json` does not exist yet; creating it is part of this card.
```

---

## 7. Какие файлы вы как владелец обновляете

После каждого принятого gate:

```text
docs/architecture_status.json
    accepted_integration_sha
    active_slice
    last_accepted_gate
    accepted_handoff_paths
    known_blockers

.../task_manifest.json
    status карточки
    status разблокированных зависимых карточек

artifacts/envelope_<slice>/...
    immutable handoff and evidence
```

Не переписывайте принятую карточку задним числом. Если scope нужно изменить после старта, создайте amendment/новую версию pack и запишите причину.

---

## 8. Первая фактическая очередь

```text
1. BASE-00
2. DOC-00 и FIX-00 — отдельные ветки/сессии после BASE-00
3. принять оба handoff
4. D-R2-00 — atomic multiway interaction contract gate
5. после принятого D-R2-00 открыть C-R2D-01
```

`C-R2C-01`…`C-R2C-04` не запускаются: выбранный baseline
`c2622d0...` уже содержит консолидированный принятый C-R2C gate. Карточки
сохранены как superseded historical planning records. Любой новый delta для
этого слоя требует отдельной amendment-карточки.

`D-R2-00` запускается только после принятия `DOC-00` и `FIX-00`, потому что
accepted field receipt уже воспроизводит
`MULTIWAY_INTERACTION_POLICY_UNPROVEN`, а portable fixture и непротиворечивый
onboarding должны быть готовы до нового semantic contract.

---

## 9. Чего не делать

Не следует:

- загружать весь ZIP в каждый новый чат;
- копировать всю историю переписки;
- давать агенту все карточки сразу;
- продолжать один чат через coverage → ownership → runtime boundaries;
- запускать карточку до принятия dependencies;
- позволять агенту выбирать base branch по своему усмотрению;
- использовать `main` как base только потому, что это default branch;
- принимать handoff, существующий только в сообщении чата;
- смешивать implementation и independent verification в одной сессии;
- коммитить ZIP как единственный рабочий формат;
- менять accepted card после выполнения без amendment record.

---

## 10. Минимальный чек-лист владельца перед каждым запуском

```text
[ ] Card status is READY.
[ ] Every dependency handoff is accepted and committed.
[ ] Immutable base SHA is known.
[ ] One clean branch/worktree exists for this card.
[ ] A fresh AI session is opened.
[ ] Bootstrap names exactly one card.
[ ] Agent can read only the required repository context.
[ ] Legacy paths are forbidden unless role explicitly allows them.
[ ] Handoff path and acceptance gate are stated before work starts.
[ ] Independent verification session is planned.
```
