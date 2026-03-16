# Docs Overview

В папке `docs/` остаются только актуальные документы.

## Read Order

1. `docs/cftuv_architecture_v2.0.md`
   Главный документ проекта.
   Описывает текущее устройство addon'а, IR, solve pipeline и актуальные runtime rules.

2. `docs/cftuv_refactor_roadmap_for_agents.md`
   Companion document.
   Описывает критические structural problems, безопасный порядок следующих рефакторингов
   и отдельный practical runtime track для frame alignment / closure stabilization.

## Notes

- Если нужно понять проект с нуля, достаточно этих двух документов.
- Если агент продолжает именно текущий runtime bugfix track, он должен читать roadmap не как "вторичный фон", а как active implementation guide.
- Старые handoff-файлы по ранней стабилизации Phase 3 удалены как устаревшие.
- `AGENTS.md` и `CLAUDE.md` продолжают ссылаться на `docs/cftuv_architecture_v2.0.md` как на главный документ.
