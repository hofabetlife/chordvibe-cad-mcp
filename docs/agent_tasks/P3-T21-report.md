# Отчёт по задаче P3-T21: Актуализация конструктора `TriangleMesh::new` в изолированном спайке `spikes/monstertruck_brep/src/main.rs`

**Дата выполнения:** 15.11.2026  
**Статус:** ✅ DONE  

---

## 1. Цель и проблема

После добавления полей топологического именования граней `face_ids: Vec<u32>` и `faces: Vec<FaceInfo>` в структуру `mcp_cad_core::topology::TriangleMesh` (задача `P2-T6`) изолированный спайк [`spikes/monstertruck_brep/src/main.rs`](../../spikes/monstertruck_brep/src/main.rs) (вынесенный за пределы корневого `[workspace]` через собственный `[workspace]` в [`spikes/monstertruck_brep/Cargo.toml`](../../spikes/monstertruck_brep/Cargo.toml)) перестал компилироваться.

В функции конвертации [`polygon_mesh_to_cad_mesh`](../../spikes/monstertruck_brep/src/main.rs#L46-L63) инициализация `TriangleMesh { vertices, indices }` на строке 60 выполнялась через структурный литерал без новых полей, что приводило к ошибке компиляции:
`error[E0063]: missing fields face_ids and faces in initializer of mcp_cad_core::TriangleMesh`.

---

## 2. Реализация

- В [`spikes/monstertruck_brep/src/main.rs`](../../spikes/monstertruck_brep/src/main.rs#L60) структурный литерал `TriangleMesh { vertices, indices }` заменён на вызов конструктора `TriangleMesh::new(vertices, indices)`, инициализирующего `face_ids: Vec::new()` и `faces: Vec::new()`.
- Дополнительных предупреждений `clippy` в [`spikes/monstertruck_brep/src/main.rs`](../../spikes/monstertruck_brep/src/main.rs) не обнаружено.

---

## 3. Результаты верификации (после финальной правки кода)

| Проверка / Команда | Итог | Детали |
|---|---|---|
| `cargo check --manifest-path spikes/monstertruck_brep/Cargo.toml --all-targets` | **PASS** (`exit 0`) | 0 ошибок компиляции (`36.26s`) |
| `cargo clippy --manifest-path spikes/monstertruck_brep/Cargo.toml --all-targets -- -D warnings` | **PASS** (`exit 0`) | 0 ошибок, 0 предупреждений в `spike-monstertruck-brep` (`23.63s`) |
| `cargo test --manifest-path spikes/monstertruck_brep/Cargo.toml` | **PASS** (`exit 0`) | `0 passed; 0 failed; 0 ignored` (unit-тесты в бинарном спайке не определены, `31.30s`) |
| `cargo run --manifest-path spikes/monstertruck_brep/Cargo.toml` | **PASS** (`exit 0`) | Прошли без паник все 5 секций (`monstertruck-solid`: 20 граней; `Top End-Cap Face` с 2 контурами; `validate_mesh`: `PASS`, `watertight=true`, `open_edges=0`, `non_manifold=0`, `genus=1.0`, ошибка объёма `0.0190%`; 1-edge R=2mm fillet: `OK`, `watertight=true`; neck fillet: `Ok("OK")`; STEP export: `233470 bytes`, `2727 lines`, `ISO-10303-21;`) |
| `cargo clippy --workspace --all-targets -- -D warnings` | **PASS** (`exit 0`) | 0 ошибок, 0 предупреждений во всех 5 крейтах workspace (`0.40s`) |
