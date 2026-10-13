# tests/

Host (x86 Linux) unit test suite using [CppUTest](http://cpputest.github.io/).

## Layout

| Folder | Role |
|--------|------|
| [`src/`](src/) | Test files — one per module under test |
| [`mocks/`](mocks/) | CppUTest mocks (call expectations, return-value injection) |
| [`fakes/`](fakes/) | Stub implementations: nRF SDK headers, BSP, fake services |
| `data/` | Fixture data (sample AOP files, configs, packets) |
| `build/` | CMake build output (gitignored) |
| `reports/` | XML test reports |

## Build & run

```bash
cd tests && mkdir -p build && cd build
cmake .. && make -j$(nproc)
cd .. && ln -sf data build/ && ./build/TrackerTests -xg SWSLongRun -v
```

**Run from `tests/`, and keep the `-xg SWSLongRun`.** Two traps, both measured:

- From the repository root instead, five PASPW tests fail because they read
  `tests/data/*.json` by relative path. They are false failures and they send you
  hunting a defect that does not exist.
- A bare `./build/TrackerTests` runs `TEST_GROUP(SWSLongRun)`, fifteen scenarios
  each simulating 75 days of deployment: **over 40 minutes**, against 24 s for the
  other 942 tests. A suite that takes three quarters of an hour stops being run.

To run the simulation deliberately: `./build/TrackerTests -g SWSLongRun`. It is a
research instrument -- it reports detection rates per scenario and asserts one
invariant (clean electrodes with a fixed delay must catch ≥ 90 % of surfacings),
so read its output rather than just its exit code.

Or via the script:
```bash
./scripts/build_unit_tests.sh
```

## Pre-commit hook

`.git/hooks/pre-commit` builds `TrackerTests` on every commit (build only, not run). A failed build aborts the commit.

## Adding a test

1. Add `mything_test.cpp` in [`src/`](src/), using CppUTest's `TEST_GROUP` / `TEST` macros.
2. Add the file to [`CMakeLists.txt`](CMakeLists.txt) `target_sources` for `TrackerTests`.
3. Mock or fake any hardware your code touches (see [`mocks/README.md`](mocks/README.md), [`fakes/README.md`](fakes/README.md)).
