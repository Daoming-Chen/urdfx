# Implementation Tasks: Restructure Project Layout

This document outlines the step-by-step tasks to restructure the urdfx project for better multi-language support.

## Phase 1: Preparation and Validation

### Task 1.1: Create New Directory Structure
- [ ] Create `core/` directory
- [ ] Create `core/include/`, `core/src/`, `core/tests/` subdirectories
- [ ] Create `bindings/` directory
- [ ] Create `bindings/python/`, `bindings/wasm/` subdirectories
- [ ] Create `examples/cpp/`, `examples/python/`, `examples/javascript/` subdirectories
- [ ] Create `apps/` directory
- [ ] Create `docs/api/`, `docs/guides/`, `docs/tutorials/` subdirectories

**Validation**: Verify all directories exist with `ls` commands.

### Task 1.2: Backup Current Configuration
- [ ] Create a backup branch: `git checkout -b backup-before-restructure`
- [ ] Tag current commit: `git tag pre-restructure`

**Validation**: Confirm branch and tag exist with `git branch` and `git tag`.

### Task 1.3: Update Root CMakeLists.txt (Preparation)
- [ ] Add comments indicating new subdirectory structure
- [ ] Prepare conditional includes for new directories
- [ ] Ensure backward compatibility during transition

**Validation**: `cmake --build build` should still work.

## Phase 2: Core C++ Library Migration

### Task 2.1: Migrate Include Files
- [ ] Run: `git mv include core/include`
- [ ] Update `core/CMakeLists.txt` to reference new include path
- [ ] Update `target_include_directories()` calls

**Validation**: Check that header files are accessible with `#include <urdfx/...>`.

### Task 2.2: Migrate Source Files
- [ ] Run: `git mv src core/src`
- [ ] Update `core/CMakeLists.txt` source file references
- [ ] Ensure relative includes in source files still work

**Validation**: Run `cmake --build build` and verify no compilation errors.

### Task 2.3: Migrate C++ Tests
- [ ] Run: `git mv tests core/tests`
- [ ] Update test CMakeLists.txt paths
- [ ] Update test fixture paths (e.g., URDF file references)

**Validation**: Run `ctest` from `build/` directory, all tests pass.

### Task 2.4: Update Core CMakeLists.txt
- [ ] Create `core/CMakeLists.txt` with proper include/src/tests setup
- [ ] Configure installation rules for headers
- [ ] Export urdfx target for use by bindings

**Validation**: `cmake --install build --prefix /tmp/urdfx-test` succeeds.

### Task 2.5: Verify Core Library Build
- [ ] Clean build directory: `rm -rf build; mkdir build`
- [ ] Configure: `cmake -B build -S .`
- [ ] Build: `cmake --build build`
- [ ] Test: `ctest --test-dir build`

**Validation**: All steps complete without errors.

## Phase 3: Bindings Migration

### Task 3.1: Migrate WASM Bindings
- [ ] Run: `git mv wasm bindings/wasm`
- [ ] Update `bindings/wasm/CMakeLists.txt` to reference `core/`
- [ ] Update include paths in `bindings/wasm/src/bindings.cpp`
- [ ] Update `package.json` scripts if needed

**Validation**: WASM build succeeds: `cd bindings/wasm; emcmake cmake ...; emmake make`.

### Task 3.2: Create Python Bindings Placeholder
- [ ] Create `bindings/python/README.md` with "Coming Soon" notice
- [ ] Create `bindings/python/CMakeLists.txt` stub
- [ ] Create `bindings/python/setup.py` template
- [ ] Add Python binding tasks to main tasks.md

**Validation**: Directory structure matches proposal.

### Task 3.3: Update Root CMakeLists.txt for Bindings
- [ ] Add `add_subdirectory(bindings/wasm)` with appropriate conditions
- [ ] Remove old `add_subdirectory(wasm)` reference
- [ ] Add option flags for Python bindings (for future use)

**Validation**: CMake configuration succeeds.

## Phase 4: Examples and Applications Migration

### Task 4.1: Migrate C++ Examples
- [ ] Run: `git mv examples examples-old`
- [ ] Create new `examples/cpp/` directory
- [ ] Run: `git mv examples-old/*.cpp examples/cpp/`
- [ ] Move `examples-old/CMakeLists.txt` to `examples/cpp/CMakeLists.txt`
- [ ] Update CMakeLists.txt to reference `core/` library
- [ ] Delete `examples-old/` if empty

**Validation**: Build examples: `cmake --build build --target forward_kinematics_example`.

### Task 4.2: Create Python Examples Placeholder
- [ ] Create `examples/python/README.md` with example plan
- [ ] Create `examples/python/forward_kinematics.py` stub
- [ ] Create `examples/python/requirements.txt` (empty or with `urdfx`)

**Validation**: Directory and files exist.

### Task 4.3: Create JavaScript Examples Placeholder
- [ ] Create `examples/javascript/README.md` with example plan
- [ ] Create `examples/javascript/forward_kinematics.js` stub
- [ ] Create `examples/javascript/package.json` template

**Validation**: Directory and files exist.

### Task 4.4: Migrate Visualization App
- [ ] Run: `git mv visualization apps/visualization`
- [ ] Update `apps/visualization/package.json` paths if needed
- [ ] Update any import paths referencing WASM module
- [ ] Update Vite config if it has absolute paths

**Validation**: `cd apps/visualization; npm run dev` works correctly.

### Task 4.5: Update Examples CMakeLists.txt
- [ ] Create `examples/CMakeLists.txt` that includes `cpp/`
- [ ] Update root CMakeLists.txt to `add_subdirectory(examples)`

**Validation**: Examples build successfully.

## Phase 5: Documentation and Cleanup

### Task 5.1: Create Documentation Structure
- [ ] Create `docs/api/cpp/Doxyfile` for C++ API docs
- [ ] Create `docs/api/python/conf.py` for Sphinx (future)
- [ ] Create `docs/guides/getting-started.md`
- [ ] Create `docs/guides/urdf-parsing.md`
- [ ] Create `docs/tutorials/cpp-tutorial.md`

**Validation**: Documentation files exist and are well-structured.

### Task 5.2: Extract Content from README
- [ ] Move detailed C++ API examples to `docs/tutorials/cpp-tutorial.md`
- [ ] Move Python examples to `docs/tutorials/python-tutorial.md`
- [ ] Move build instructions to `docs/guides/getting-started.md`
- [ ] Keep README.md concise with links to docs

**Validation**: README is clear and concise (< 200 lines).

### Task 5.3: Update Path References in Documentation
- [ ] Search for old paths in all markdown files: `rg "include/urdfx" *.md`
- [ ] Replace with new paths: `core/include/urdfx`
- [ ] Update code snippets with correct include paths

**Validation**: All documentation references are correct.

### Task 5.4: Update GitHub Workflows
- [ ] Update `.github/workflows/ci.yml` with new paths
- [ ] Update test commands to reference `core/tests`
- [ ] Update artifact paths for WASM builds (`bindings/wasm/`)

**Validation**: Push to a test branch and verify CI passes.

### Task 5.5: Update Project Metadata Files
- [ ] Update `openspec/project.md` with new directory structure
- [ ] Update `CONTRIBUTING.md` with new file organization guidelines
- [ ] Create `MIGRATION.md` guide for external users
- [ ] Update `.gitignore` if needed

**Validation**: All metadata is consistent.

### Task 5.6: Handle Test Fixtures
- [ ] Decide on `ur5_urdf/` location (propose: `examples/models/`)
- [ ] Run: `git mv ur5_urdf examples/models/ur5`
- [ ] Update all URDF path references in tests
- [ ] Update examples to reference new model path

**Validation**: All tests find URDF files correctly.

### Task 5.7: Clean Up Build Artifacts
- [ ] Update `.gitignore` to reflect new structure
- [ ] Remove `build/` and `build-wasm/` (they are not tracked)
- [ ] Clean any IDE-specific files if needed

**Validation**: `git status` shows no unexpected files.

## Phase 6: Final Verification

### Task 6.1: Full Clean Build
- [ ] Delete all build directories
- [ ] Configure from scratch: `cmake -B build -S . -DCMAKE_BUILD_TYPE=Release`
- [ ] Build: `cmake --build build`
- [ ] Test: `ctest --test-dir build --output-on-failure`

**Validation**: Build succeeds, all tests pass.

### Task 6.2: WASM Build Verification
- [ ] Clean WASM build: `rm -rf build-wasm`
- [ ] Build with Emscripten: `scripts/build-wasm.ps1` (or `.sh`)
- [ ] Check binary size: should be < 2MB
- [ ] Test in `apps/visualization`

**Validation**: WASM module loads and functions correctly.

### Task 6.3: Benchmarks Verification
- [ ] Build benchmarks: `cmake --build build --target ik_benchmarks`
- [ ] Run benchmarks
- [ ] Verify results are comparable to baseline

**Validation**: Performance metrics are within expected range.

### Task 6.4: CI/CD Pipeline Test
- [ ] Push to feature branch: `git push origin feat/restructure-project-layout`
- [ ] Verify all CI checks pass
- [ ] Review test logs for any warnings

**Validation**: Green CI pipeline.

### Task 6.5: Documentation Build Test
- [ ] Generate C++ API docs: `doxygen docs/api/cpp/Doxyfile` (if configured)
- [ ] Verify generated docs are correct
- [ ] Check for broken links

**Validation**: Documentation builds without errors.

### Task 6.6: Git History Verification
- [ ] Test file history tracking: `git log --follow core/src/robot_model.cpp`
- [ ] Ensure original commits are visible
- [ ] Update `.git-blame-ignore-revs` with restructure commit

**Validation**: Git history is preserved.

## Phase 7: Finalization

### Task 7.1: Create Pull Request
- [ ] Write comprehensive PR description
- [ ] Link to this proposal and tasks
- [ ] Add before/after directory tree comparison
- [ ] Request reviews from team

**Validation**: PR is clear and reviewable.

### Task 7.2: Update Release Notes
- [ ] Add entry to CHANGELOG.md
- [ ] Note breaking changes (path changes)
- [ ] Provide migration guide link

**Validation**: Release notes are comprehensive.

### Task 7.3: Merge and Tag
- [ ] Squash or merge PR based on team convention
- [ ] Tag release if appropriate: `git tag v1.0.0-beta.1`
- [ ] Push tags: `git push --tags`

**Validation**: Main branch updated, tag exists.

### Task 7.4: Post-Merge Verification
- [ ] Delete feature branch
- [ ] Verify main branch CI passes
- [ ] Check that documentation site updates (if auto-deployed)

**Validation**: All systems green on main branch.

## Rollback Plan

If critical issues are discovered:

1. **Immediate Rollback**: `git revert <restructure-commit>` or `git reset --hard pre-restructure`
2. **Partial Rollback**: Cherry-pick specific commits to undo problematic changes
3. **Documentation**: Create issue documenting what went wrong and lessons learned

## Dependencies

- All tasks in Phase N must complete before starting Phase N+1
- Task 2.5 (Core Library Build) must pass before any binding migration
- Task 4.4 (Visualization) depends on Task 3.1 (WASM migration)

## Estimated Timeline

- **Phase 1**: 1 hour
- **Phase 2**: 2 hours
- **Phase 3**: 2 hours
- **Phase 4**: 2 hours
- **Phase 5**: 2 hours
- **Phase 6**: 1.5 hours
- **Phase 7**: 0.5 hours

**Total**: ~11 hours (can be split across multiple days)

## Success Metrics

- [ ] All C++ tests pass (100% pass rate)
- [ ] WASM build succeeds and is < 2MB
- [ ] Visualization app runs without errors
- [ ] CI/CD pipeline completes successfully
- [ ] Documentation paths are correct (0 broken links)
- [ ] Git history preserved (verifiable with `--follow`)
- [ ] No regressions in benchmark performance
