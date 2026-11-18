# AGENTS.md

Instructions for AI coding assistants working in this repository.

This file is the single source of instructions for this repo. `.claude/CLAUDE.md` (`@../AGENTS.md`) and `GEMINI.md` (`@AGENTS.md`) contain only an import of it, because Gemini CLI and older Claude Code versions do not read `AGENTS.md` themselves. `.github/copilot-instructions.md` is a symlink to this file, for the Copilot Chat and IDE code-review surfaces, which read only that path. Edit this file. Do not replace the import files or the symlink with copies.

## What this repo is

`moveit2_tutorials` holds the source of the MoveIt 2 documentation site (https://moveit.picknik.ai/). It is two things at once:

1. **A Sphinx (reStructuredText) website.** `index.rst` sits at the root and the content lives under `doc/` (`tutorials/`, `examples/`, `how_to_guides/`, `concepts/`, `api/`, `how_to_contribute/`).
2. **A single ament_cmake ROS 2 package** named `moveit2_tutorials`. It holds the runnable C++ and Python demo code. Each tutorial's code sits next to its `.rst` file (for example `doc/examples/moveit_cpp/{src,launch,config}/`).

The `main` branch targets ROS 2 Rolling and is published as the Rolling docs. The `humble` branch is published as the Humble docs. The MoveIt source code is in a separate repo (moveit/moveit2).

Where this repo's guidance comes from elsewhere, the other source is authoritative, and this repo keeps only what agents need while editing it:
- MoveIt-wide code and PR conventions: [moveit2's `AGENTS.md`](https://github.com/moveit/moveit2/blob/main/AGENTS.md) and its `.claude/rules/`.
- C++ style: the [MoveIt code style guide](https://moveit.ai/documentation/contributing/code/).
- Writing pages: the contributor guide in `doc/how_to_contribute/`.

## Core principles

- **Tutorial code gets copied.** Readers paste this code into their own robot applications, so it must be correct, safe, and idiomatic. A contribution that wouldn't pass review in moveit2 shouldn't pass review here either. Never leave a demo that looks finished but skips a step or ignores an error.
- **Code and prose must agree.** Most pages render their code straight from the source files (see `tutorial-formatter` below). When you change code, update the prose around it, and the reverse. Only state behavior you verified against the code or the MoveIt API.
- **Keep the scope minimal.** Make the smallest change that solves the problem, and match the structure, tone, and formatting of the neighboring tutorials. When you delete a page or demo, also delete its images, launch files, `CMakeLists.txt` entries, and toctree entry. Add a `.. redirect-from::` for the old path if other sites may link to it.
- **Validate before calling it done.** For code changes, build the package and run the affected demo or test. For docs changes, build the site and fix any new Sphinx warnings or broken links. If something fails, fix it instead of handing the failure back to the user.

## Build, test, lint

### ROS package

The repo is checked out under `src/` of a colcon workspace. Run these from the workspace root after sourcing `/opt/ros/$ROS_DISTRO/setup.bash`:

```bash
vcs import src < src/moveit2_tutorials/moveit2_tutorials.repos
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
# First build: also builds the source dependencies imported from the .repos file
colcon build --packages-up-to moveit2_tutorials --cmake-args -DCMAKE_BUILD_TYPE=Release
# Later rebuilds of only this package
colcon build --packages-select moveit2_tutorials --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# The only integration test: the Quickstart in RViz bringup (gtest + launch_testing, 120 s timeout)
colcon test --packages-select moveit2_tutorials --ctest-args -R bringup_test
colcon test-result --verbose
```

Launch files and configs are installed to `share/moveit2_tutorials`. After sourcing `install/setup.bash`, demos run as `ros2 launch moveit2_tutorials <file>.launch.py` and `ros2 run moveit2_tutorials <executable>`.

CI (`.github/workflows/ci.yaml`) builds with industrial_ci on the `moveit/moveit2:{rolling-source,rolling-release,lyrical-release,jazzy-release}` images, so `main` must build on Jazzy, Lyrical, and Rolling. CI takes its upstream source dependencies from `.github/upstream.repos`, not from `moveit2_tutorials.repos`. When a source dependency changes, check both files.

### Documentation site

```bash
python3 -m venv .venv && source .venv/bin/activate   # needed on Ubuntu 24.04+ (Python >= 3.11)
source /opt/ros/rolling/setup.bash
./build_locally.sh            # installs doxygen, graphviz, requirements.txt; builds with the C++ API; opens a browser
./build_locally.sh noinstall  # skip the dependency install
./build_locally.sh loop       # rebuild on file changes (needs inotifywait)
make html                     # fast Sphinx-only rebuild into build/html, without the Doxygen API
./htmlproofer.sh              # what the CI "Website" job runs: full build plus an html-proofer link check (needs Ruby)
```

`make local-with-api MOVEIT_BRANCH=main` (the variable is required; `build_locally.sh` sets it) shallow-clones moveit/moveit2 into `build/html` and runs Doxygen to produce `build/html/api/MoveIt.tag`, which the `:cpp_api:` role resolves against. Without that step, `:cpp_api:` links do not resolve. `make multiversion` (sphinx-multiversion, used by `.github/workflows/deploy.yml`) builds both `main` and `humble`. Sphinx warnings are not yet errors, but `htmlproofer.sh` fails CI on any 4xx link, so fix or replace an external URL that starts returning 404.

### Formatting

```bash
pre-commit run --all-files    # the Format CI job runs the same hooks
```

The hooks are clang-format (`.clang-format`: Google base style, 120 columns, `SortIncludes: false`) for C/C++, black for Python, codespell (allowlist in `.codespell_words`), and a `misspelled-moveit` check. That check rejects a lowercase "i", a trailing "!", and a "2" suffix with no space. Always write **MoveIt**, or "MoveIt 2" when the version matters.

## Architecture

- **Root `CMakeLists.txt`** calls `moveit_package()` (from `moveit_common`) and `find_package()`s every dependency once. It then `add_subdirectory()`s each tutorial directory. The ROS 1-era examples that are not ported yet are commented out. A tutorial subdirectory's `CMakeLists.txt` only declares its executables and installs them, plus its `launch/` and `config/`. New runtime dependencies go in the root `package.xml`.
- **`conf.py`** configures Sphinx and defines the custom pieces the pages rely on: distro macros, `.. redirect-from::`, `extlinks` roles, doxylink (`:cpp_api:`), and the per-branch overrides for multiversion builds (`smv_rewrite_configs`).
- **`_scripts/tutorialformatter.py`** provides `.. tutorial-formatter:: ./src/file.cpp`, which pulls a source file into a page and renders its `BEGIN_TUTORIAL` comments as prose. These comments *are* the documentation. The tag rules are in `.claude/rules/tutorial-code.md`.

## Rules

Topic rules live in `.claude/rules/`. Claude Code loads each one automatically when you read a file that matches its `paths:` frontmatter. Other assistants should open the file when its trigger applies.

| Trigger | Rule file |
|---|---|
| Writing or changing `.rst` pages, `conf.py`, images, or videos | `.claude/rules/docs-authoring.md`: cross-references, distro macros, toctrees, media, page structure |
| Writing or changing tutorial C++, Python, launch, CMake, or `package.xml` | `.claude/rules/tutorial-code.md`: tutorial tags, C++ style, logging, distro compatibility |

## Branches, PRs, and git

- **Target `main`.** The `humble` branch has diverged (different APIs and robot configs), so do not assume a change applies to it as-is. To backport, add the `backport-humble` label, and Mergify opens the backport PR after merge. Open a backport PR by hand only if Mergify's backport has conflicts.
- **PRs are squash-merged**, so the PR title becomes the commit subject on `main`. Write it as a short imperative sentence ("Fix ...", "Add ..."). A `doc:` or `ci:` prefix is fine.
- **Fill in `.github/PULL_REQUEST_TEMPLATE.md`**: describe the change and link the related issue.
- **CodeRabbit** reviews PRs automatically, as configured in `.coderabbit.yaml`. Each push gets an incremental review. It skips drafts and pauses after 5 reviewed commits. In those cases, post `@coderabbitai review`, or `@coderabbitai full review` to re-review commits it has already seen. Check each finding against the code before applying it, and reply in the thread when you decline one. Do not post `@coderabbitai resolve`, because it closes every thread whether or not the finding was addressed.
- **Tag AI-written GitHub text.** When you publish text to GitHub for the user (PR descriptions, issue bodies, PR and issue comments, review summaries, and inline review comments), make `[written by AI]` the first line of the body, followed by a blank line. This lets maintainers tell human-written text from generated text. Do not add the tag to commit messages, code, or documentation pages.
- Run `pre-commit run --all-files` before committing.
- Before any destructive git command (`reset`, `checkout -- <file>`, `clean`, force push), confirm the exact files and revisions with the user.
