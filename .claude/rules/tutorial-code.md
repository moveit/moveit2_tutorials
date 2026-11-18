---
paths:
  - "doc/**/*.cpp"
  - "doc/**/*.hpp"
  - "doc/**/*.h"
  - "doc/**/*.py"
  - "doc/**/*.ipynb"
  - "**/CMakeLists.txt"
  - "package.xml"
  - "*.repos"
  - ".github/upstream.repos"
  - ".github/workflows/**"
---

# Tutorial Code

## Tutorial tags

Most `.rst` pages render their code through `.. tutorial-formatter::` (`_scripts/tutorialformatter.py`), so the source file is also the page text.

- Only the code between `BEGIN_TUTORIAL` and `END_TUTORIAL` appears on the page. `BEGIN_SUB_TUTORIAL`/`CALL_SUB_TUTORIAL` let the page show a block (such as a callback defined above `main`) at a different point than it appears in the file.
- Keep `BEGIN_TUTORIAL`, `END_TUTORIAL`, `BEGIN_SUB_TUTORIAL <name>`, `END_SUB_TUTORIAL`, and `CALL_SUB_TUTORIAL <name>` intact. Every `CALL_SUB_TUTORIAL` needs a matching `BEGIN_SUB_TUTORIAL` with the same name in the same file. Sub-tutorials cannot nest.
- Comment lines inside the tutorial block that start with the marker prefix (the text before `BEGIN_TUTORIAL` on its line, usually `//` or `#`) become reStructuredText prose. Write them as RST: headings underlined with `^`, double-backtick inline code, and roles like `:cpp_api:`.
- Keep setup boilerplate (includes, node and executor setup, shutdown) outside the tutorial block, unless the page is about that boilerplate.
- After you change a tagged file, build the docs (`make html`) and check the rendered page.

## Running the code

- Every demo must run from this package with `ros2 launch moveit2_tutorials ...` or `ros2 run moveit2_tutorials ...`. Python executables are installed with `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})` (see `doc/examples/motion_planning_python_api/CMakeLists.txt`) and run through `ros2 run`, not as `python3 script.py`.
- A new executable needs `add_executable` and `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` in its directory's `CMakeLists.txt`, plus an `add_subdirectory()` in the root `CMakeLists.txt`. Install `launch/` and `config/` to `share/${PROJECT_NAME}`.
- Add new dependencies to the root `package.xml`. If a dependency has no apt binary on some distro in the CI matrix, also add it to both `moveit2_tutorials.repos` and `.github/upstream.repos`.
- Use the Panda (`moveit_resources_panda_moveit_config`) or the Kinova Gen3 config, like the existing tutorials, unless the page needs a different robot.

## C++ style

The tutorials follow the [MoveIt code style guide](https://moveit.ai/documentation/contributing/code/), which is authoritative, and readers copy it. This list covers what reviewers check most often. pre-commit runs clang-format (see `AGENTS.md`).

- `.clang-format` sets `SortIncludes: false`, so do not reorder includes.
- Names: `CamelCase` types, `camelBack` functions and methods, `lower_case` variables, `lower_case_` private members, and `UPPER_CASE` constants.
- Include MoveIt headers in their `.hpp` form (`<moveit/moveit_cpp/moveit_cpp.hpp>`). The `.h` forms are deprecated shims. Some older tutorials still use them, so switch them when you touch the file.
- Logging: the tutorials declare `static const rclcpp::Logger LOGGER = rclcpp::get_logger("<tutorial_name>");` at the top of the file. Follow that pattern in the file you're editing.
- Do not use `auto` for the result of an Eigen expression. It holds a lazy expression template that can dangle. Spell out the type, as in `const Eigen::Vector3d axis = rotation.col(2);`.
- Use a lambda instead of `std::bind`, and `nullptr` instead of `NULL`. Mark overrides with `override`.
- Start a new source file with the BSD 3-clause license block, with your own copyright holder and year.

## Python style

Formatting is black. Use the `moveit_py` and `moveit_configs_utils` APIs that the neighboring tutorials use. Launch files build their parameters with `MoveItConfigsBuilder`.

## Distro compatibility

`main` must build and run on every distro in the CI matrix (listed in `AGENTS.md`). The Humble docs come from the separate `humble` branch, so don't add Humble guards on `main`. These rules follow moveit2's `doc/DISTRO_COMPAT_POLICY.md`, which is authoritative.

- `moveit_package()` builds the package as C++17 on Jazzy and as C++20 on Lyrical and Rolling. Write C++17 that also builds without warnings as C++20. For example, capture variables by name instead of an implicit `this` through `[=]`.
- When a header moved between distros, guard it with `__has_include`, as `doc/tutorials/pick_and_place_with_moveit_task_constructor/src/mtc_node.cpp` does for `tf2_geometry_msgs`. Add a comment that names the oldest distro needing the guard, for example `// Remove when Jazzy goes EOL.`
- In the PR description, link the upstream PR or release that causes the difference.
