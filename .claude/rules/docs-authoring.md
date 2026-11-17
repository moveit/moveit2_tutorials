---
paths:
  - "**/*.rst"
  - "conf.py"
  - "_static/**"
  - "_templates/**"
  - "_scripts/**"
---

# Docs Authoring

The full contributor guide is in `doc/how_to_contribute/` (`how_to_write_tutorials.rst`, `how_to_write_how_to_guides.rst`, `how_to_cross_reference.rst`). This file covers what reviewers check most often.

## Page structure

- Each tutorial teaches one feature or interface. Show first, then tell: put the video or demo near the top and the explanation after it. Match the headings and flow of the neighboring pages in the same section.
- Add every new page to the toctree in its section index (`doc/tutorials/tutorials.rst`, `doc/examples/examples.rst`, `doc/how_to_guides/how_to_guides.rst`, `doc/concepts/concepts.rst`). Files named `_*.rst` are excluded from the build, which suits include-only fragments.
- Pages that are still unported from ROS 1 carry a `:moveit1:` tag at the top. Remove it only when the whole page and its code work on ROS 2.
- When you move or rename a page, add `.. redirect-from::` with the old path (without `.rst`) to the new page, so old links keep working. `conf.py` raises an error if two pages claim the same redirect.

## Links

- Link other pages with `:doc:`, using an absolute path with a leading slash: `` :doc:`/doc/tutorials/getting_started/getting_started` ``.
- Link sections with `:ref:` and the autosectionlabel id, which has no leading slash: `` :ref:`text <doc/tutorials/getting_started/getting_started:Install ROS 2 and Colcon>` ``. Renaming a section heading breaks every `:ref:` to it, so search for the old id before you rename.
- Link C++ API symbols with `` :cpp_api:`moveit::core::RobotModel` ``. The valid symbols are listed in `build/html/api/MoveIt.tag` after a build with the API (`./build_locally.sh` or `make local-with-api`).
- For source on GitHub, use the `extlinks` roles from `conf.py`: `:codedir:` (this repo's `doc/`), `:moveit_codedir:`, `:moveit_resources_codedir:`, `:moveit_msgs_codedir:`. Don't use raw URLs for tutorials, the API, or MoveIt source.
- Every external URL is checked by `htmlproofer.sh` in CI. Verify that a new external link resolves.

## Distro text

Write `{DISTRO}`, `{DISTRO_TITLE}`, `{DISTRO_TITLE_FULL}`, and `{REPOS_FILE_BRANCH}` instead of hardcoding `rolling`, `Rolling`, or `main`. They are plain text replacements applied before parsing, so they also work inside code blocks and URLs. Each branch's build fills them in.

## Code on the page

- Show tutorial source with `.. tutorial-formatter:: ./src/<file>`, and explain the code with comments between `BEGIN_TUTORIAL` and `END_TUTORIAL` in the source file (see `.claude/rules/tutorial-code.md`). Use `.. literalinclude::` for whole files such as launch files. Don't paste copies of repo code into the `.rst`, because the copies drift out of date.
- Shell commands in `code-block:: bash` must run as written, starting from a sourced workspace.

## Images and video

- Put images next to the `.rst` file and include them with `.. image:: name.png` and a `:width:`.
- Do not add animated GIFs, because the files are very large. Record `webm` instead, put it in `_static/videos/`, and embed it with a raw HTML `<video>` tag, as in `doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.rst`.
- Embed YouTube videos with a raw HTML `<iframe>`, as in the existing pages.
