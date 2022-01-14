# Make file to generate documentation

SOURCE     = .
OUT        = build
BUILD      = python3 -m sphinx
OPTS       =-c .

help:
	@$(BUILD) -M help "$(SOURCE)" "$(OUT)" $(OPTS)
	@echo "  multiversion to build documentation for all branches"

multiversion: Makefile
	sphinx-multiversion $(OPTS) "$(SOURCE)" build/html
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=galactic/index.html\" /></head></html>" > build/html/index.html

multiversion-with-api: Makefile
	@echo Building multiversion with API
	@echo Step 1 of 2: Building multiversion
	sphinx-multiversion $(OPTS) "$(SOURCE)" build/html
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=foxy/index.html\" /></head></html>" > build/html/index.html
	@echo Step 2 of 2: Clone MoveIt 2 API into the website
	cd build/html && git clone https://github.com/ros-planning/moveit2 -b main

.PHONY: help Makefile multiversion multiversion-with-api

# By default this is the 'html' build
%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(OUT)" $(OPTS)
