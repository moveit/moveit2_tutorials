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
	@echo Step 1 of 4: Building multiversion
	sphinx-multiversion $(OPTS) "$(SOURCE)" build/html
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=galactic/index.html\" /></head></html>" > build/html/index.html
	@echo Step 2 of 4: Clone MoveIt 2 Rolling and build API
	cd build/html/main && if cd moveit2; then git pull; else git clone https://github.com/ros-planning/moveit2 -b main && cd moveit2; fi && \
		sed -i "s/HTML_EXTRA_STYLESHEET  =.*/HTML_EXTRA_STYLESHEET  = ..\/..\/..\/theme.css/g" Doxyfile && DOXYGEN_OUTPUT_DIRECTORY="../api" doxygen && cd .. && rm -rf moveit2
	@echo Step 3 of 4: Clone MoveIt 2 Galactic and build API
	cd build/html/galactic && if cd moveit2; then git pull; else git clone https://github.com/ros-planning/moveit2 -b galactic && cd moveit2; fi && \
		sed -i "s/HTML_EXTRA_STYLESHEET  =.*/HTML_EXTRA_STYLESHEET  = ..\/..\/..\/theme.css/g" Doxyfile && DOXYGEN_OUTPUT_DIRECTORY="../api" doxygen && cd .. && rm -rf moveit2
	@echo Step 4 of 4: Clone MoveIt 2 Foxy and build API
	cd build/html/foxy && if cd moveit2; then git pull; else git clone https://github.com/ros-planning/moveit2 -b foxy && cd moveit2; fi && \
		sed -i "s/HTML_EXTRA_STYLESHEET  =.*/HTML_EXTRA_STYLESHEET  = ..\/..\/..\/theme.css/g" Doxyfile && DOXYGEN_OUTPUT_DIRECTORY="../api" doxygen && cd .. && rm -rf moveit2

local-with-api: Makefile
	@echo Building local with API
	@echo Step 1 of 2: Building multiversion
	make html
	@echo Step 2 of 2: Clone MoveIt 2 and build API using selected distro
	cd build/html && if cd moveit2; then git pull; else git clone https://github.com/ros-planning/moveit2 -b $(MOVEIT_BRANCH) && cd moveit2; fi && \
		sed -i "s/HTML_EXTRA_STYLESHEET  =.*/HTML_EXTRA_STYLESHEET  = ..\/..\/..\/theme.css/g" Doxyfile && DOXYGEN_OUTPUT_DIRECTORY="../api" doxygen &&  cd .. && rm -rf moveit2

.PHONY: help local-with-api Makefile multiversion multiversion-with-api

# By default this is the 'html' build
%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(OUT)" $(OPTS)
