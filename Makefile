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
	@echo Step 1 of 3: Building multiversion
	sphinx-multiversion $(OPTS) "$(SOURCE)" build/html
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=foxy/index.html\" /></head></html>" > build/html/index.html
	@echo Step 2 of 3: Clone MoveIt 2 Rolling API into the website
	# cd build/html/main && if cd api; then git pull && cd ..; else git clone https://github.com/vatanaksoytezer/moveit2 api -b api; fi
	# @echo Step 3 of 3: Clone MoveIt 2 Foxy API into the website TODO Make this actually foxy API
	# pwd && cd build/html/foxy && if cd api; then git pull; else git clone https://github.com/vatanaksoytezer/moveit2 api -b api; fi

local-with-api: Makefile
	@echo Building local with API
	@echo Step 1 of 2: Building multiversion
	make html
	@echo Step 2 of 2: Clone MoveIt 2 Rolling API into the website
	cd build/html && if cd api; then git pull; else git clone https://github.com/vatanaksoytezer/moveit2 api -b api; fi

.PHONY: help local-with-api Makefile multiversion multiversion-with-api

# By default this is the 'html' build
%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(OUT)" $(OPTS)
