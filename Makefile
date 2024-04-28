all: help

help:
	@echo ""
	@echo "   Help Menu"
	@echo ""
	@echo "   make clone            - clones the subrepostories needed"
	@echo "   make pull             - pull the subrepostories"
	@echo ""

pull:
	git pull origin
	git --work-tree=./src/firmware pull origin
	git --work-tree=./src/simulation pull origin
	git --work-tree=./src/core pull origin

status:
	git status
	git --work-tree=./src/firmware status
	git --work-tree=./src/simulation status
	git --work-tree=./src/core status	
	
src/firmware:
		git clone git@github.com:Roboost-Robotics/firmware.git $@

src/simulation:
		git clone git@github.com:Roboost-Robotics/simulation.git $@
		
src/core:
		git clone git@github.com:Roboost-Robotics/core.git $@

clone:  \
	src/firmware \
	src/simulation \
	src/core
	
