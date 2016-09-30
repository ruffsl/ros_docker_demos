all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build_nvidia          - build nvidia images"
	@echo "   1. make build_intel           - build intel images"
	# @echo "   1. make pull             - pull all images"
	# @echo "   1. make clean            - remove all images"
	@echo ""

build_nvidia:
		@docker build --tag=ros:turtlebot-nvidia nvidia/.

build_intel:
		@docker build --tag=ros:turtlebot-intel intel/.

# pull:
# 	@docker pull ros

# clean:
# 	@docker rmi -f ros
