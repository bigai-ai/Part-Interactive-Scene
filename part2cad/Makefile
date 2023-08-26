.PHONY: install uninstall dev dep

################################################################
# Environment Setup
################################################################
COM_COLOR   = \033[0;34m
OBJ_COLOR   = \033[0;36m
OK_COLOR    = \033[0;32m
ERROR_COLOR = \033[0;31m
WARN_COLOR  = \033[0;33m
NO_COLOR    = \033[m

# Print Warning Info
# print_warning($1: info_string)
# @$(call print_warning, "info")
define print_warning
	printf "%b" "$(OK_COLOR)$(1)$(NO_COLOR)\n"
endef


################################################################
# Installation
################################################################

# install the ncs Package
install:
	@$(call print_warning,"----------Install\ Naive Cutting Simulator----------")
	python setup.py bdist_wheel
	pip install ./dist/*.whl

# uninstall the ncs Package
uninstall:
	@$(call print_warning,"----------Uninstall\ Naive Cutting Simulator----------")
	pip uninstall part2cad
	rm -rf build *.egg-info *.bdist_wheel dist build

# install the ncs Package (development mode)
dev:
	@$(call print_warning,"----------Install\ Naive Cutting Simulator\ [dev]----------")
	pip install -e .