SHELL := /bin/bash

virtual-env::
	virtualenv -p python3 venv || exit -1
	(source venv/bin/activate || exit -1; pip install -r requirements.txt || exit -1)
run:
	python -m pyliner.new_pyliner
