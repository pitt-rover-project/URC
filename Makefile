# The purpose of this Makefile is to clean up the project directory by removing unnecessary files and directories.

.PHONY: clean

clean:
	rm -rf build/
	find . -name '*.pyc' -delete
	find . -type d -name '__pycache__' -exec rm -rf {} +