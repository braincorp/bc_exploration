#!/usr/bin/env bash
find ../ -name '*.py' -not -name '__init__.py' -not -name 'conftest.py' | xargs wc -l
