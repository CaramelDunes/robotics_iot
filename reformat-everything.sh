#!/usr/bin/env sh

find . -iname '*.cc' -o -iname '*.h' -o -iname '*.cpp' -o -iname '*.c' | xargs clang-format -i -style=file
