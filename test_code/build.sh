#!/bin/bash

[ "$1" = lines ] && g++ -o lines ransac_lines.cpp test_ransac.cpp -std=c++11 -pthread
[ "$1" = segments ] && g++ -o segments ransac_lines.cpp test_segments.cpp -std=c++11 -pthread
[ "$1" = localization ] && g++ -o localization ransac_lines.cpp test_localization.cpp -std=c++11 -pthread
[ "$1" = mapping ] && g++ -o mapping ransac_lines.cpp test_mapping.cpp -std=c++11 -pthread
[ "$1" = simulator ] && g++ -o simulator ransac_lines.cpp particle_filter.cpp simulator.cpp -std=c++11 -pthread -O3
