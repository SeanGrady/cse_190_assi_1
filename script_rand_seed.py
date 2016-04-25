#!/usr/bin/env python

import os
seed_list = [100, 3, 1, 0, 9, 12345, 4, 444, 563, 124]
#seed_list = [12345, 4, 444, 563, 124]

for seed in seed_list:
	os.system("sed -i 's/\"seed\":.*/\"seed\": " + str(seed) + "/g' scripts/configuration.json")
	os.system("roslaunch launch/solution_python.launch")
	os.system("mkdir logs/log_seeds3/seed_" + str(seed)) 
	os.system("cp scripts/*.json logs/log_seeds3/seed_" + str(seed) + "/")