#! /usr/bin/env python3

'''
@file demo_solo12_jump.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08
'''

from os import path
from momentumopt.kino_dyn_planner_solo import main

if __name__ == "__main__":

    cfg_solo_jump = path.join("..", "config", "cfg_solo12_jump.yaml")
    if(not path.exists(cfg_solo_jump)):
        raise RuntimeError("Please run this script from the demos folder")

    args = ["-i", cfg_solo_jump, "--solo12"]

    main(args)