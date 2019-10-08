#!/usr/bin/python

'''
@file display.py
@package momentumopt
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-08

Use example: ipython
             run display.py -i <path_to_datafile>
'''

import sys, getopt
from helpers import Graphics

def main(argv):
    inputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print ('display.py -i <path_to_datafile>')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print ('display.py -i <path_to_datafile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg

    motion = Graphics()
    motion.show_motion(inputfile)

if __name__ == "__main__":
   main(sys.argv[1:])
