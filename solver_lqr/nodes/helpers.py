#!/usr/bin/python

'''
@package helper
@author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-07

Use example: ipython ../../../scripts/display.py -- -i ./cfg_demo01_lqr_results.yaml
'''

import numpy as np
import yaml, math, matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec

class DataObject:
    'Initialization'
    def __init__(self):
        self.dt = 0.1
        self.tdim = 1
        self.xdim = 1
        self.udim = 1
        self.states = np.matrix((2,2))
        self.ctrl_ff = np.matrix((2,2))
        self.extravar = np.matrix((2,2))
        
        self.ctrl_fb = list()
        for id in range(0, self.tdim):
            self.ctrl_fb.insert(id, np.matrix((2,2)))

'Base class to show motion'
class Graphics:
    'Initialization'
    def __init__(self):
        self.data = list()
        self.frc_plot = False

    'Function to draw arrows for displaying motion'
    def draw_line(self, axis, x, y, angle, length):
        x_end = x + length * math.cos(angle)
        y_end = y + length * math.sin(angle)
        axis.annotate("", xy=(x, y), xycoords='data',
                          xytext=(x_end, y_end), textcoords='data',
                          arrowprops = dict(arrowstyle="<|-", connectionstyle="arc3", color='cornflowerblue'), 
                     )

    'Function to show the motion'
    def show_motion(self, cfg_file):

        'Read data from file'
        with open(cfg_file, 'r') as stream:
            try:
                self.data = DataObject()
                cfg_pars = yaml.load(stream)
                self.data.dt = cfg_pars['solverlqr_variables']['dt']
                self.data.tdim = cfg_pars['solverlqr_variables']['tdim']
                self.data.xdim = cfg_pars['solverlqr_variables']['xdim']
                self.data.udim = cfg_pars['solverlqr_variables']['udim']
                self.data.name = cfg_pars['solverlqr_variables']['problem_name']
                self.data.states = np.matrix(cfg_pars['solverlqr_variables']['states'])
                self.data.ctrl_ff = np.matrix(cfg_pars['solverlqr_variables']['control_ff'])
                for id in range(0, self.data.tdim):
                    self.data.ctrl_fb.insert(id, np.matrix(cfg_pars['solverlqr_variables']['control_fb_'+str(id)]))
                if (self.data.name == "PointMassForce"):
                    self.data.extravar = np.matrix(cfg_pars['forces'])
                    self.frc_plot = True
                if (self.data.name == "TwoDofArmForce"): 
                    self.data.extravar = np.matrix(cfg_pars['forces'])
                    self.frc_plot = True

            except yaml.YAMLError as exc:
                print(exc)

        fig1 = plt.figure(figsize=(15, 7))
        gs = gridspec.GridSpec(1, 4)

        curobj = self.data
        nominal_plot = plt.subplot(gs[0,0])
        state_plot   = plt.subplot(gs[0,1])
        ctrl_ff_plot = plt.subplot(gs[0,2])
        ctrl_fb_plot = plt.subplot(gs[0,3])
            
        fb_ylim = [-200.0,  200.0]
        fb_xlim = [ 0.0, (curobj.tdim+1)*curobj.dt]
        time = np.linspace(0, curobj.tdim*curobj.dt, curobj.tdim+1)

        nominal_plot.grid()

        if (curobj.name == "TwoDofArmViapoints"):
            'Displaying motion for 2-dof arm with viapoints optimization'
            nm_ylim = [ 0.25, 0.75]
            nm_xlim = [-0.10, 0.40]
            fb_ylim = [-25.0,  2.0]

            nominal_plot.set_title("TwoDofArm")

            q1 = np.squeeze(np.asarray(curobj.states[0,:]))
            q2 = np.squeeze(np.asarray(curobj.states[1,:]))
                
            xvec = 0.30*np.cos(q1)+0.33*np.cos(q1+q2)
            yvec = 0.30*np.sin(q1)+0.33*np.sin(q1+q2)

            nominal_plot.plot(xvec, yvec)

            offset = 0.02
            nominal_plot.text(xvec[0]+offset, yvec[0]+offset, 'Start', color='blue')
            nominal_plot.text(xvec[curobj.tdim]-3.0*offset, yvec[curobj.tdim]-3.0*offset, 'End', color='red')
            nominal_plot.plot(xvec[0], yvec[0], 'b', marker='o', markerfacecolor='blue', markersize=10)
            nominal_plot.plot(xvec[curobj.tdim], yvec[curobj.tdim], 'r', marker='o', markerfacecolor='red', markersize=10)

            nominal_plot.set_xlim(nm_xlim)
            nominal_plot.set_ylim(nm_ylim)

            with open(cfg_file, 'r') as stream:
                try:
                    cfg_pars = yaml.load(stream)
                    nviapoints = cfg_pars['solverlqr_variables']['user_parameters']['nviapoints']
                    for via_id in range(0, nviapoints):
                        viapoint = np.squeeze(np.asarray(cfg_pars['solverlqr_variables']['user_parameters']['via'+str(via_id)][0:-2]))
                        nominal_plot.plot(viapoint[0], viapoint[1], 'g', marker='*', markerfacecolor='green', markersize=10)
                            
                        nominal_plot.text(viapoint[0]+offset, viapoint[1], 'Via', color='green')
                        
                except yaml.YAMLError as exc:
                    print(exc)

        for id in range(0, curobj.xdim):
            state_id_vec = np.squeeze(np.asarray(curobj.states[id,:]))
            state_plot.plot(time, state_id_vec)
        state_plot.set_title('States')
        state_plot.grid()
        state_plot.set_xlim(fb_xlim)

        for id in range(0, curobj.udim):
            ctrl_id_vec = np.squeeze(np.asarray(curobj.ctrl_ff[id,:]))
            ctrl_ff_plot.plot(time[1:], ctrl_id_vec)
        ctrl_ff_plot.set_title('FeedForward Control')
        ctrl_ff_plot.grid()
        ctrl_ff_plot.set_xlim(fb_xlim)
        
        for row in range(0, curobj.udim):
            for col in range(0, curobj.xdim):
                fb_vec = np.zeros((1,curobj.tdim)) 
                for time_id in range(0, curobj.tdim):
                    fb_vec[0,time_id] = curobj.ctrl_fb[time_id][row,col]
                ctrl_fb_plot.plot(time[1:], np.squeeze(np.asarray(fb_vec)))  
        ctrl_fb_plot.set_title('FeedBack Control')
        ctrl_fb_plot.grid()
        ctrl_fb_plot.set_xlim(fb_xlim)
        ctrl_fb_plot.set_ylim(fb_ylim)
        
        fig1.tight_layout()
        
        plt.show()