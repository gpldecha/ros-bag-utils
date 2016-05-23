#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import rosbag
import numpy as np
from std_msgs.msg import String
import scipy.io as sio
import os
import glob
import ntpath



def load_FT_net(bag):
    X = np.empty(shape=(0,6))
    x = np.empty(shape=(1,6))
    for msg in bag.read_messages(topics=['/ft_sensor/netft_data']):
        x[0][0] = msg[1].wrench.force.x
        x[0][1] = msg[1].wrench.force.y
        x[0][2] = msg[1].wrench.force.z
        x[0][3] = msg[1].wrench.torque.x
        x[0][4] = msg[1].wrench.torque.y
        x[0][5] = msg[1].wrench.torque.z
        X = np.append(X,x,axis=0)  
    return X
    

def load_ee_pos(bag):
    X = np.empty(shape=(0,7))
    x = np.empty(shape=(1,7))
    for msg in bag.read_messages(topics=['/lwr/ee_pose']):
        x[0][0] = msg[1].position.x
        x[0][1] = msg[1].position.y
        x[0][2] = msg[1].position.z
        x[0][3] = msg[1].orientation.x
        x[0][4] = msg[1].orientation.y
        x[0][5] = msg[1].orientation.z    
        x[0][6] = msg[1].orientation.w
        X = np.append(X,x,axis=0)  
    return X    
    
def load_pfilter(bag):
    
    particle_sets = []    
    for msg in bag.read_messages(topics=['/pfilter']):   
        X = np.zeros((len(msg[1].points), 4));
        i = 0
        #print type(np.array(msg[1].channels[0].values))
        for p in msg[1].points:
            X[i][0] = p.x   
            X[i][1] = p.y            
            X[i][2] = p.z
            i       = i + 1    
        X[:,3] = np.array(msg[1].channels[0].values)  
        particle_sets.append(X)
    return particle_sets                    

def load_belief_features(bag):
    X = np.empty(shape=(0,4))
    x = np.empty(shape=(1,4))

    for msg in bag.read_messages(topics=['/belief_feature_SF']):
        x[0][0] = msg[1].data[0]
        x[0][1] = msg[1].data[1]
        x[0][2] = msg[1].data[2]
        x[0][3] = msg[1].data[3]
        X       = np.append(X,x,axis=0)  
    return X             

def load_ee_sf_pos(bag):
    X = np.empty(shape=(0,3))
    x = np.empty(shape=(1,3))

    for msg in bag.read_messages(topics=['/ee_pos_SF']):
        x[0][0] = msg[1].data[0]
        x[0][1] = msg[1].data[1]
        x[0][2] = msg[1].data[2]
        X       = np.append(X,x,axis=0)  
    return X            
    

def f_extract_bag_bel_pos(full_path,folder_name,file_name):
    bag     = rosbag.Bag(full_path)
    X_pos   = load_ee_sf_pos(bag)     
    X_bel   = load_belief_features(bag)
    print "size: ", X_pos.shape, X_bel.shape
    sio.savemat(folder_name + 'mat/' + file_name + '.mat', {'Pos': X_pos, 'X_bel': X_bel})
  


def load_bag_files(folder_name,f_extract_data_bag):
    files = glob.glob(folder_name + '*.bag')
    for f in files:
       file_name   = ntpath.basename(f).split(".")[-2] 
       print '... Loading: ', file_name
       f_extract_data_bag(f,folder_name,file_name)
    
if __name__ == '__main__':
    print "Loading bag files"
    load_bag_files('/home/guillaume/roscode/bag_record/socket_two/',f_extract_bag_bel_pos)    

