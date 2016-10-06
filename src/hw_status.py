#!/usr/bin/env python

import math
import string
import subprocess

import roslib; roslib.load_manifest('mav_fcs')
import rospy


#===============================================================================

def getCpuUsage():
    usage = 0.0
    f = open('/proc/loadavg', 'r')
    line = f.readline()
    tokens = line.split(' ')
    if (len(tokens) > 0):
        usage = string.atof(tokens[0])
    f.close()        
    return math.ceil(usage)
    

def getRamUsage():
    usage = 0.0
    f = open('/proc/meminfo', 'r')
    total = 0
    line = f.readline()
    tokens = line.split(' ')
    size =len(tokens)
    if (size > 2):
        total = string.atoi(tokens[size-2])
        
    free = 0
    line = f.readline()
    tokens = line.split(' ')
    size =len(tokens)
    if (size > 2):
        free = string.atoi(tokens[size-2])

    f.close()
    
    if (total > 0):
        usage = math.ceil( ((total-free)*100.0) / total )        
    return usage
    
    
def getStorageUsage():
    usage = 0
    cmd = ['df', '-h', '--type=ext4']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE)
    out, err = p.communicate()
    lines = out.split('\n')
    #print lines
    if (len(lines)<3):
        return 0.0
    tokens = lines[1].split(' ')
    #print tokens
    size = len(tokens)
    if (size > 2):
        percentage = tokens[size-2]
        percentage = percentage.replace("%", "")
        usage = string.atoi(percentage)
    return usage
    
    
