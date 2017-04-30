#!/usr/bin/env python

from prototype.srv import *
import rospy
import sys
import time

stop_spinning = False

class Parent():
    
    def __init__(self):
        self.s = None
    
    def handle_featuretype(self, req):
        global stop_spinning
        global feature_type
        print "Received Feature Type"

        feature_type = req.type
        
        stop_spinning = True
    
    def featuretype_server(self):
        global stop_spinning
       
        stop_spinning = False  
        try:
            rospy.init_node('featuretype_server')
            print "Node Initiated"
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('featuretype', FeatureType, self.handle_featuretype)
        except:
            print "Couldn't start service'"

        print "Waiting for feature type information"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)
        
class Child(Parent, object):
    def __init__(self):
        super(Pride, self).__init__()

test = Child()
test.featuretype_server()