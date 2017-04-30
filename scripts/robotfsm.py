#!/usr/bin/env python
from prototype.srv import *


import rospy
import sys
import time

from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBroker

from behaviors import BehaviorModule, LeiaFSM

from transitions import Machine
from optparse import OptionParser

NAO_IP = "10.254.25.108"
FSM = None
s = None
node_initiated = False
stop_spinning = False
feature_type = None
first_runthrough = True

recommendation = {
                    'upper' : None,
                    'lower' : None,
                    'outer' : None,
                    'shoes' : None 
}

user = {
           'name' : None,
           'activity': None, 
           'acceptance': None 
}

replace = {
                    'all'   : None,
                    'upper' : None,
                    'lower' : None,
                    'outer' : None,
                    'shoes' : None 
}

                    
def main():
    """
     Main function for the robot's finite state machine. Connects to the Nao robot, and runs the appropriate behaviors.

    """
    parser = OptionParser()
    parser.add_option("--pip",
        help="Parent broker port. The IP address or your robot",
        dest="pip")
    parser.add_option("--pport",
        help="Parent broker port. The port NAOqi is listening to",
        dest="pport",
        type="int")
    parser.set_defaults(
        pip=NAO_IP,
        pport=9559)

    (opts, args_) = parser.parse_args()
    pip   = opts.pip
    pport = opts.pport

    myBroker = ALBroker("myBroker",
    "0.0.0.0",   # listen to anyone
    0,           # find a free port and use it
    pip,         # parent broker IP
    pport)       # parent broker port


    global FSM
    FSM = LeiaFSM()
    FSM.wake()
    FSM.username_server()
    FSM.profile_chosen()
    FSM.rest()
    print FSM.state

    print "Exiting"

if __name__ == "__main__":
    main()
  