#!/usr/bin/env python

from prototype.srv import *
import rospy
import sys
import time
from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBroker

from optparse import OptionParser

NAO_IP = "10.254.25.108"
Speech = None
node_initiated = False
stop_spinning = False

class SpeechModule(ALModule):
    """ A simple module used to react to a ROS service.

    """
    
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Create a proxy to ALTextToSpeech for later use
        self.tts = ALProxy("ALTextToSpeech")
        self.s = None

    def handle_say_something(self, req):
        global stop_spinning
        print "Saying something"
        self.tts.say(req.script)
        stop_spinning = True
        # self.s.shutdown()
        # rospy.signal_shutdown('testing some shiiiet') 

    def say_something_server(self):
        global node_initiated
        global stop_spinning
        stop_spinning = False
        if node_initiated == False:
            try:
                rospy.init_node('say_something_server')
                node_initiated = True
            except:
                print "Couldn't initiate the node"
        else:
            print "Node already initiated"
        try:
            self.s = rospy.Service('say_something', SaySomething, self.handle_say_something)
        except:
            print "Couldn't start service'"

        print "Ready to be told what to say"
       
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)

    
def main():
    """ Main entry point

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


    global Speech
    Speech = SpeechModule("Speech")
    Speech.say_something_server()
    time.sleep(1)
    Speech.say_something_server()
    print "Exiting"

if __name__ == "__main__":
    main()
  