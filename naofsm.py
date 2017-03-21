#!/usr/bin/env python

from prototype.srv import *


import rospy
import sys
import time

from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBroker

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

                    
class BehaviorModule(ALModule):
    """ Module used to encapsulate all of the robot's behaviors and ROS clients/services.

    """
    global feature_type
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Create a proxy to ALTextToSpeech for later use
        self.tts = ALProxy("ALTextToSpeech")
        self.atts = ALProxy("ALAnimatedSpeech")
        self.motion = ALProxy("ALMotion")
        self.tracker = ALProxy("ALTracker")
        self.posture = ALProxy("ALRobotPosture")
        self.leds = ALProxy("ALLeds")
        self.is_breathing = False
        self.s = None
        
        self.create_eye_leds()
    
    def screen_transition_client(self, screen_name):
        rospy.wait_for_service('screen_transition')
        print "Telling Android App to Transition"
        try:
            screen_transition = rospy.ServiceProxy('screen_transition', ScreenTransition)
            screen_transition(screen_name)
        except rospy.ServiceException, e:
            print"Service call failed: %s"%e

    def recommendations_client(self):
        rospy.wait_for_service('recommendations')
        print "Requesting Recommendation from the Clothing Recommender Module"
        try:
            recommendations = rospy.ServiceProxy('recommendations', Recommendations)
            resp = recommendations()
            recommendation['upper'] =  resp.upper
            recommendation['lower'] =  resp.lower
            recommendation['outer'] =  resp.outer
            recommendation['shoes'] =  resp.shoes

        except rospy.ServiceException, e:
            print"Service call failed: %s"%e   

    
    def handle_featuretype(self, req):
        global stop_spinning
        global feature_type
        print "Received Feature Type"

        feature_type = req.type
        user['activity'] = req.activity
        
        stop_spinning = True

    def featuretype_server(self):
        global stop_spinning
        stop_spinning = False  
        try:
            rospy.init_node('featuretype_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('featuretype', FeatureType, self.handle_featuretype)
        except:
            print "Couldn't start service'"

        print "Waiting for feature type information"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)
    
    def handle_waitnext(self, req):
        global stop_spinning
        print "Received confirmation to proceed"
        
        stop_spinning = True
        return WaitNextResponse(0)
        

    def waitnext_server(self):
        global stop_spinning
        stop_spinning = False  
        try:
            rospy.init_node('waitnext_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('waitnext', WaitNext, self.handle_waitnext)
        except:
            print "Couldn't start service'"

        print "Waiting for user to hit the next button"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)
            
    def handle_waitacceptance(self,req):
        global stop_spinning
        print "Received if user accepted or rejected outfit"
        if req.acceptance == 0:
            user['acceptance'] = 0
        else:
            user['acceptance'] = 1
        stop_spinning = True

    def waitacceptance_server(self):
        global stop_spinning
        stop_spinning = False  
        try:
            rospy.init_node('waitacceptance_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('waitacceptance', WaitAcceptance, self.handle_waitacceptance)
        except:
            print "Couldn't start service'"

        print "Waiting for user's acceptance'"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)
    
    def handle_waitreplace(self,req):
        global stop_spinning
        print "Received which outfit components user wants to replace"
        if req.all == 1:
            replace['all'] = 1
            replace['upper'] = 1
            replace['lower'] = 1
            replace['outer'] = 1
            replace['shoes'] = 1  
        else:
            if req.upper == 1:
                replace['upper'] = 1
            if req.lower == 1:
                replace['lower'] = 1
            if req.outer == 1:
                replace['outer'] = 1
            if req.shoes == 1:
                replace['shoes'] = 1           
        stop_spinning = True

    def waitreplace_server(self):
        global stop_spinning
        stop_spinning = False  
        try:
            rospy.init_node('waitreplace_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('waitreplace', WaitReplace, self.handle_waitreplace)
        except:
            print "Couldn't start service'"

        print "Waiting for user to say which clothing to replace"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)
    
    def handle_username(self,req):
        global stop_spinning
        print "Received user's name"
        user['name'] = req.name
        stop_spinning = True

    def username_server(self):
        global stop_spinning
        stop_spinning = False  
        try:
            rospy.init_node('username_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('username', UserName, self.handle_username)
        except:
            print "Couldn't start service'"

        print "Waiting for username."
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)   
    
    
    '''
    Other useful functions
    '''
    def check_featuretype(self):
        if feature_type == 0:
            self.choose_set_features()
        else:
            self.choose_individual_features()
    
    def reset_replace(self):
        replace['all'] = None
        replace['upper'] = None
        replace['lower'] = None
        replace['outer'] = None
        replace['shoes'] = None
            
    '''
    Robot Behaviors.
    '''
    
    def introduction(self):
        self.say_animated("^start(animations/Stand/Gestures/Hey_3) Hey there," + user['name'] + ". ^wait(animations/Stand/Gestures/Hey_3)")
        self.say_animated("^start(animations/Stand/Gestures/Me_2)I'm Leia, your personal wardrobe assistant.^wait(animations/Stand/Gestures/Me_2)")
        self.say_animated("^start(animations/Stand/Gestures/Explain_4) My job is to help you pick out what to wear today. ^wait(animations/Stand/Gestures/Explain_4)")
        self.say_animated("^start(animations/Stand/Gestures/Explain_2) First, I'll need to know what you're doing today. ^wait(animations/Stand/Gestures/Explain_2)")
        self.say_animated("^start(animations/Stand/Gestures/Explain_2) There are many preset activities available. ^wait(animations/Stand/Gestures/Explain_2)")
        self.say_animated("^start(animations/Stand/Gestures/Explain_5) Or, you can choose your own custom settings. ^wait(animations/Stand/Gestures/Explain_5)")
        self.say_post("On the touch screen, let me know what type of activity you're planning.")
        self.beckon()
        self.reset_eyes()
        self.neutral()
        self.featuretype_server()
 

    def individual_features(self):
        global first_runthrough 
        first_runthrough = False
       
        self.breathing_off()
        self.say_animated("^start(animations/Stand/Gestures/Explain_2) Okay, first I need some information about your plans for today and your preferences. ^wait(animations/Stand/Gestures/Explain_2)")
        self.say_animated("^start(animations/Stand/Gestures/HeSays_2) Firstly, I'll need to know if you're getting dressed to do an athletic activity or not. ^wait(animations/Stand/Gestures/HeSays_2)")
        self.say_animated("^start(animations/Stand/Gestures/ShowFloor_1)Just let me know by inputting your answer on the touch screen.^wait(animations/Stand/Gestures/ShowFloor_1)") 
        self.neutral()
        
    def wait_first_feature(self):
        self.waitnext_server()
        self.received_first_feature()

    def comfort(self):
        self.breathing_off()
        self.say_animated("^start(animations/Stand/Gestures/Enthusiastic_2) Okay, now I need to know how important your comfort level is to you today. ^wait(animations/Stand/Gestures/Enthusiastic_2)")
        self.say_contextual("Choose a value on the scale from one to five, where five means comfort is very important to you, and one means comfort isn't a priority.")
        self.neutral()
    
    def wait_second_feature(self):
        self.waitnext_server()
        self.received_second_feature()
    
    def inside_outside(self):
        self.breathing_off()
        self.say_animated("^start(animations/Stand/Gestures/Explain_7) Next, I'll need to know if you're going to be spending your day ^start(animations/Stand/Gestures/Choice_2) mostly inside, or mostly outside. ^wait(animations/Stand/Gestures/Choice_2)")
        self.neutral()
    
    def wait_third_feature(self):
        self.waitnext_server()
        self.received_third_feature()

    def casual_formal(self):
        self.breathing_off()
        self.say_animated("^start(animations/Stand/Gestures/Give_1) Lastly, I need to know what type of dress code you're looking to meet today. ^wait(animations/Stand/Gestures/Give_1)")
        self.say_animated("Your three ^start(animations/Stand/Gestures/CountThree_2) options are  formal,  smart casual, and casual. ^wait(animations/Stand/Gestures/CountThree_2) ")
        self.say_animated("^start(animations/Stand/Gestures/ShowFloor_1) Pick the closest option to your requirements on the touch screen. ^wait(animations/Stand/Gestures/ShowFloor_1)")
        self.neutral() 

    def wait_fourth_feature(self):
        self.waitnext_server()
        self.received_fourth_feature()

    def generate_recommendation(self):
        global first_runthrough
        self.breathing_off()
        if first_runthrough == True:
            self.say_animated("^start(animations/Stand/Gestures/Enthusiastic_1)Great, you chose," + user['activity'] + ".^wait(animations/Stand/Gestures/Enthusiastic_1)" )
            first_runthrough = False
        self.say_animated("^start(animations/Stand/Gestures/Thinking_3)Give me one moment to come up with your recommended outfit. ^wait(animations/Stand/Gestures/Thinking_3)")
        self.recommendations_client()
        time.sleep(1)
        self.say_animated("^start(animations/Stand/Gestures/ShowSky_2) Okay, here is your clothing recommendation for the day. ^wait(animations/Stand/Gestures/ShowSky_2) ^start(animations/Stand/Gestures/This_1) I recommend you wear your" + recommendation['upper'] + ", your" + recommendation['lower'] + ", ^wait(animations/Stand/Gestures/This_1)  ^start(animations/Stand/Gestures/This_3)your" +
                      recommendation['outer'] + ", ^wait(animations/Stand/Gestures/This_3)  ^start(animations/Stand/Gestures/This_4) and your" + recommendation['shoes'] + ". ^wait(animations/Stand/Gestures/This_4)")
        self.neutral()

    def wait_next(self):
        self.breathing_off
        self.say_animated("^start(animations/Stand/Gestures/Explain_2) I'll give you a few moments to look over the outfit recommendation ^wait(animations/Stand/Gestures/Explain_2).")
        self.say_animated("^start(animations/Stand/Gestures/ShowFloor_1) When you're ready, hit the next button on the screen. ^wait(animations/Stand/Gestures/ShowFloor_1)")
        self.waitnext_server()
        self.neutral()
    
    def ask_feedback(self):
        self.breathing_off()
        self.say_animated("^start(animations/Stand/Gestures/Explain_4) Okay, now I need you to let me know if you're happy with the recommended outfit or not. ^wait(animations/Stand/Gestures/Explain_4) ^start(animations/Stand/Gestures/No_2) I won't be offended if you didn't like it. ^wait(animations/Stand/Gestures/No_2)")
        self.say_animated("^start(animations/Stand/Gestures/ShowFloor_1) Use the screen to let me know if you like it or not. ^wait(animations/Stand/Gestures/ShowFloor_1)")
        self.neutral()
        self.waitacceptance_server()
        if user['acceptance'] == 0:
            self.outfit_rejected()
        elif user['acceptance'] == 1:
            self.outfit_accepted()
        
    
    def respond_rejection(self):
        self.breathing_off()
        self.say_animated("^start(animations/Stand/Emotions/Negative/Sad_2) That's too bad. ^wait(animations/Stand/Emotions/Negative/Sad_2)")
        self.say_animated("^start(animations/Stand/Gestures/Determined_1) Let's fix that then. Let me know if you want to replace ^start(animations/Stand/Gestures/Choice_2) the whole outfit, or just part of the outfit. ^wait(animations/Stand/Gestures/Choice_2)")
        self.neutral()
        self.waitreplace_server()
        if replace['all'] == 1:
            self.replace_all()
        else:
            self.replace_part()
        

    def respond_acceptance(self):
        self.breathing_off()
        self.say_animated("^start(animations/Stand/Gestures/Joy_1) Great, I'm glad you like it. ^wait(animations/Stand/Gestures/Joy_1)")
        self.neutral()
    
    
    def respond_replace(self):
        self.breathing_off()
        if replace['all'] == 1:
            self.say_animated("^start(animations/Stand/Gestures/Everything_2) Alright, you chose to replace the whole outfit. ^wait(animations/Stand/Gestures/Everything_2)")
            self.say_animated("^start(animations/Stand/Gestures/Me_2) Let me come up with a new recommended outfit for you. ^wait(animations/Stand/Gestures/Me_2)")
        else:
            self.say_animated("^start(animations/Stand/Gestures/Me_2) Okay, let me come up with new recommendations for the pieces you chose to replace. ^wait(animations/Stand/Gestures/Me_2)")
        self.neutral()
        self.reset_replace()
        self.generate_recommendation()
    

    def closing_remarks(self):
        self.breathing_off()
        self.say_animated("^start(animations/Stand/Gestures/Salute_1) I'm so glad I could be of assistance ^wait(animations/Stand/Gestures/Salute_1). ^start(animations/Stand/Gestures/You_5) Hope you have a great rest of your day. ^wait(animations/Stand/Gestures/You_5)")
        self.say_animated("^start(animations/Stand/Gestures/Hey_1) Bye for now, hope to see you again soon. ^wait(animations/Stand/Gestures/Hey_1)")
        self.reset_eyes()
    

    '''
    Robot Actions
    '''
    def wake(self):
        self.waitnext_server()
        self.motion.wakeUp() 
        self.say_post("Select a user profile to begin.")
        self.beckon()
        

    def rest(self):
        self.motion.rest()

    def say_contextual(self, text):
        self.atts.say(text, "contextual")

    def say_animated(self, text):
        self.atts.say(text)

    def say_post(self, text):
        self.tts.post.say(text)
    
    def neutral(self):
        self.posture.goToPosture("Stand", 0.5)
        self.breathing_on()
    
    def reset_eyes(self):
        self.leds.fadeRGB("LedEye", "white", 0.5)
    
    def create_eye_leds(self):
        name1 = ["FaceLedRight6","FaceLedRight2",
                 "FaceLedLeft6", "FaceLedLeft2"]
        self.leds.createGroup("LedEyeCorner",name1)
        name2 = ["FaceLedRight7","FaceLedRight0","FaceLedRight1",
                 "FaceLedLeft7","FaceLedLeft0","FaceLedLeft1"]
        self.leds.createGroup("LedEyeTop",name2)
        name3 = ["FaceLedRight5","FaceLedRight4","FaceLedRight3",
                 "FaceLedLeft5","FaceLedLeft4","FaceLedLeft3"]
        self.leds.createGroup("LedEyeBottom",name3)
        self.leds.createGroup("LedEyeTopBottom",name2+name3)
        self.leds.createGroup("LedEye",name1+name2+name3)
    

    def breathing_on(self):
        if not self.is_breathing:
            self.motion.setBreathEnabled('Body', True)
            print "Started Breathing"
            self.is_breathing = True
            self.facetrack_on()
    
    def breathing_off(self):
        self.facetrack_off()
        if self.is_breathing:
            self.motion.setBreathEnabled('Body', False)
            print "Stopped Breathing"
            self.is_breathing = False
           
    def facetrack_on(self):
        target_name = "Face"
        face_width = 0.1
        self.tracker.registerTarget(target_name, face_width)
        self.tracker.track(target_name)
    
    def facetrack_off(self):
        self.tracker.stopTracker()
        self.tracker.unregisterAllTargets()
    
    
    def beckon(self):


        # Choregraphe bezier export in Python.

        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([2.68, 4.52])
        keys.append([[-0.0782759, [3, -0.893333, 0], [3, 0.613333, 0]], [-0.144238, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("HeadYaw")
        times.append([2.68, 4.52])
        keys.append([[0.477032, [3, -0.893333, 0], [3, 0.613333, 0]], [0.0106959, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LAnklePitch")
        times.append([2.68, 4.52])
        keys.append([[0.0889301, [3, -0.893333, 0], [3, 0.613333, 0]], [0.0889301, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LAnkleRoll")
        times.append([2.68, 4.52])
        keys.append([[-0.125746, [3, -0.893333, 0], [3, 0.613333, 0]], [-0.125746, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LElbowRoll")
        times.append([2.68, 4.52])
        keys.append([[-0.328234, [3, -0.893333, 0], [3, 0.613333, 0]], [-0.391128, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([2.68, 4.52])
        keys.append([[-1.89146, [3, -0.893333, 0], [3, 0.613333, 0]], [-1.21037, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([2.68, 4.52])
        keys.append([[0.4008, [3, -0.893333, 0], [3, 0.613333, 0]], [0.3108, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LHipPitch")
        times.append([2.68, 4.52])
        keys.append([[0.1335, [3, -0.893333, 0], [3, 0.613333, 0]], [0.1335, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LHipRoll")
        times.append([2.68, 4.52])
        keys.append([[0.096684, [3, -0.893333, 0], [3, 0.613333, 0]], [0.096684, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LHipYawPitch")
        times.append([2.68, 4.52])
        keys.append([[-0.176368, [3, -0.893333, 0], [3, 0.613333, 0]], [-0.176368, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LKneePitch")
        times.append([2.68, 4.52])
        keys.append([[-0.0844119, [3, -0.893333, 0], [3, 0.613333, 0]], [-0.0844119, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([2.68, 4.52])
        keys.append([[1.00626, [3, -0.893333, 0], [3, 0.613333, 0]], [1.4772, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([2.68, 4.52])
        keys.append([[0.504644, [3, -0.893333, 0], [3, 0.613333, 0]], [0.1733, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([2.68, 4.52])
        keys.append([[-1.82387, [3, -0.893333, 0], [3, 0.613333, 0]], [0.0889301, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RAnklePitch")
        times.append([2.68, 4.52])
        keys.append([[0.093616, [3, -0.893333, 0], [3, 0.613333, 0]], [0.093616, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RAnkleRoll")
        times.append([2.68, 4.52])
        keys.append([[0.135034, [3, -0.893333, 0], [3, 0.613333, 0]], [0.135034, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([2.68, 4.52])
        keys.append([[0.786984, [3, -0.893333, 0], [3, 0.613333, 0]], [0.397348, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([2.68, 4.52])
        keys.append([[0.484702, [3, -0.893333, 0], [3, 0.613333, 0]], [1.18114, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([2.68, 4.52])
        keys.append([[0.452, [3, -0.893333, 0], [3, 0.613333, 0]], [0.3092, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RHipPitch")
        times.append([2.68, 4.52])
        keys.append([[0.131882, [3, -0.893333, 0], [3, 0.613333, 0]], [0.131882, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RHipRoll")
        times.append([2.68, 4.52])
        keys.append([[-0.099668, [3, -0.893333, 0], [3, 0.613333, 0]], [-0.099668, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RHipYawPitch")
        times.append([2.68, 4.52])
        keys.append([[-0.176368, [3, -0.893333, 0], [3, 0.613333, 0]], [-0.176368, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RKneePitch")
        times.append([2.68, 4.52])
        keys.append([[-0.0923279, [3, -0.893333, 0], [3, 0.613333, 0]], [-0.0923279, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([2.68, 4.52])
        keys.append([[1.19043, [3, -0.893333, 0], [3, 0.613333, 0]], [1.47115, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([2.68, 4.52])
        keys.append([[0.199378, [3, -0.893333, 0], [3, 0.613333, 0]], [-0.15651, [3, -0.613333, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([2.68, 4.52])
        keys.append([[1.31766, [3, -0.893333, 0], [3, 0.613333, 0]], [0.115008, [3, -0.613333, 0], [3, 0, 0]]])

        try:
            self.motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err

    

class LeiaFSM(BehaviorModule, object):
    """ Finite state machine implementation for robot behaviors.

    """

    # State Definitions
    states = ['waiting',
              'intro',
              'give recommendation',
              'first feature',
              'second feature',
              'third feature',
              'fourth feature',
              'wait',
              'ask feedback',
              'outro']
    # Transitions
    transitions = [
        {'trigger': 'profile_chosen', 'source': 'waiting', 'dest': 'intro', 'before': 'introduction', 'after': 'check_featuretype'},
        {'trigger': 'choose_set_features', 'source': 'intro', 'dest': 'give recommendation', 'before': 'generate_recommendation', 'after': 'wait_state'},
        {'trigger': 'wait_state', 'source': 'give recommendation', 'dest': 'wait', 'before': 'wait_next', 'after': 'ask_feedback'},
        {'trigger': 'choose_individual_features', 'source': 'intro', 'dest': 'first feature', 'before': 'individual_features', 'after':'wait_first_feature'},
        {'trigger': 'received_first_feature', 'source': 'first feature', 'dest': 'second feature', 'before':'inside_outside', 'after':'wait_second_feature'},
        {'trigger': 'received_second_feature', 'source': 'second feature', 'dest': 'third feature', 'before':'comfort', 'after':'wait_third_feature'},
        {'trigger': 'received_third_feature', 'source': 'third feature', 'dest': 'fourth feature', 'before': 'casual_formal', 'after': 'wait_fourth_feature'},
        {'trigger': 'received_fourth_feature', 'source': 'fourth feature', 'dest': 'give recommendation', 'before': 'generate_recommendation', 'after': 'wait_state'},
        {'trigger': 'outfit_rejected', 'source': 'wait', 'dest': 'ask feedback', 'after':'respond_rejection' },
        {'trigger': 'outfit_accepted', 'source': 'wait', 'dest': 'outro', 'before': 'respond_acceptance', 'after':'closing_remarks'},
        {'trigger': 'replace_all', 'source': 'ask feedback', 'dest': 'give recommendation', 'before':'respond_replace', 'after':'wait_state'},
        {'trigger': 'replace_part', 'source': 'ask feedback', 'dest': 'give recommendation', 'before':'respond_replace', 'after':'wait_state'},
        {'trigger': 'recommend_again', 'source': 'replace part', 'dest': 'give recommendation', 'before':'request_recommendation'},
        {'trigger': 'recommend_again', 'source': 'replace all', 'dest': 'give recommendation', 'before': 'request_recommendation'},
    ]

    def __init__(self):
        #Inherit methods from Parent Class
        super(LeiaFSM, self).__init__("Module")
        # Initialize the state machine
        self.machine = Machine(model=self, states=LeiaFSM.states, initial='waiting', transitions=LeiaFSM.transitions)
        


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


    global FSM
    FSM = LeiaFSM()
    FSM.wake()
    FSM.username_server()
    FSM.profile_chosen()
    FSM.rest()
    print FSM.state
    #FSM.recommendations_client()
    #print recommendation

    print "Exiting"

if __name__ == "__main__":
    main()
  