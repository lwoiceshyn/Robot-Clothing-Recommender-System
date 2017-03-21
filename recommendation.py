#!/usr/bin/env python
import sys
import rospy
import time
import os
import traceback
import math

from transitions import Machine

import MySQLdb

from prototype.srv import *

import weka.core.jvm as jvm
import weka.core.classes as classes
from weka.core.dataset import Attribute
from weka.core.dataset import Instances
from weka.core.dataset import Instance
import weka.core.serialization as serialization
from weka.classifiers import Classifier
from weka.core.converters import Loader

stop_spinning = True
acceptance = None
name = None
features = {
            'warmth' : None,
            'casual_formal' : None,
            'comfort' : None,
            'snow' : None,
            'rain' : None,
            'outside_warmth' : None,
            'athletic' : None}

replace = {
            'upper' : None,
            'lower' : None,
            'outer' : None,
            'shoes' : None,
            'accepted' : None}

recommendation = {
                    'upper' : None,
                    'lower' : None,
                    'outer' : None,
                    'shoes' : None}

current_recommendation_rank = {
                                'upper' : 0,
                                'lower' : 0,
                                'outer' : 0,
                                'shoes' : 0}

upper_clothing = [  "Grey Tank Top",
                    "Grey T-Shirt",
                    "Maroon Henley",
                    "Nike Training Shirt",
                    "Blue Button-Down Shirt",
                    "Polo Shirt",
                    "White Dress Shirt",
                    "Suit Jacket",
                    "Navy Blazer",
                    "Grey Hoodie",
                    "Blue Sweater",
                    "Blouse",
                    "Day Dress",
                    "Evening Dress"]

lower_clothing = [  "Beige Cotton Shorts",
                    "Nike Training Shorts",
                    "Black Sweat Pants",
                    "Navy Blue Jeans",
                    "Tan Chinos",
                    "Skirt",
                    "Dress Pants"]

outer_clothing =   ["Black Windbreaker",
                    "Canada Goose Parka",
                    "Yellow Rain Coat"]

shoes_clothing = [  "White Sneakers",
                    "Running Shoes",
                    "Grey Desert Boots",
                    "Black Oxfords"]
upper_indices = None
lower_indices = None
outer_indices = None
shoes_indices = None

class ClothingFSM(object):
    """ Finite state machine implementation for clothing recommendations.

    """

    # State Definitions
    states = ['wait for info',
              'wait for inputs',
              'send recommendation',
              'remove clothing',
              'wait for acceptance',
              'update model']

    # Transitions
    transitions = [
        {'trigger': 'received_user_info', 'source': 'wait for info', 'dest': 'wait for inputs'},
        {'trigger': 'received_inputs', 'source': 'wait for inputs', 'dest': 'send recommendation', 'before': 'generate_recommendation', 'after': 'wait_state'},
        {'trigger': 'wait_state', 'source': 'send recommendation', 'dest': 'wait for acceptance', 'after': 'wait_acceptance'},
        {'trigger': 'user_rejected', 'source': 'wait for acceptance', 'dest': 'send recommendation', 'before': 'update_recommendation', 'after': 'wait_state'},
        {'trigger': 'user_accepted', 'source': 'wait for acceptance', 'dest': 'update model', 'before': 'update_model'},
    ]

    def __init__(self):
        # Initialize the state machine
        self.machine = Machine(model=self, states=ClothingFSM.states, initial='wait for inputs', transitions=ClothingFSM.transitions)
        self.s = None       

    '''
    FSM Methods
    '''
    def wait_acceptance(self):
        self.waitacceptancetwo_server()
        if acceptance == True:
            self.user_accepted()
        elif acceptance == False:
            self.user_rejected()
    
    def wait_replace(self):
        self.waitreplacetwo_server()
    
    def reset_replace(self):
        replace['upper'] = None
        replace['lower'] = None
        replace['outer'] = None
        replace['shoes'] = None

    def update_recommendation(self):
        global recommendation
        self.waitreplacetwo_server()
        if replace['upper'] == 1:
            if current_recommendation_rank['upper'] == 5:
                current_recommendation_rank['upper'] = 1
            else:    
                current_recommendation_rank['upper'] += 1
        if replace['lower'] == 1:
            if current_recommendation_rank['lower'] == 5:
                current_recommendation_rank['lower'] = 1
            else:    
                current_recommendation_rank['lower'] += 1 
        if replace['outer'] == 1:
            if current_recommendation_rank['outer'] == 3:
                current_recommendation_rank['outer'] = 1
            else:    
                current_recommendation_rank['outer'] += 1
        if replace['shoes'] == 1:
            if current_recommendation_rank['shoes'] == 3:
                current_recommendation_rank['shoes'] = 1
            else:    
                current_recommendation_rank['shoes'] += 1
            
        self.reset_replace()
        self.recommend(upper_clothing, lower_clothing, outer_clothing, shoes_clothing, upper_indices, lower_indices, outer_indices, shoes_indices)
        time.sleep(1)
        self.recommendations_server()
        self.recommendationstwo_server()
        
        
        #self.wait_acceptance()
    
    def recommend(self, upper_clothing, lower_clothing, outer_clothing, shoes_clothing, upper_indices, lower_indices, outer_indices, shoes_indices):
        print upper_indices
        print current_recommendation_rank['upper']
        recommendation['upper'] = upper_clothing[upper_indices[current_recommendation_rank['upper']]]
        recommendation['lower'] = lower_clothing[lower_indices[current_recommendation_rank['lower']]]
        recommendation['outer'] = outer_clothing[outer_indices[current_recommendation_rank['outer']]]
        recommendation['shoes'] = shoes_clothing[shoes_indices[current_recommendation_rank['shoes']]]

        print recommendation
    
    def generate_recommendation(self):
        global recommendation
        self.recommend(upper_clothing, lower_clothing, outer_clothing, shoes_clothing, upper_indices, lower_indices, outer_indices, shoes_indices)
        time.sleep(1)
        self.recommendations_server()
        self.recommendationstwo_server()
   
    def update_model(self):
        print "This is the model update step"
        
    '''
    ROS Clients and Services
    '''
    def handle_features(self, req):
        global stop_spinning
        print "Features received, commencing with program"

        features['inside_outside'] = req.inside_outside
        features['outside_warmth'] = req.outside_warmth
        features['athletic'] = req.athletic
        features['comfort'] = req.comfort
        features['snow'] = req.snow
        features['rain'] = req.rain
        features['casual_formal'] = req.casual_formal

        stop_spinning = True


    def features_server(self):
        global stop_spinning
        stop_spinning = False  
        try:
            rospy.init_node('features_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('features', Features, self.handle_features)
        except:
            print "Couldn't start service'"

        print "Waiting for input features"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)

    def handle_recommendations(self, req):
        global stop_spinning
        print "Sending recommendations to Robot"

        upper = recommendation['upper']
        lower = recommendation['lower']
        outer = recommendation['outer']
        shoes = recommendation['shoes']
        stop_spinning = True

        return RecommendationsResponse(upper, lower, outer, shoes)
        

    def recommendations_server(self):
        global stop_spinning
        stop_spinning = False  
        print "Recommendations Server Function Call"
        try:
            rospy.init_node('recommendations_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('recommendations', Recommendations, self.handle_recommendations)
        except:
            print "Couldn't start service'"

        print "Waiting for recommendations request"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)
    
    def handle_recommendationstwo(self, req):
        global stop_spinning
        print "Sending recommendations to Touchscreen"

        upper = recommendation['upper']
        lower = recommendation['lower']
        outer = recommendation['outer']
        shoes = recommendation['shoes']

        stop_spinning = True

        return RecommendationsTwoResponse(upper, lower, outer, shoes)
        

    def recommendationstwo_server(self):
        global stop_spinning
        stop_spinning = False  
        print "Recommendations Server Function Call"
        try:
            rospy.init_node('recommendationstwo_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('recommendationstwo', RecommendationsTwo, self.handle_recommendationstwo)
        except:
            print "Couldn't start service'"

        print "Waiting for recommendations request from Touchscreen"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)

    def handle_feedback(req):
        global stop_spinning
        print "Taking feedback"

        accepted = req.accepted
        replace['upper'] = upper
        replace['lower'] = lower
        replace['outer'] = outer
        replace['shoes'] = shoes
        stop_spinning = True

    
    def feedback_server():
        global stop_spinning
        stop_spinning = False  
        try:
            rospy.init_node('feedback_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('feedback', Feedback, self.handle_feedback)
        except:
            print "Couldn't start service'"

        print "Waiting for recommendations request"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)
    
    def handle_waitacceptancetwo(self,req):
        global stop_spinning, acceptance
        print "Received if user accepted or rejected outfit"
        if req.acceptance == 0:
            acceptance = False
        else:
            acceptance = True
        stop_spinning = True

    def waitacceptancetwo_server(self):
        global stop_spinning
        stop_spinning = False  
        try:
            rospy.init_node('waitacceptancetwo_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('waitacceptancetwo', WaitAcceptanceTwo, self.handle_waitacceptancetwo)
        except:
            print "Couldn't start service'"

        print "Waiting for user to accept/reject recommendation"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)

    def handle_waitreplacetwo(self,req):
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

    def waitreplacetwo_server(self):
        global stop_spinning
        stop_spinning = False  
        try:
            rospy.init_node('waitreplacetwo_server')
        except:
            print "Couldn't initiate the node"
        try:
            self.s = rospy.Service('waitreplacetwo', WaitReplaceTwo, self.handle_waitreplacetwo)
        except:
            print "Couldn't start service'"

        print "Waiting for information on clothing replacements"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)
    
    def handle_username(self,req):
        global stop_spinning, name
        print "Received user's name'"
        name = req.name  
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

        print "Waiting for information on clothing replacements"
        
        while not stop_spinning:
            rospy.rostime.wallsleep(0.5)
    
    def recommendationstwo_client(self, upper, lower, outer, shoes):
        print "Waiting for RecommendationsTwo Service"
        rospy.wait_for_service('recommendationstwo')
        print "Sending Recommendations to Touchscreen"
        try:
            recommendationstwo = rospy.ServiceProxy('recommendationstwo', RecommendationsTwo)
            recommendationstwo(upper, lower, outer, shoes)

        except rospy.ServiceException, e:
            print"Service call failed: %s"%e   

def main():
    global stop_spinning, upper_indices, lower_indices, outer_indices, shoes_indices
    '''
    Classifies clothing using stored models
    '''
    FSM = ClothingFSM()
    #FSM.username_server()
    #FSM.received_user_info()

    #In final program, we will receive this information from database


    #Set to true or false if receiving features vs testing defaults
    receive_features = True

    if receive_features == False:
        #Wait to Receive input 

        #Example inputs from user/weather API
        features['casual_formal'] = 3
        #5 is very comfortable 1 is not comfortable
        features['comfort'] = 3
        #1 is not snowing 2 is light snow 3 is heavy snow
        features['snow'] = 1
        #1 is not raining 3 is raining(no medium)
        features['rain'] = 3
        #If user is spending their time mostly outside, set warmth to outsidewarmth. If not, set warmth
        features['warmth'] = 1
        features['outside_warmth'] = 4
        #1 is no 0 is yes
        features['athletic'] = 1

        snowstring = ''
        rainstring = ''
        athleticstring = ''
    
    else:
        FSM.features_server()
    
    

 
    upper_array = [None] * 14
    lower_array = [None] * 7
    upper_prediction_array = []
    lower_prediction_array = []
    #outer_prediction_array = [None] * 3

    warmth_att = Attribute.create_numeric("Warmth")
    comfort_att = Attribute.create_numeric("Comfort")
    casual_att = Attribute.create_numeric("Casual")
    rain_att = Attribute.create_numeric("Rain")
    snow_att = Attribute.create_numeric("Snow")
    athletic_att = Attribute.create_numeric("Athletic")

    upperlower_attributes = [warmth_att, casual_att, comfort_att, athletic_att]
    outer_attributes = [warmth_att, casual_att, comfort_att, snow_att, rain_att]
    shoes_attributes = [casual_att, comfort_att, athletic_att]

    Instances.create_instances("upper_instances", upperlower_attributes, 0)
    Instances.create_instances("lower_instances", upperlower_attributes, 0)
    Instances.create_instances("outer_instances", outer_attributes, 0)
    Instances.create_instances("shoes_instances", shoes_attributes, 0)

    #Simulate their wardrobe
    #Upper
    upper_array[0] = 1 # Tank Top
    upper_array[1] = 1 # T-Shirt
    upper_array[2] = 1 # Long-Sleeved Shirt
    upper_array[3] = 1 # Athletic Top
    upper_array[4] = 1 # Button-down Shirt
    upper_array[5] = 0 # Polo Shirt
    upper_array[6] = 1 # Dress Shirt
    upper_array[7] = 0 # Suit Jacket
    upper_array[8] = 1 # Blazer
    upper_array[9] = 1 # Hoodie
    upper_array[10] = 1 # Sweater
    upper_array[11] = 0 # Blouse
    upper_array[12] = 0  # Day Dress
    upper_array[13] = 0  # Evening Dress

    #Lower
    lower_array[0] = 1 # Shorts
    lower_array[1] = 1 # Athletic Shorts
    lower_array[2] = 1 # Athletic Pants
    lower_array[3] = 1 # Jeans
    lower_array[4] = 1 # Trousers
    lower_array[5] = 0 # Skirt
    lower_array[6] = 1 # Dress Pants

    upperlower_list = [features['outside_warmth'], features['casual_formal'], features['comfort'], math.fabs(1-features['athletic'])]
    outer_list = [features['outside_warmth'], features['casual_formal'], features['comfort'], features['rain'], features['snow']]
    shoes_list = [features['casual_formal'], features['comfort'], math.fabs(1-features['athletic'])]
    upper_instance = Instance.create_instance(upperlower_list, classname='weka.core.DenseInstance', weight= 1.0)
    lower_instance = Instance.create_instance(upperlower_list, classname='weka.core.DenseInstance', weight= 1.0)
    outer_instance = Instance.create_instance(outer_list, classname='weka.core.DenseInstance', weight= 1.0)
    shoes_instance = Instance.create_instance(shoes_list, classname='weka.core.DenseInstance', weight= 1.0)

    upper_path = '/home/leo/models/uppermodel2.model'
    lower_path = '/home/leo/models/lowermodel2.model'
    outer_path = '/home/leo/models/outermodel.model'
    shoes_path = '/home/leo/models/shoesmodel7.model'

    upper_classifier = Classifier(jobject=serialization.read(upper_path))
    lower_classifier = Classifier(jobject=serialization.read(lower_path))
    outer_classifier = Classifier(jobject=serialization.read(outer_path))
    shoes_classifier = Classifier(jobject=serialization.read(shoes_path))

    upper_predictions = upper_classifier.distribution_for_instance(upper_instance)
    lower_predictions = lower_classifier.distribution_for_instance(lower_instance)
    outer_predictions = outer_classifier.distribution_for_instance(outer_instance)
    shoes_predictions = shoes_classifier.distribution_for_instance(shoes_instance)


    if features['rain'] == 1:
        rainstring = 'No'
    if features['rain'] == 3:
        rainstring = 'Yes'
    if features['snow'] == 1:
        snowstring = 'No'
    if features['snow'] == 3:
        snowstring = 'Yes'
    if features['athletic'] == 1:
        athleticstring = 'No'
    if features['athletic'] == 0:
        athleticstring = 'Yes'

    print "Features being Classified:"
    print "Outside Warmth:", features['outside_warmth'], "Warmth:", features['warmth'], "Casual-Formal:", features['casual_formal'], "Comfort:", features['comfort'], "Athletic:", athleticstring, "Rain:", rainstring, "Snow:", snowstring



    #Remove Clothing Options User Doesn't Own
    for i in range(len(upper_array)):
        if upper_array[i] == 0:
            upper_prediction_array.append(0)
        else:
            upper_prediction_array.append(upper_predictions[i])

    for i in range(len(lower_array)):
        if lower_array[i] == 0:
            lower_prediction_array.append(0)
        else:
            lower_prediction_array.append(lower_predictions[i])

    # for i in range(3):
    #     outer_prediction_array[i] = outer_predictions[i]


    #Find the top 3 options for each classifier
    
    
    max_index_upper1 = 0
    max_index_upper2 = 0
    max_index_upper3 = 0
    max_index_upper4 = 0
    max_index_upper5 = 0


    for i in range(1,len(upper_prediction_array)):
        n = upper_prediction_array[max_index_upper1]
        if upper_prediction_array[i] > n:
            max_index_upper1 = i

    upper_prediction_array[max_index_upper1] = 0

    for i in range(1, len(upper_prediction_array)):
        n = upper_prediction_array[max_index_upper2]
        if upper_prediction_array[i] > n:
            max_index_upper2 = i

    upper_prediction_array[max_index_upper2] = 0

    for i in range(1, len(upper_prediction_array)):
        n = upper_prediction_array[max_index_upper3]
        if upper_prediction_array[i] > n:
            max_index_upper3 = i

    upper_prediction_array[max_index_upper3] = 0
    
    for i in range(1, len(upper_prediction_array)):
        n = upper_prediction_array[max_index_upper4]
        if upper_prediction_array[i] > n:
            max_index_upper4 = i
    
    upper_prediction_array[max_index_upper4] = 0   

    for i in range(1, len(upper_prediction_array)):
        n = upper_prediction_array[max_index_upper5]
        if upper_prediction_array[i] > n:
            max_index_upper5 = i

    upper_indices = [max_index_upper1, max_index_upper2, max_index_upper3, max_index_upper4, max_index_upper5]

    max_index_lower1 = 0
    max_index_lower2 = 0
    max_index_lower3 = 0
    max_index_lower4 = 0
    max_index_lower5 = 0        

    for i in range(1,len(lower_prediction_array)):
        n = lower_prediction_array[max_index_lower1]
        if lower_prediction_array[i] > n:
            max_index_lower1 = i

    lower_prediction_array[max_index_lower1] = 0

    for i in range(1,len(lower_prediction_array)):
        n = lower_prediction_array[max_index_lower2]
        if lower_prediction_array[i] > n:
            max_index_lower2 = i

    lower_prediction_array[max_index_lower2] = 0

    for i in range(1,len(lower_prediction_array)):
        n = lower_prediction_array[max_index_lower3]
        if lower_prediction_array[i] > n:
            max_index_lower3 = i
    
    lower_prediction_array[max_index_lower3] = 0
    
    for i in range(1, len(lower_prediction_array)):
        n = lower_prediction_array[max_index_lower4]
        if lower_prediction_array[i] > n:
            max_index_lower4 = i
    
    lower_prediction_array[max_index_lower4] = 0   

    for i in range(1, len(lower_prediction_array)):
        n = lower_prediction_array[max_index_lower5]
        if lower_prediction_array[i] > n:
            max_index_lower5 = i
    
    lower_indices = [max_index_lower1, max_index_lower2, max_index_lower3, max_index_lower4, max_index_lower5]

    max_index_outer1 = 0
    max_index_outer2 = 0
    max_index_outer3 = 0

    for i in range(1, len(outer_predictions)):
        n = outer_predictions[max_index_outer1]
        if outer_predictions[i] > n:
            max_index_outer1 = i

    outer_predictions[max_index_outer1] = 0

    for i in range(1, len(outer_predictions)):
        n = outer_predictions[max_index_outer2]
        if outer_predictions[i] > n:
            max_index_outer2 = i

    outer_predictions[max_index_outer2] = 0

    for i in range(1, len(outer_predictions)):
        n = outer_predictions[max_index_outer3]
        if outer_predictions[i] > n:
            max_index_outer3 = i

    outer_indices = [max_index_outer1, max_index_outer2, max_index_outer3]

    max_index_shoes1 = 0
    max_index_shoes2 = 0
    max_index_shoes3 = 0

    for i in range(1, len(shoes_predictions)):
        n = shoes_predictions[max_index_shoes1]
        if shoes_predictions[i] > n:
            max_index_shoes1 = i

    shoes_predictions[max_index_shoes1] = 0

    for i in range(1, len(shoes_predictions)):
        n = shoes_predictions[max_index_shoes2]
        if shoes_predictions[i] > n:
            max_index_shoes2 = i

    shoes_predictions[max_index_shoes2] = 0

    for i in range(1, len(shoes_predictions)):
        n = shoes_predictions[max_index_shoes3]
        if shoes_predictions[i] > n:
            max_index_shoes3 = i
    
    shoes_indices = [max_index_shoes1, max_index_shoes2, max_index_shoes3]
    
    print "Upper Indices:", upper_indices
    FSM.received_inputs()
    print "Exiting Program"
    

if __name__ == "__main__":
    try:
        jvm.start()
    except:
        print "Could not start JVM"
    finally:
        main()
        jvm.stop()


