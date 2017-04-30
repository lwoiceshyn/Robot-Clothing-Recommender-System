#!/usr/bin/env python
from transitions import Machine
import rospy

class ClothingFSM(object):
    """ 
    Finite state machine implementation for clothing recommendations.
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
        '''
        ROS Server which waits for the user to accept or reject a clothing recommendation
        '''
        self.waitacceptancetwo_server()
        if acceptance == True:
            self.user_accepted()
        elif acceptance == False:
            self.user_rejected()
    
    def wait_replace(self):
        '''
        ROS Server which waits for the user to give information on clothing replacements
        '''
        self.waitreplacetwo_server()
    
    def reset_replace(self):
        '''
        Resets selected replacements after they are incorporated
        '''
        replace['upper'] = None
        replace['lower'] = None
        replace['outer'] = None
        replace['shoes'] = None

    def update_recommendation(self):
        '''
        Updates the current top recommended clothing items 
        '''
        self.waitreplacetwo_server()
        if replace['upper'] == 1:
            if current_recommendation_rank['upper'] == 4:
                current_recommendation_rank['upper'] = 0
            else:    
                current_recommendation_rank['upper'] += 1
        if replace['lower'] == 1:
            if current_recommendation_rank['lower'] == 4:
                current_recommendation_rank['lower'] = 0
            else:    
                current_recommendation_rank['lower'] += 1 
        if replace['outer'] == 1:
            if current_recommendation_rank['outer'] == 2:
                current_recommendation_rank['outer'] = 0
            else:    
                current_recommendation_rank['outer'] += 1
        if replace['shoes'] == 1:
            if current_recommendation_rank['shoes'] == 2:
                current_recommendation_rank['shoes'] = 0
            else:    
                current_recommendation_rank['shoes'] += 1
            
        self.reset_replace()
        self.recommend(upper_clothing, lower_clothing, outer_clothing, shoes_clothing, upper_indices, lower_indices, outer_indices, shoes_indices)
        time.sleep(1)
        self.recommendations_server()
        self.recommendationstwo_server()
    
    def recommend(self, upper_clothing, lower_clothing, outer_clothing, shoes_clothing, upper_indices, lower_indices, outer_indices, shoes_indices):
        '''
        Finds the top recommendations for each category
        '''
        global upper_clothing_keys, lower_clothing_keys, outer_clothing_keys, shoes_clothing_keys
        print shoes_indices
        print current_recommendation_rank['shoes']
        
        #Taking care of cases where multiple clothing items in a category exist
        if len(upper_clothing[upper_clothing_keys[upper_indices[current_recommendation_rank['upper']]]]) == 1:
            recommendation['upper'] = upper_clothing[upper_clothing_keys[upper_indices[current_recommendation_rank['upper']]]][0]
        elif len(upper_clothing[upper_clothing_keys[upper_indices[current_recommendation_rank['upper']]]]) > 1:
            index = random.randint(0,len(upper_clothing[upper_clothing_keys[upper_indices[current_recommendation_rank['upper']]]]) - 1)
            recommendation['upper'] = upper_clothing[upper_clothing_keys[upper_indices[current_recommendation_rank['upper']]]][index]

        if len(lower_clothing[lower_clothing_keys[lower_indices[current_recommendation_rank['lower']]]]) == 1:
            recommendation['lower'] = lower_clothing[lower_clothing_keys[lower_indices[current_recommendation_rank['lower']]]][0]
        elif len(lower_clothing[lower_clothing_keys[lower_indices[current_recommendation_rank['lower']]]]) > 1:
            index = random.randint(0,len(lower_clothing[lower_clothing_keys[lower_indices[current_recommendation_rank['lower']]]]) - 1)
            recommendation['lower'] = lower_clothing[lower_clothing_keys[lower_indices[current_recommendation_rank['lower']]]][index]

        if len(outer_clothing[outer_clothing_keys[outer_indices[current_recommendation_rank['outer']]]]) == 1:
            recommendation['outer'] = outer_clothing[outer_clothing_keys[outer_indices[current_recommendation_rank['outer']]]][0]
        elif len(outer_clothing[outer_clothing_keys[outer_indices[current_recommendation_rank['outer']]]]) > 1:
            index = random.randint(0,len(outer_clothing[outer_clothing_keys[outer_indices[current_recommendation_rank['outer']]]]) - 1)
            recommendation['outer'] = outer_clothing[outer_clothing_keys[outer_indices[current_recommendation_rank['outer']]]][index]

        if len(shoes_clothing[shoes_clothing_keys[shoes_indices[current_recommendation_rank['shoes']]]]) == 1:
            recommendation['shoes'] = shoes_clothing[shoes_clothing_keys[shoes_indices[current_recommendation_rank['shoes']]]][0]
        elif len(shoes_clothing[shoes_clothing_keys[shoes_indices[current_recommendation_rank['shoes']]]]) > 1:
            index = random.randint(0,len(shoes_clothing[shoes_clothing_keys[shoes_indices[current_recommendation_rank['shoes']]]]) - 1)
            recommendation['shoes'] = shoes_clothing[shoes_clothing_keys[shoes_indices[current_recommendation_rank['shoes']]]][index]

        print recommendation
    
    def generate_recommendation(self):
        '''
        Sends the recommendations to the touchscreen and runs the ROS servers for recommendations
        '''
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
        '''
        ROS Server response which accepts the input features from the touchscreen
        '''
        global stop_spinning
        print "Features received, commencing with program"

        features['inside_outside'] = req.inside_outside
        if features['inside_outside'] == 1:
            features['outside_warmth'] = req.outside_warmth
        elif features['inside_outside'] == 0:
            features['outside_warmth'] = 2
        features['athletic'] = req.athletic
        features['comfort'] = req.comfort
        features['snow'] = req.snow
        features['rain'] = req.rain
        features['casual_formal'] = req.casual_formal

        stop_spinning = True


    def features_server(self):
        '''
        ROS Server which waits for the input features
        '''
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
        '''
        ROS Server response which sends the recommended clothing to the touchscreen
        '''
        global stop_spinning
        print "Sending recommendations"

        upper = recommendation['upper']
        lower = recommendation['lower']
        outer = recommendation['outer']
        shoes = recommendation['shoes']
        stop_spinning = True

        return RecommendationsResponse(upper, lower, outer, shoes)
        

    def recommendations_server(self):
        '''
        ROS Server which waits for the touchscreen to request the recommendations
        '''
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
        '''
        ROS Server response which sends the recommended clothing to the robot FSM
        '''
        global stop_spinning
        print "Sending recommendations to Touchscreen"

        upper = recommendation['upper']
        lower = recommendation['lower']
        outer = recommendation['outer']
        shoes = recommendation['shoes']

        stop_spinning = True

        return RecommendationsTwoResponse(upper, lower, outer, shoes)
        

    def recommendationstwo_server(self):
        '''
        ROS Server which waits for the robot FSM to request the recommendations
        '''
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
        '''
        ROS Server response which accepts the feedback information from the touchscreen
        '''
        global stop_spinning
        print "Taking feedback"

        accepted = req.accepted
        replace['upper'] = upper
        replace['lower'] = lower
        replace['outer'] = outer
        replace['shoes'] = shoes
        stop_spinning = True

    
    def feedback_server():
        '''
        ROS Server which waits for the feedback for replacements from the touchscreen
        '''
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
        '''
        ROS Server response to the robot FSM requesting acceptance information
        '''
        global stop_spinning, acceptance
        print "Received if user accepted or rejected outfit"
        if req.acceptance == 0:
            acceptance = False
        else:
            acceptance = True
        stop_spinning = True

    def waitacceptancetwo_server(self):
        '''
        ROS Server which waits for the robot FSM to request acceptance information
        '''
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
        '''
        ROS Server response to the robot FSM requesting replacement information
        '''
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
        '''
        ROS Server which waits for the robot FSM to request replacement information
        '''
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
        '''
        ROS Server which responds and saves the user's profile information
        '''
        global stop_spinning, name
        print "Received user's name'"
        name = req.name  
        stop_spinning = True

    def username_server(self):
        '''
        ROS Server which waits for the user's profile information from the touchscreen
        '''
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

