#!/usr/bin/env python
import sys
import rospy
import os
import traceback
import math
import time
import random

import MySQLdb

from transitions import Machine

from prototype.srv import *
from clothing import ClothingFSM

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
            'inside_outside' : None,
            'casual_formal' : None,
            'comfort' : None,
            'snow' : None,
            'rain' : None,
            'outside_warmth' : None,
            'inside_warmth' : 2,
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



upper_clothing = {  "Tank Top": [],
                    "T-Shirt": [],
                    "Long-sleeved Shirt": [],
                    "Athletic Top": [],
                    "Button-down Shirt": [],
                    "Polo Shirt": [],
                    "Dress Shirt": [],
                    "Suit Jacket": [],
                    "Blazer": [],
                    "Hoodie": [],
                    "Sweater": [],
                    "Blouse": [],
                    "Day Dress": [],
                    "Evening Dress": []}

lower_clothing = {  "Shorts": [],
                    "Athletic Shorts": [],
                    "Athletic Pants": [],
                    "Jeans": [],
                    "Trousers": [],
                    "Skirt": [],
                    "Dress Pants": []}

outer_clothing =   {"Light Jacket": [],
                    "Winter Jacket": [],
                    "Rain Jacket": []}

shoes_clothing = {  "Casual Shoes": [],
                    "Athletic Shoes": [],
                    "Dress Shoes": [],
                    "Business Casual Shoes": []}

upper_clothing_keys = [  "Tank Top",
                    "T-Shirt",
                    "Long-sleeved Shirt",
                    "Athletic Top",
                    "Button-down Shirt",
                    "Polo Shirt",
                    "Dress Shirt",
                    "Suit Jacket",
                    "Blazer",
                    "Hoodie",
                    "Sweater",
                    "Blouse",
                    "Day Dress",
                    "Evening Dress"]

lower_clothing_keys = [  "Shorts",
                    "Athletic Shorts",
                    "Athletic Pants",
                    "Jeans",
                    "Trousers",
                    "Skirt",
                    "Dress Pants"]

outer_clothing_keys =   ["Light Jacket",
                    "Winter Jacket",
                    "Rain Jacket"]

shoes_clothing_keys = [  "Casual Shoes",
                    "Athletic Shoes",
                    "Dress Shoes",
                    "Business Casual Shoes"]

upper_indices = None
lower_indices = None
outer_indices = None
shoes_indices = None

random.seed()




def main():
    global stop_spinning, name, upper_clothing, lower_clothing, outer_clothing, shoes_clothing, upper_indices, lower_indices, outer_indices, shoes_indices
    '''
    Classifies clothing using stored classification models for each user
    '''
    FSM = ClothingFSM()
    #FSM.username_server()

    clothingdb = MySQLdb.connect(host="localhost",
                                 user="root",
                                 passwd="lolcats123", # Change to your SQL DB password
                                 db = "userprofiles")
    cursor = clothingdb.cursor()

    cursor.execute("SELECT * FROM clothing")

    name = "Study"

    #Populate clothing dictionaries with user's wardrobe
    for row in cursor.fetchall():
        print str(row[2])
        print str(row[6])
        if str(row[0]) == name:
            if str(row[1]) == "Upper Body":
                try:
                    upper_clothing[row[2]].append(row[6])
                except:
                    print "Problem appending clothing to dictionary"
            if str(row[1]) == "Lower Body":
                try:
                    lower_clothing[row[3]].append(row[6])
                except:
                    print "Problem appending clothing to dictionary"
            if str(row[1]) == "Outerwear":
                try:
                    outer_clothing[row[4]].append(row[6])
                except:
                    print "Problem appending clothing to dictionary"
            if str(row[1]) == "Shoes":
                try:
                    shoes_clothing[row[5]].append(row[6])
                except:
                    print "Problem appending clothing to dictionary"
    
    print upper_clothing, lower_clothing, outer_clothing, shoes_clothing
    # FSM.received_user_info()

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
    outer_array = [None] * 3
    shoes_array = [None] * 4
    upper_prediction_array = []
    lower_prediction_array = []
    outer_prediction_array = []
    shoes_prediction_array = []

    warmth_att = Attribute.create_numeric("Warmth")
    comfort_att = Attribute.create_numeric("Comfort")
    casual_att = Attribute.create_numeric("Casual")
    rain_att = Attribute.create_numeric("Rain")
    snow_att = Attribute.create_numeric("Snow")
    athletic_att = Attribute.create_numeric("Athletic")

    
    upper_attributes = [warmth_att, casual_att, comfort_att, athletic_att]
    lower_attributes = [warmth_att, casual_att, comfort_att, athletic_att]
    outer_attributes = [warmth_att, casual_att, comfort_att, snow_att, rain_att]
    shoes_attributes = [casual_att, comfort_att, athletic_att]

    Instances.create_instances("upper_instances", upper_attributes, 0)
    Instances.create_instances("lower_instances", lower_attributes, 0)
    Instances.create_instances("outer_instances", outer_attributes, 0)
    Instances.create_instances("shoes_instances", shoes_attributes, 0)

    #Simulate their wardrobe
    #Upper
    # Tank Top
    if len(upper_clothing['Tank Top']) == 0:
        upper_array[0] = 0 
    else:
        upper_array[0] = 1 
    # T-Shirt
    if len(upper_clothing['T-Shirt']) == 0:
        upper_array[1] = 0 
    else:
        upper_array[1] = 1 
    # Long-Sleeved Shirt
    if len(upper_clothing['Long-sleeved Shirt']) == 0:
        upper_array[2] = 0 
    else:
        upper_array[2] = 1 
    # Athletic Top
    if len(upper_clothing['Athletic Top']) == 0:
        upper_array[3] = 0 
    else:
        upper_array[3] = 1     
    # Button-down Shirt
    if len(upper_clothing['Button-down Shirt']) == 0:
        upper_array[4] = 0 
    else:
        upper_array[4] = 1     
    # Polo Shirt
    if len(upper_clothing['Polo Shirt']) == 0:
        upper_array[5] = 0 
    else:
        upper_array[5] = 1  
    # Dress Shirt
    if len(upper_clothing['Dress Shirt']) == 0:
        upper_array[6] = 0 
    else:
        upper_array[6] = 1  
    # Suit Jacket
    if len(upper_clothing['Suit Jacket']) == 0:
        upper_array[7] = 0 
    else:
        upper_array[7] = 1  
    # Blazer
    if len(upper_clothing['Blazer']) == 0:
        upper_array[8] = 0 
    else:
        upper_array[8] = 1  
    # Hoodie
    if len(upper_clothing['Hoodie']) == 0:
        upper_array[9] = 0 
    else:
        upper_array[9] = 1  
    # Sweater
    if len(upper_clothing['Sweater']) == 0:
        upper_array[10] = 0 
    else:
        upper_array[10] = 1  
    # Blouse
    if len(upper_clothing['Blouse']) == 0:
        upper_array[11] = 0 
    else:
        upper_array[11] = 1

    # Day Dress
    if len(upper_clothing['Day Dress']) == 0:
        upper_array[12] = 0 
    else:
        upper_array[12] = 1
    # Evening Dress
    if len(upper_clothing['Evening Dress']) == 0:
        upper_array[13] = 0 
    else:
        upper_array[13] = 1

    #Lower

    # Regular Shorts
    if len(lower_clothing['Shorts']) == 0:
        lower_array[0] = 0 
    else:
        lower_array[0] = 1
    # Athletic Shorts
    if len(lower_clothing['Athletic Shorts']) == 0:
        lower_array[1] = 0 
    else:
        lower_array[1] = 1
    # Athletic Pants
    if len(lower_clothing['Athletic Pants']) == 0:
        lower_array[2] = 0 
    else:
        lower_array[2] = 1
    # Jeans
    if len(lower_clothing['Jeans']) == 0:
        lower_array[3] = 0 
    else:
        lower_array[3] = 1
    # Trousers
    if len(lower_clothing['Trousers']) == 0:
        lower_array[4] = 0 
    else:
        lower_array[4] = 1
    # Skirt
    if len(lower_clothing['Skirt']) == 0:
        lower_array[5] = 0 
    else:
        lower_array[5] = 1
    # Dress Pants
    if len(lower_clothing['Dress Pants']) == 0:
        lower_array[6] = 0 
    else:
        lower_array[6] = 1

    #Outer
    # Light Jacket
    if len(outer_clothing['Light Jacket']) == 0:
        outer_array[0] = 0 
    else:
        outer_array[0] = 1
    # Heavy Jacket
    if len(outer_clothing['Winter Jacket']) == 0:
        outer_array[1] = 0 
    else:
        outer_array[1] = 1
    # Rain Jacket
    if len(outer_clothing['Rain Jacket']) == 0:
        outer_array[2] = 0 
    else:
        outer_array[2] = 1
    
    #Shoes 
    # Casual Shoes
    if len(shoes_clothing['Casual Shoes']) == 0:
        shoes_array[0] = 0 
    else:
        shoes_array[0] = 1
    # Athletic Shoes
    if len(shoes_clothing['Athletic Shoes']) == 0:
        shoes_array[1] = 0 
    else:
        shoes_array[1] = 1
    # Dress Shoes
    if len(shoes_clothing['Dress Shoes']) == 0:
        shoes_array[2] = 0 
    else:
        shoes_array[2] = 1
    # Dressy Casual  Shoes
    if len(shoes_clothing['Business Casual Shoes']) == 0:
        shoes_array[3] = 0 
    else:
        shoes_array[3] = 1
    

    upper_list = [features['outside_warmth'], features['casual_formal'], features['comfort'], features['athletic']]
    lower_list = [features['outside_warmth'], features['casual_formal'], features['comfort'], math.fabs(1-features['athletic'])]
    outer_list = [features['outside_warmth'], features['casual_formal'], features['comfort'], features['rain'], features['snow']]
    shoes_list = [features['casual_formal'], features['comfort'], math.fabs(1-features['athletic'])]
    upper_instance = Instance.create_instance(upper_list, classname='weka.core.DenseInstance', weight= 1.0)
    lower_instance = Instance.create_instance(lower_list, classname='weka.core.DenseInstance', weight= 1.0)
    outer_instance = Instance.create_instance(outer_list, classname='weka.core.DenseInstance', weight= 1.0)
    shoes_instance = Instance.create_instance(shoes_list, classname='weka.core.DenseInstance', weight= 1.0)

    upper_path = '/home/leo/models/uppermodel2.model'
    lower_path = '/home/leo/models/lowermodel2.model'
    outer_path = '/home/leo/models/outermodel2.model'
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
    print "Outside Warmth:", features['outside_warmth'], "Inside-Outside:", features['inside_outside'], "Casual-Formal:", features['casual_formal'], "Comfort:", features['comfort'], "Athletic:", athleticstring, "Rain:", rainstring, "Snow:", snowstring



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

    for i in range(len(outer_array)):
        if outer_array[i] == 0:
            outer_prediction_array.append(0)
        else:
            outer_prediction_array.append(outer_predictions[i])

    for i in range(len(shoes_array)):
        if shoes_array[i] == 0:
            shoes_prediction_array.append(0)
        else:
            shoes_prediction_array.append(shoes_predictions[i])

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
            max_index_upper4 = i
    
    lower_prediction_array[max_index_lower4] = 0   

    for i in range(1, len(lower_prediction_array)):
        n = lower_prediction_array[max_index_lower5]
        if lower_prediction_array[i] > n:
            max_index_lower5 = i
    
    lower_indices = [max_index_lower1, max_index_lower2, max_index_lower3, max_index_lower4, max_index_lower5]

    max_index_outer1 = 0
    max_index_outer2 = 0
    max_index_outer3 = 0

    for i in range(1, len(outer_prediction_array)):
        n = outer_prediction_array[max_index_outer1]
        if outer_prediction_array[i] > n:
            max_index_outer1 = i

    outer_prediction_array[max_index_outer1] = 0

    for i in range(1, len(outer_prediction_array)):
        n = outer_prediction_array[max_index_outer2]
        if outer_prediction_array[i] > n:
            max_index_outer2 = i

    outer_prediction_array[max_index_outer2] = 0

    for i in range(1, len(outer_prediction_array)):
        n = outer_prediction_array[max_index_outer3]
        if outer_prediction_array[i] > n:
            max_index_outer3 = i

    outer_indices = [max_index_outer1, max_index_outer2, max_index_outer3]

    max_index_shoes1 = 0
    max_index_shoes2 = 0
    max_index_shoes3 = 0
    max_index_shoes4 = 0

    for i in range(1, len(shoes_prediction_array)):
        n = shoes_prediction_array[max_index_shoes1]
        if shoes_prediction_array[i] > n:
            max_index_shoes1 = i

    shoes_prediction_array[max_index_shoes1] = 0

    for i in range(1, len(shoes_prediction_array)):
        n = shoes_prediction_array[max_index_shoes2]
        if shoes_prediction_array[i] > n:
            max_index_shoes2 = i

    shoes_prediction_array[max_index_shoes2] = 0

    for i in range(1, len(shoes_prediction_array)):
        n = shoes_prediction_array[max_index_shoes3]
        if shoes_prediction_array[i] > n:
            max_index_shoes3 = i

    shoes_prediction_array[max_index_shoes3] = 0

    for i in range(1, len(shoes_prediction_array)):
        n = shoes_prediction_array[max_index_shoes4]
        if shoes_prediction_array[i] > n:
            max_index_shoes4 = i
    
    shoes_indices = [max_index_shoes1, max_index_shoes2, max_index_shoes3, max_index_shoes4]
    
    print "Outer Indices:", outer_indices
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


