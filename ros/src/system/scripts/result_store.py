#!/usr/bin/env python
import numpy as np
import rospy
import json
import csv
from std_msgs.msg import String, Float64, Int64, Bool
from mavs_msgs.msg import state, control
from nloptcontrol_planner.msg import Trajectory

def request_shutdown(csv_writer):
    record_data_row(csv_writer)
    close_file(csv_file)
    rospy.set_param("result_store/flags/shutdown", True)

def get_csv_file_writer(file_path):
    global csv_file
    try:
        fh = open(file_path, 'r')
        fh.close()
        csv_file = open(file_path, 'a+') # create a new handle in append mode and return
        return csv.writer(csv_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

    except Exception:
        # create a new file, add header and return get_file_handle
        csv_file = open(file_path, 'w+')
        writer = csv.writer(csv_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        add_file_headers(writer)
        return writer

def add_file_headers(csv_writer):
    global model
    headers = []
    for obj in model["model"]:
        if "save" in obj and obj["save"] == True:
            k = "PLEASE_PROVIDE_KEY"
            if "key" in obj:
                k = obj["key"]
            headers.append(k)
    csv_writer.writerow(headers)

def close_file():
    global csv_file
    try:
        csv_file.close()
    except Exception:
        pass

def record_data_row():
    pass

def add_dataval_variable_in_model_objects():
    global model
    for obj in model["model"]:
        obj["__VAL__"] = None

# store a map of index of each object to save
def update_key_map():
    global key_map
    global model
    for i in range(len(model["model"])):
        key_map[model["model"][i]["key"]] = i

def update_topic_val(msg, args):
    global model
    global key_map
    key = args[0]
    sub_val = args[1]
    val = msg
    if sub_val is not None:
        child_arr = sub_val.split("/")

        print "###value = ", val, "\n"
        for ch in child_arr:
            # TODO debug this
            #val=val[ch]
            print "###chil = ", ch, "type = ", type(val) ,"\n"
    print "updating model at idx =", key_map[key], "at key =", key, "with val", val
    model["model"][key_map[key]]["__VAL__"] = msg

def subscribe_to_topics_in_model():
    global model
    global subscribers
    for obj in model["model"]:
        if obj["ros_msg_type"] == "topic":
            _key = obj["key"]
            topic_name = obj["val"]
            sub_val = None
            if "sub_val" in obj:
                sub_val = obj["sub_val"]
            _dtype = obj["dataType"]
            if _dtype == "Bool":
                sub = rospy.Subscriber(topic_name, Bool, update_topic_val, (_key, None))
            elif _dtype == "String":
                sub = rospy.Subscriber(topic_name, String, update_topic_val, (_key, None))
            elif _dtype == "Int64":
                sub = rospy.Subscriber(topic_name, Int64, update_topic_val, (_key, None))
            elif _dtype == "Float64":
                sub = rospy.Subscriber(topic_name, Float64, update_topic_val, (_key, None))
            elif _dtype == "state":
                print "######### Added 5", topic_name
                sub = rospy.Subscriber(topic_name, state, update_topic_val, (_key, sub_val))
            elif _dtype == "control":
                sub = rospy.Subscriber(topic_name, control, update_topic_val, (_key, sub_val))
            elif _dtype == "Trajectory":
                sub = rospy.Subscriber(topic_name, Trajectory, update_topic_val, (_key, sub_val))
            subscribers.append(sub)

def check_shutdown_conditions():
    global shutdown_flag
    global model
    pass
#flag_traj = rospy.Subscriber("nlopcontrol_planner/control", Trajectory, plot_opt)
#ax_min_temp = rospy.get_param('/vehicle/nloptcontrol_planner/ax_min')
if __name__ == '__main__':
    rospy.init_node("result_store")
    csv_file = ""
    model = {}
    key_map = {}
    shutdown_flag = False
    subscribers=[]

    file_path = rospy.get_param("/result_store/results_file")
    model_path = rospy.get_param("/result_store/model_filename_path")
    # load data model to save
    with open(model_path) as f:
        model = json.load(f)
    update_key_map()
    csv_writer = get_csv_file_writer(file_path)

    # Add a __VAL__ to all model objects
    add_dataval_variable_in_model_objects()

    # subscribe to topics in a loop and update them with new data continuously
    subscribe_to_topics_in_model()

    rospy.set_param("system/result_store/flags/initialized", True)
    rate = rospy.Rate(2) # 50hz
    while not rospy.is_shutdown() and not shutdown_flag:
        print "############Yay force=", model["model"][3]["__VAL__"]
        # loop over model for ros parameter updates
        # TODO

        check_shutdown_conditions()
        rate.sleep()
    request_shutdown(csv_writer)
