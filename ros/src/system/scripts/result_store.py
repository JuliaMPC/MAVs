#!/usr/bin/env python
import numpy as np
import rospy
import json
import csv
from std_msgs.msg import String, Float64, Int64, Bool
from mavs_msgs.msg import state, control
from nloptcontrol_planner.msg import Trajectory

def do_shutdown_preprocess():
    record_data_row()
    close_file()

def set_ros_param_shutdown_flag():
    rospy.set_param("result_store/flags/shutdown", True)

def get_csv_file_writer(file_path):
    global csv_file
    global csv_writer
    try:
        fh = open(file_path, 'r')
        fh.close()
        csv_file = open(file_path, 'a+') # create a new handle in append mode and return
        csv_writer = csv.writer(csv_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

    except Exception:
        # create a new file, add header and return get_file_handle
        csv_file = open(file_path, 'w+')
        csv_writer = csv.writer(csv_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        add_file_headers()

def add_file_headers():
    global model
    global csv_writer
    headers = []
    for obj in model["model"]:
        if "save" in obj and obj["save"] == True:
            k = "PLEASE_PROVIDE_KEY"
            if "key" in obj:
                k = obj["key"]
            headers.append(k)
    if model["add_description"] == True:
        d_header = "Description"
        if "description_header" in model:
            d_header = model["description_header"]
        headers.append(d_header)
    csv_writer.writerow(headers)

def close_file():
    global csv_file
    try:
        csv_file.close()
    except Exception:
        pass

def record_data_row():
    global model
    global csv_writer
    row=[]
    desc = ""
    for obj in model["model"]:
        if "save" in obj and obj["save"] == True:
            row.append(obj["__VAL__"])
        if obj["__CAUSED_SHUTDOWN__"] == True:
            if "description" in obj["stopping_criteria"]:
                desc = desc + obj["stopping_criteria"]["description"]
    if model["add_description"] == True:
        if len(desc) == 0 and "default_description" in model:
            desc = model["default_description"]
        row.append(desc)
    csv_writer.writerow(row)

def check_if_object_causes_shutdown(key):
    global shutdown_flag
    global model
    global key_map
    caused_shutdown = False
    ob = model["model"][key_map[key]]
    if not ("stopping_criteria" in ob) or not ("conditions" in ob["stopping_criteria"]):
        return False
    else:
        conditons = ob["stopping_criteria"]["conditions"]
        op = "None"
        if "logical_op" in ob["stopping_criteria"]:
            op = ob["stopping_criteria"]["logical_op"]
        caused_shutdown = evaluate_shutdown_conditions_for_object(ob["__VAL__"], conditons, op)
        model["model"][key_map[key]]["__CAUSED_SHUTDOWN__"] = caused_shutdown
        return caused_shutdown

def evaluate_shutdown_conditions_for_object(val, conditions, logical_op="None"):
    #default logical connector for conditions
    if logical_op is None:
        logical_op = "OR"
    logical_op = logical_op.upper()
    if logical_op == "AND":
        shutdown_criteria_met = True and len(conditions) > 0
        for c in conditions:
            shutdown_criteria_met = shutdown_criteria_met and eval_single_condition(val, c, logical_op)
            if shutdown_criteria_met == False:
                break
        return shutdown_criteria_met
    else:
        shutdown_criteria_met = False
        for c in conditions:
            shutdown_criteria_met = shutdown_criteria_met or eval_single_shutdown_condition(val, c, logical_op)
            if shutdown_criteria_met == True:
                break
        return shutdown_criteria_met

def eval_single_shutdown_condition(val, condition, logical_op):
    conditional_op = condition["op"]
    target = condition["val"]
    if not isinstance(val, list):
        return check_single_conditional_equality(val, target, conditional_op)
    else: # if value is a list then all element should satisfy conditions
        if logical_op == "AND":
            shutdown_criteria_met = True
            for v in val:
                shutdown_criteria_met = shutdown_criteria_met and check_single_conditional_equality(v, target, conditional_op)
                if shutdown_criteria_met == False:
                    break
            return shutdown_criteria_met
        else:
            shutdown_criteria_met = False
            for v in val:
                shutdown_criteria_met = shutdown_criteria_met or check_single_conditional_equality(v, target, conditional_op)
                if shutdown_criteria_met == True:
                    break
            return shutdown_criteria_met

def check_single_conditional_equality(val, target, conditional_op):
    conditional_op = conditional_op.lower()
    if conditional_op == "lt":
        return val < target
    elif conditional_op == "lte":
        return val <= target
    elif conditional_op == "gt":
        return val > target
    elif conditional_op == "gte":
        return val >= target
    elif conditional_op == "eq":
        return val == target
    elif conditional_op == "neq":
        return val != target


def add_dataval_and_caused_shutdown_variable_in_model_objects():
    global model
    for obj in model["model"]:
        obj["__VAL__"] = None
        obj["__CAUSED_SHUTDOWN__"] = False

# store a map of index of each object to save
def update_key_map():
    global key_map
    global model
    for i in range(len(model["model"])):
        key_map[model["model"][i]["key"]] = i

def update_object_val(msg, args):
    global model
    global key_map
    global shutdown_flag
    if shutdown_flag == False:
        key = args[0]
        sub_val = args[1]
        val = msg
        if sub_val is not None:
            child_arr = sub_val.split("/")
            for ch in child_arr:
                val=getattr(val, ch)
        model["model"][key_map[key]]["__VAL__"] = val
        shutdown_flag = check_if_object_causes_shutdown(key)

def update_ros_parameters_in_model():
    global model
    for obj in model["model"]:
        if obj["ros_msg_type"].lower() == "parameter":
            key = obj["key"]
            if rospy.has_param(obj["val"]):
                val = rospy.get_param(obj["val"])
                update_object_val(val, (key, None))

def subscribe_to_topics_in_model():
    global model
    global subscribers
    for obj in model["model"]:
        if obj["ros_msg_type"].lower() == "topic":
            _key = obj["key"]
            topic_name = obj["val"]
            sub_val = None
            if "sub_val" in obj:
                sub_val = obj["sub_val"]
            _dtype = obj["dataType"]
            if _dtype == "Bool":
                sub = rospy.Subscriber(topic_name, Bool, update_object_val, (_key, None))
            elif _dtype == "String":
                sub = rospy.Subscriber(topic_name, String, update_object_val, (_key, None))
            elif _dtype == "Int64":
                sub = rospy.Subscriber(topic_name, Int64, update_object_val, (_key, None))
            elif _dtype == "Float64":
                sub = rospy.Subscriber(topic_name, Float64, update_object_val, (_key, None))
            elif _dtype == "state":
                sub = rospy.Subscriber(topic_name, state, update_object_val, (_key, sub_val))
            elif _dtype == "control":
                sub = rospy.Subscriber(topic_name, control, update_object_val, (_key, sub_val))
            elif _dtype == "Trajectory":
                sub = rospy.Subscriber(topic_name, Trajectory, update_object_val, (_key, sub_val))
            subscribers.append(sub)

def update_shutdown_flag_for_future_enhancements():
    # In future you can call your own functions here to set the global shutdown_flag
    # It will automatically trigger shutdown with parameter saving
    pass

if __name__ == '__main__':
    rospy.init_node("result_store")
    csv_file = ""
    csv_writer = None
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
    get_csv_file_writer(file_path)

    # Add a __VAL__ and __CAUSED_SHUTDOWN__ to all model objects
    add_dataval_and_caused_shutdown_variable_in_model_objects()

    # subscribe to topics in a loop and update them with new data continuously
    subscribe_to_topics_in_model()
    # Set shutdown callback for saving file, etc
    rospy.on_shutdown(do_shutdown_preprocess)
    rospy.set_param("system/result_store/flags/initialized", True)
    rate = rospy.Rate(10) # 50hz
    while not rospy.is_shutdown() and not shutdown_flag:
        update_ros_parameters_in_model()
        update_shutdown_flag_for_future_enhancements()
        rate.sleep()
    if shutdown_flag == True:
        set_ros_param_shutdown_flag()
