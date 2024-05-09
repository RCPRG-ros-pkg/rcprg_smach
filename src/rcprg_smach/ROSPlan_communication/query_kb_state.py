#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from diagnostic_msgs.msg import *
from itertools import chain, combinations, permutations, product
from db_connection import dbConnector
import json
import pymongo

def get_plan():
    pt = "/rosplan_parsing_interface/complete_plan"
    action_feedback = rospy.wait_for_message(pt, CompletePlan, timeout=5)
    # print("Complete plan:")
    # print(action_feedback.plan)
    return (len(action_feedback.plan))

def get_full_plan():
    pt = "/rosplan_parsing_interface/complete_plan"
    action_feedback = rospy.wait_for_message(pt, CompletePlan, timeout=5)
    # print("Complete plan:")
    # print(action_feedback.plan)
    return action_feedback

def get_goals():
    # rosservice call  "predicate_name: ''"
    rospy.wait_for_service('/rosplan_knowledge_base/state/goals', timeout=10)
    try:
        predicate_name = ""
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/state/goals', GetAttributeService)
        resp = query_proxy(predicate_name)
        return resp.attributes
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def remove_goal(completed_goal):
    goals = get_goals()
    for goal in goals:
        v = goal.values[1]
        location = v.value
        print("LOCATION")
        print(location)
        print("COMPLETED GOAL")
        print(completed_goal)
        if location == completed_goal: 
            rospy.wait_for_service('/rosplan_knowledge_base/update', timeout=10)
            try:
                query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
                resp = query_proxy(3, goal)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    print("\n\n\n\n\n")
    print("GOALS STILL ACTIVE")
    print(get_goals())
    print("\n\n\n\n\n")

def update_goal(active_goal):
    goals = get_goals()
    for goal in goals:
        v = goal.values[1]
        location = v.value
        print("LOCATION")
        print(location)
        print("ACTIVE GOAL")
        print(active_goal)
        if location == active_goal: 
            rospy.wait_for_service('/rosplan_knowledge_base/update', timeout=10)
            try:
                query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
                resp = query_proxy(3, goal)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    print("\n\n\n\n\n")
    print("GOALS STILL ACTIVE")
    print(get_goals())
    print("\n\n\n\n\n")

    
def update_kb_snapshot_goals(task_id):
    dbc = dbConnector(task_id)
    active_goals = get_goals()
    for goal in active_goals:
        g = {'_id' :  goal.attribute_name,
            'attribute_name': goal.attribute_name,
            'values': goal.values}
        dbc.insert_predicate(g)
        print("Goal " + str(goal.attribute_name) + " inserted.")


def query_kb(ki):
    query = []
    rospy.wait_for_service('/rosplan_knowledge_base/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/query_state', KnowledgeQueryService)
        resp = query_proxy([ki])
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return ki


def query_kb_functions(ki):
    query = []
    rospy.wait_for_service('/rosplan_knowledge_base/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/state/functions_values', KnowledgeQueryService)
        resp = query_proxy([ki])
        resp_fk = resp.false_knowledge
        if len(resp_fk) > 0:
            function_value = resp_fk[0].function_value
            ki.function_value = function_value
            return ki
        else:
            return ki
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    return ki

def list_functions_updates():
    fd_list = fetch_functions_details()
    ki_list = []
    obj_inst = fetch_objects_instances()
    for fd in fd_list:
        tp_objects = []
        types = fetch_types()
        type_objects_dict = dict.fromkeys(types, 0)
        f_name = fd.name
        f_tp = fd.typed_parameters
        tp_objects = []
        for tp in f_tp:
            tp_objects.append(tp.value)
            type_objects_dict[tp.value] += 1

        for key, value in type_objects_dict.items():
            if value > 0:
                instances = obj_inst[key]

                permuted_instances = list(powerset(instances, value))

                for pi in permuted_instances:
                    ki = KnowledgeItem()
                    ki.knowledge_type = KnowledgeItem.FUNCTION
                    ki.attribute_name = fd.name
                    for i in pi:
                        ki.values.append(diagnostic_msgs.msg.KeyValue(key, i))
                    ki_list.append(query_kb_functions(ki))
    return ki_list


def list_predicates_updates():
    pd_list = fetch_predicates_details()
    ki_list = []
    obj_inst = fetch_objects_instances()
    for pd in pd_list:
        tp_objects = []
        types = fetch_types()
        type_objects_dict = dict.fromkeys(types, 0)
        p_name = pd.name
        p_tp = pd.typed_parameters
        tp_objects = []
        for tp in p_tp:
            tp_objects.append(tp.value)
            type_objects_dict[tp.value] += 1        
        permuted_instances_dict = {}
        permuted_instances_list = []
        permuted_instances = []
        for key, value in type_objects_dict.items():
            if value > 0:
                instances = obj_inst[key]
                # if a predicate is declared in domain file but there are no instances in problem file
                if len(instances) == 0:
                    break                    
                permuted_instances_dict[key] = (powerset(instances, value))
        
        for key, value in permuted_instances_dict.items():
            it = 0
            for v in value:
                if (type(v) is list or type(v) is tuple) and len(v) == 1:
                    if type(v) == "tuple":
                        value = list(v)
                    v = v[0]
                    value[it] = v
                it += 1

        for tpo in list(set(tp_objects)):
            for key, value in permuted_instances_dict.items():
                if key == tpo:
                    permuted_instances_list.append(value)

        if len(permuted_instances_list) == 1:
            permuted_instances_list = permuted_instances_list[0]
            for i in range(len(permuted_instances_list)):
                if type(permuted_instances_list[i]) == tuple:
                    permuted_instances_list[i] = list(permuted_instances_list[i])
        
        if len(permuted_instances_list) > 0 and len(permuted_instances_dict.keys()) > 1:
            if len(permuted_instances_list) == 1:
                for element in product(permuted_instances_list[0]):
                    permuted_instances.append(element)
            elif len(permuted_instances_list) == 2:
                for element in product(permuted_instances_list[0], permuted_instances_list[1]):
                    permuted_instances.append(element)
            elif len(permuted_instances_list) == 3:
                for element in product(permuted_instances_list[0], permuted_instances_list[1], permuted_instances_list[2]):
                    permuted_instances.append(element)
            elif len(permuted_instances_list) == 4:
                for element in product(permuted_instances_list[0], permuted_instances_list[1], permuted_instances_list[2], permuted_instances_list[3]):
                    permuted_instances.append(element)
        else:
            for element in permuted_instances_list:
                permuted_instances.append(tuple(element))
        
        permuted_instances = list(set(permuted_instances))
        for pi in permuted_instances:
            ki = KnowledgeItem()
            ki.knowledge_type = KnowledgeItem.FACT
            ki.attribute_name = pd.name
            if len(permuted_instances_dict.keys()) > 1:
                for i in pi:
                    # finding object type of instance i
                    for key, value in permuted_instances_dict.items():
                        tuples_equal = False
                        for v in value:
                            if tuple(i) == tuple(v):
                                tuples_equal = True
                                break
                        if tuples_equal:
                            if type(i) is tuple:
                                for ins in i:
                                    ki.values.append(diagnostic_msgs.msg.KeyValue(key, ins))
                            else:
                                ki.values.append(diagnostic_msgs.msg.KeyValue(key, i))
                            break
            else:
                for key, value in permuted_instances_dict.items():
                    tuples_equal = False
                    for v in value:
                        if tuple(pi) == tuple(v):
                            tuples_equal = True
                            break
                    if tuples_equal:
                        for i in tuple(pi):
                            ki.values.append(diagnostic_msgs.msg.KeyValue(key, i))
                        break
            ki_list.append([ki, query_kb(ki)])
    return ki_list
        
def fetch_functions_details():
    rospy.wait_for_service('/rosplan_knowledge_base/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/domain/functions', GetDomainAttributeService)
        function_details = query_proxy()
    except rospy.ServiceException, e:
        print "Fetch function service call failed: %s"%e
    return function_details.items

def fetch_predicates_details():
    rospy.wait_for_service('/rosplan_knowledge_base/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/domain/predicates', GetDomainAttributeService)
        predicates_details = query_proxy()
    except rospy.ServiceException, e:
        print "Fetch predicates service call failed: %s"%e
    return predicates_details.items

def fetch_types():
    rospy.wait_for_service('/rosplan_knowledge_base/domain/types', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/domain/types', GetDomainTypeService)
        types = query_proxy()
    except rospy.ServiceException, e:
        print "Fetch predicates service call failed: %s"%e
    return types.types

def fetch_objects_instances():
    types = fetch_types()
    type_instances_dict = {}
    for typ in types:
        rospy.wait_for_service('/rosplan_knowledge_base/state/instances', timeout=10)
        try:
            params = {'type_name' :typ,
                    'include_constants' :'false',
                    'include_subtypes' :'false'
                    } 
            query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/state/instances', GetInstanceService)
            instances = query_proxy(typ, False, False)
            type_instances_dict[typ] = instances.instances
        except rospy.ServiceException, e:
            print "Fetch predicates service call failed: %s"%e
    return type_instances_dict

def update_kb_snapshot(task_id):
    dbc = dbConnector(task_id)
    print("Successfully connected to DB of id "+str(task_id))
    snapshot = {}
    predicates = list_predicates_updates()
    print("List of predicates updates ready.")
    functions = list_functions_updates()
    print("List of functions updates ready.")
    predicates_list = []
    functions_list = []
    p_iter = 0
    f_iter = 0
    for predicate in predicates:
        ki = predicate[0]
        query_resp = predicate[1]
        att_name = ""
        values = []
        values_list = []
        values_dict = {}
        status = False
        if query_resp.all_true is False:
            query_resp.false_knowledge = query_resp.false_knowledge[0]
            att_name = query_resp.false_knowledge.attribute_name
            values_list = query_resp.false_knowledge.values
            status = False
        else:
            att_name = ki.attribute_name
            values_list = ki.values
            status = True

        for v in values_list:
            values_dict = {}
            values_dict[v.key] = v.value
            values.append(values_dict) 
        
        id = str(att_name)
        for v in values:
            id += str(v.values()[0])
        
        p = {'_id' :  id,
            'attribute_name': att_name,
            'values': values,
            'all_true': status}
        dbc.insert_predicate(p)
        print("Predicate " + str(att_name) + " inserted.")
        p_iter += 1

    for function in functions:
        values_list = []
        values_dict = {}
        values = []
        att_name = function.attribute_name
        values_list = function.values
        function_value = function.function_value

        for v in values_list:
            values_dict = {}
            values_dict[v.key] = v.value
            values.append(values_dict) 

        id = str(att_name)
        for v in values:
            id += str(v.values()[0])

        f = {'_id' :  id,
            'attribute_name': att_name,
            'values': values,
            'function_value': function_value}
        
        dbc.insert_function(f)
        print("Function " + str(att_name) + " inserted.")
        f_iter += 1

    if_restart = {'restart' : 1}
    dbc.insert_if_restart(if_restart)

    # UPDATE KB GOALS
    update_kb_snapshot_goals()
    
    print("\n\n\n\n")
    print("SWITCHED IF RESTART FLAG TO TRUE")
    print("\n\n\n\n")

def update_kb_from_db(id):
    dbc = dbConnector(id)
    [predicates, functions, goals] = dbc.read_db()
    for p in predicates:
        ki = KnowledgeItem()
        ki.knowledge_type = KnowledgeItem.FACT
        ki.attribute_name = str(p['attribute_name'])
        values_list = p['values']
        for value in values_list:
            k = str(value.keys()[0])
            v = str(value.values()[0])
            ki.values.append(diagnostic_msgs.msg.KeyValue(k, v))
        
        if p['all_true'] is True:
            remove_from_kb(ki)
            add_to_kb(ki)
        else:
            remove_from_kb(ki)
        
    for f in functions:
        old_ki = KnowledgeItem()
        old_ki.knowledge_type = KnowledgeItem.FUNCTION
        old_ki.attribute_name = str(f['attribute_name'])
        new_ki = KnowledgeItem()
        new_ki.knowledge_type = KnowledgeItem.FUNCTION
        new_ki.function_value = float(f['function_value'])
        new_ki.attribute_name = str(f['attribute_name'])
        values_list = f['values']
        for value in values_list:
            k = str(value.keys()[0])
            v = str(value.values()[0])
            new_ki.values.append(diagnostic_msgs.msg.KeyValue(k, v))
            old_ki.values.append(diagnostic_msgs.msg.KeyValue(k, v))
        f_val = query_kb_functions(old_ki).function_value
        old_ki.function_value = f_val
        remove_from_kb(old_ki)
        add_to_kb(new_ki)

    for g in goals:
        for v in g.values:
            if v.key == 'end-loc':
                update_goal(str(v.value))
        print("GOAL UPDATED")
        


def update_kb_db(id, ki, pred_status=True):
    values_list = []
    values_dict = {}
    values = []

    att_name = ki.attribute_name
    values_list = ki.values
    function_value = ki.function_value
    dbc = dbConnector(id)

    for v in values_list:
        values_dict = {}
        values_dict[v.key] = v.value
        values.append(values_dict) 

    id = str(att_name)
    for v in values:
        id += str(v.values()[0])
    
    if ki.knowledge_type == KnowledgeItem.FACT:
        f = {'_id' :  id,
            'attribute_name': att_name,
            'values': values,
            'all_true': pred_status}
        dbc.insert_predicate(f)
    
    elif ki.knowledge_type == KnowledgeItem.FUNCTION:
        f = {'_id' :  id,
            'attribute_name': att_name,
            'values': values,
            'function_value': function_value}
        dbc.insert_function(f)

def remove_from_kb(ki):
    print "Waiting for service"
    rospy.wait_for_service('/rosplan_knowledge_base/update', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
        # query_proxy takes 2 arguments. Update type (REMOVE_KNOWLEDGE=2, KI)
        resp = query_proxy(2, ki)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def add_to_kb(ki):
    rospy.wait_for_service('/rosplan_knowledge_base/update', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
        # query_proxy takes 2 arguments. Update type (REMOVE_KNOWLEDGE=2, KI)
        resp = query_proxy(0, ki)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False
        
def powerset(iterable, elements_number):
    s = list(iterable)
    return list(chain.from_iterable(permutations(s, r) for r in range(elements_number, elements_number+1)))


# ki = KnowledgeItem()
# ki.knowledge_type = KnowledgeItem.FUNCTION
# # ki.attribute_name = "battery-level"
# # ki.values.append(diagnostic_msgs.msg.KeyValue("robot", "rico"))
# ki.attribute_name = "distance"
# ki.values.append(diagnostic_msgs.msg.KeyValue("location", "start_point"))
# ki.values.append(diagnostic_msgs.msg.KeyValue("location", "end_point"))
# ki.function_value = 134

# ki = KnowledgeItem()
# ki.knowledge_type = KnowledgeItem.FACT
# ki.attribute_name = "at-location"
# ki.values.append(diagnostic_msgs.msg.KeyValue("robot", "rico"))
# ki.values.append(diagnostic_msgs.msg.KeyValue("location", "start_point"))

# update_kb_from_db(10)


# remove_from_kb(ki)
# print(query_kb(ki))
# add_to_kb(ki)
# print(query_kb_functions(ki))



# update_kb_db(10, ki)

# update_goal("kuchnia")
# print(get_goals())
# get_plan()