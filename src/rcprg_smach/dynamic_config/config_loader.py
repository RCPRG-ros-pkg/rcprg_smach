#!/usr/bin/env python
# encoding: utf8

from cgitb import text
from xml.dom import NOT_FOUND_ERR
import xml.etree.ElementTree as ET

config_filepath = '/home/nkvch/tiago_public_ws/src/rcprg/rcprg_smach/src/rcprg_smach/dynamic_config/index.xml'

class DynamicConfigParser:
    def __init__(self):
        self.tree = ET.parse(config_filepath)
        self.root = self.tree.getroot()

    def create_text_element(self, tag, text):
        elt = ET.Element(tag)
        elt.text = text

        return elt
    
    def append_by_dict(self, elt, dict):
        for tag, text in dict.items():
            elt.append(self.create_text_element(tag, text))

    def get_children_as_dict(self, elt):
        dict_ = {}
        for child in elt.find('.'):
            dict_[child.tag] = child.text
        return dict_

    def append_many(self, elt, children):
        for child in children:
            elt.append(child)

    def get_tasks(self):
        def map_to_task(task_element):
            intent_id = task_element.find('intent').text
            intent_name = task_element.find('name').text
            intent_priority = task_element.find('priority').text
            intent_params_element = task_element.find('params')
            intent_params = list(map(lambda param: param.text, intent_params_element))

            return (intent_id, intent_name, intent_priority, intent_params)
        
        tasks = list(map(map_to_task, self.root))

        return tasks

    def get_task_inputs(self, intent_name):
        tasks = self.root.findall('task')

        result = None

        for task in tasks:
            if task.find('intent').text == intent_name:
                inputs = task.find('inputs')
                result = list(map(lambda input_element: (input_element.find('name').text, input_element.find('intent').text), inputs.findall('input')))

        if result is None:
            raise BaseException('Not found XML node')
        
        return result

    def add_task_input(self, task_name, input_name, input_intent):
        tasks = self.root.findall('task')

        found = False

        for task in tasks:
            if task.find('name').text == task_name:
                new_input = ET.Element('input')
                intent_element = ET.Element('intent')
                intent_element.text = input_intent
                new_input.append(intent_element)
                name_element = ET.Element('name')
                name_element.text = input_name
                new_input.append(name_element)
                task_inputs = task.find('inputs')
                task_inputs.append(new_input)
                found = True
                break
        
        if not found:
            raise BaseException('Not found XML node')

        self.tree.write(config_filepath)
        return found

    def get_task(self, intent_name):
        tasks = self.root.findall('task')

        found = False

        task_name = None
        priority = None
        params = []
        inputs = []

        for task in tasks:
            if task.find('intent').text == intent_name:
                task_name = task.find('name').text
                priority = task.find('priority').text
                params_element = task.find('params')
                for param in params_element.findall('param'):
                    params.append(param.text)
                inputs_element = task.find('inputs')
                for input_element in inputs_element.findall('input'):
                    inputs.append(self.get_children_as_dict(input_element))
                found = True
                break
        
        if not found:
            raise BaseException('Not found XML node')
        
        return intent_name, task_name, priority, params, inputs
        

    def add_task(self, intent_name, task_name, priority, params, inputs):
        task_element = ET.Element('task')
        intent_element = self.create_text_element('intent', intent_name)
        name_element = self.create_text_element('name', task_name)
        priority_element = self.create_text_element('priority', priority)
        params_element = ET.Element('params')
        inputs_element = ET.Element('inputs')

        for param in params:
            params_element.append(self.create_text_element('param', param))

        for inpt in inputs:
            input_element = ET.Element('input')
            self.append_by_dict(input_element, inpt)
            inputs_element.append(input_element)
        
        self.append_many(task_element, [intent_element, name_element, priority_element, params_element, inputs_element])

        self.root.append(task_element)
        self.tree.write(config_filepath)

    def clone_task_with_new_intent(self, existing_intent, new_intent, additional_params):
        intent_name, task_name, priority, params, inputs = self.get_task(existing_intent)
        self.add_task(new_intent, task_name, priority, params + additional_params, inputs)

    def get_params_for_task(self, intent_name):
        tasks = self.root.findall('task')

        found = False

        params = []

        for task in tasks:
            if task.find('intent').text == intent_name:
                found = True
                params_elt = task.find('params')
                for param_elt in params_elt.findall('param'):
                    params.append(param_elt.text)
        
        if not found:
            raise BaseException('Not found XML node')

        return params
