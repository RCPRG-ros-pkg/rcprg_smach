#!/usr/bin/env python
# encoding: utf8

# Copyright (c) 2019, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import rospy
from tiago_msgs.msg import Command
import pl_nouns.odmiana as ro

class PoseDescription:
    def __init__(self, parameters):
        self.parameters = parameters

class TaskMoveTo:
    def __init__(self, pose, place_name):
        parameters = {}
        if not pose is None:
            parameters['pose'] = pose

        if not place_name is None:
            if isinstance(place_name, str):
                parameters['place_name'] = place_name.decode('utf-8')
            elif isinstance(place_name, unicode):
                parameters['place_name'] = place_name
            else:
                raise Exception('Wrong type of "place_name": neither str nor unicode')

        self.__goal__ = PoseDescription(parameters)

    def getName(self):
        return 'move_to'

    def getGoal(self):
        return self.__goal__

class TaskBringGoods:
    def __init__(self, goods_name):
        if isinstance(goods_name, str):
            self.__goal__ = goods_name.decode('utf-8')
        elif isinstance(goods_name, unicode):
            self.__goal__ = goods_name

    def getName(self):
        return 'bring_goods'

    def getGoal(self):
        return self.__goal__

class TaskUniversal:
    def __init__(self, task_name):
        self.__name__ = task_name

    def getName(self):
        return self.__name__

    def getGoal(self):
        return None

class TaskManager:
    def __init__(self, callback_list):
        raise Exception('Do not use this class')
        self.__callback_list__ = callback_list
        self.o = ro.OdmianaRzeczownikow()
        rospy.Subscriber("rico_cmd", Command, self.callback)

    def przypadki(self, word):
        blocks = self.o.getBlocks(word)
        if len(blocks) == 0:
            print u'Nie moge znaleźć nazwy miejsca w słowniku'
            word_m = word
        else:
            m_lp = self.o.getMianownikLp(blocks)
            if len(m_lp) == 0:
                m_lm = self.o.getMianownikLp(blocks)
                word_m = m_lm[0]
            else:
                word_m = m_lp[0]

        word_d = self.o.getDopelniaczLp(blocks, mianownik=word_m)
        if len(word_d) == 0:
            word_d = self.o.getDopelniaczLm(blocks, mianownik=word_m)
        if len(word_d) == 0:
            word_d = [word_m]

        word_b = self.o.getBiernikLp(blocks, mianownik=word_m)
        if len(word_b) == 0:
            word_b = self.o.getBiernikLm(blocks, mianownik=word_m)

        if len(word_b) == 0:
            word_b = [word_m]

        return word_m, word_d[0], word_b[0]

    def taskMoveTo(self, place_name):
        pl_name = place_name.strip().lower()

        if pl_name == '':
            print u'Mam gdzieś iść, ale nie podano miejsca'
            return False

        place_name_m, place_name_d, place_name_b = self.przypadki(pl_name)

        print u'Poproszono mnie o przejście do ' + place_name_d.decode('utf-8') + u' (' + place_name_m.decode('utf-8') + u')'

        task = TaskMoveTo( None, place_name_m )
        return task

    def taskWander(self):
        print u'Poproszono mnie, abym zaczął patrolowac'
        return TaskUniversal('wander')

    def taskBring(self, object_name):
        print object_name
        ob_name = object_name.strip().lower()
        ob_name_m, ob_name_d, ob_name_b = self.przypadki(ob_name)
        print u'Poproszono mnie o przyniesienie ' + ob_name_d.decode('utf-8') + u' (' + ob_name_m.decode('utf-8') + u')'

        task = TaskBringGoods(object_name)
        return task

    def taskStop(self):
        print u'Poproszono mnie o zatrzymanie się.'
        return TaskUniversal('stop')

    def questionLoad(self):
        print u'Zapytano mnie: co wiozę?'
        return TaskUniversal('q_load')

    def questionCurrentTask(self):
        print u'Zapytano mnie: co robię?'
        return TaskUniversal('q_current_task')

    def respAck(self):
        print u'Usłyszałem ogólne potwierdzenie'
        return TaskUniversal('ack')

    def respAckIgave(self):
        print u'Usłyszałem potwierdzenie podania'
        return TaskUniversal('ack_i_gave')

    def respAckItook(self):
        print u'Usłyszałem potwierdzenie odebrania'
        return TaskUniversal('ack_i_took')

    def callback(self, data):
        param_dict = {}
        for param_name, param_value in zip(data.param_names, data.param_values):
            param_dict[param_name] = param_value
            print 'param_name, param_value', param_name, param_value

        task = None
        if data.intent_name == 'projects/incare-dialog-agent/agent/intents/176ab2ca-6250-4227-985b-cc82d5497d9f':
            task = self.taskBring(param_dict['przedmiot'])

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/0165eceb-9621-4a7d-aecc-7a879951da18':
            task = self.taskMoveTo( param_dict['miejsce'] )

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/7acd4325-4cdd-4e15-99be-ad545f4dddd5':
            task = self.taskStop()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/2f028022-05b6-467d-bcbe-e861ab449c17':
            print u'Niezrozumiałe polecenie: "' + data.query_text.decode('utf-8') + '"'

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/d9e96166-030b-442f-a513-d3fa2e044030':
            task = self.taskWander()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/6a3d7152-53c5-4757-9eec-8d2e0cf16e69':
            # Do nothing: greeting
            pass

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/b8743ab9-08a1-49e8-a534-abb65155c507':
            task = self.questionLoad()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/8f45359d-ee47-4e10-a1b2-de3f3223e5b4':
            task = self.questionCurrentTask()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/ef92199b-d298-470c-8df3-1e1047dd70d1':
            # Potwierdzenie
            task = self.respAck()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/d017cbd0-93f8-45b2-996e-043cdccab629':
            # Potw_podalem
            task = self.respAckIgave()

        elif data.intent_name == 'projects/incare-dialog-agent/agent/intents/181621b6-e91e-4244-a925-c5dc32ee1f1b':
            # Potw_odebralem
            task = self.respAckItook()

        else:
            raise Exception('Unknown intent: "' + data.intent_name + '", query_text: "' + data.query_text + '"')

        if not task is None:
            for cb in self.__callback_list__:
                cb( task )
