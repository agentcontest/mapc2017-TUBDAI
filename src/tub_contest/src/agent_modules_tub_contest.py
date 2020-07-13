from __future__ import division  # force floating point division when using plain /
import rospy
import random
import math
import threading
from abc import ABCMeta, abstractmethod

from behaviour_components.behaviours import BehaviourBase
from behaviour_components.activators import MultiSensorCondition
from behaviour_components.sensors import PassThroughTopicSensor, Sensor
from utils.ros_helpers import get_topic_type

from knowledge_base.knowledge_base_client import KnowledgeBaseClient

from diagnostic_msgs.msg import KeyValue

from mac_ros_bridge.msg import *
from tub_contest.msg import *
from list_of_strings import *
from list_of_numbers import *

# patrick

def get_bridge_topic_prefix(agent_name):
    """
    Determine the topic prefix for all topics of the bridge node corresponding to the agent
    :param agent_name: current agents name
    :return: prefix just before the topic name of the bridge
    """
    return '/bridge_node_' + agent_name + '/'


def action_generic_simple(publisher, action_type, params=[]):
    """
    Generic helper function for publishing GenericAction msg
    :param publisher: publisher to use
    :param action_type: the type of the action msg
    :param params: optional parameter for the msg
    """
    action = GenericAction()
    action.action_type = action_type
    action.params = params
    publisher.publish(action)


def action_goto(facility_name, publisher):
    """
    Specific "goto" action publishing helper function
    :param facility_name: name of the facility we want to go to
    :param publisher: publisher to use
    """
    action = GenericAction()
    action.action_type = "goto"
    action.params = [KeyValue("Facility", facility_name)]
    publisher.publish(action)


def euclidean_distance(pos1, pos2):
    """
    Calculate the euclidean distance between two positions
    :param pos1: position 1
    :type pos1: Position
    :param pos2: position 2
    :type pos2: Position
    :return: euclidean distance
    """
    return math.sqrt((pos1.lat - pos2.lat) ** 2 + (pos1.long - pos2.long) ** 2)


def get_knowledge_base_tuple_facility_exploration(agent_name, facility):
    """
    Simple function to create uniform knowledge tuples for facility exploration
    :param agent_name: name of the considered agent
    :param facility: facility name or topic that is explored
    :return: generate tuple (agent_name, exploration_key)
    """
    return agent_name, 'exploring_' + facility


def get_knowledge_base_tuple_has_task(agent_name, group, has_task):
    """
    Simple function to create uniform knowledge tuples for facility exploration
    :param agent_name: name of the considered agent
    :param has_task: indicates whether the agent has a task or not with a string true or false
    :return: generate tuple (agent_name, task_key)
    """
    return agent_name, group, 'task_' + has_task

def get_item_amount(agent, item_name):
    """
    Get agents item amount
    :param agent: Agent perception
    :param item_name: name of requested item
    :return: item amount
    """
    for item in agent.items:
        if item.name == item_name:
            return item.amount
    return 0


class GotoChargingBehaviour(BehaviourBase):
    """
    Behaviour that goes to the closest charging station
    """

    def __init__(self, agent_name, facility_topic, ref_topic, **kwargs):
        super(GotoChargingBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._facility_topic = facility_topic

        self._facilities = {}

        self._selected_facility = None

        self.__client = KnowledgeBaseClient(knowledge_base_name=KNOWLEDGE_BASE_NAME)

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action',
                                                   GenericAction, queue_size=10)

        self._exploration_knowledge = get_knowledge_base_tuple_facility_exploration(self._agent_name,
                                                                                    self._facility_topic)

        facility_topic_type = get_topic_type(facility_topic)

        self._agent = None

        ref_message_type = get_topic_type(ref_topic)
        if ref_message_type is not None:
            self._sub_ref = rospy.Subscriber(ref_topic, ref_message_type, self.subscription_callback_ref_topic)
        else:
            rospy.logerr("Could not determine message type of: " + ref_topic)

        if facility_topic_type:
            self._sub_facility = rospy.Subscriber(facility_topic, facility_topic_type, self._callback_facility)
        else:
            rospy.logerr(self._agent_name + "::" + self._name + ": Failed to determine topic type of " + facility_topic)

    def subscription_callback_ref_topic(self, msg):
        # update most current reference of this agent
        self._agent = msg

    def _callback_facility(self, msg):
        # Store all available facilities in a dict
        for facility in msg.facilities:
            self._facilities[facility.name] = facility

    def move(self):

        if self._selected_facility:  # in case we did not find/know a facility

            rospy.logdebug(self._agent_name + "::" + self._name + " to " + self._selected_facility.name)

            action_goto(facility_name=self._selected_facility.name, publisher=self._pub_generic_action)
        else:  # backup action recharge agent
            rospy.logwarn(self._agent_name + "::" + self._name + " recharging because of missing facility.")
            action_generic_simple(publisher=self._pub_generic_action, action_type=RECHARGE_STRING)

    def get_closest_facility(self, agent, facility_list):
        """
        find for a given agent and a list of facilities the closest facility to the agent
        :param agent: Agent
        :param facility_list: list of facilities like charging stations
        :return: the closest facility
        """
        max_value = 99999
        best = None
        #TODO use graphhopper distance here
        for k, fac in facility_list.items():
            if (euclidean_distance(agent.pos, fac.pos) < max_value):
                max_value = euclidean_distance(agent.pos, fac.pos)
                best = fac
        return best

    def _select_facility(self):
        """
        find the closest charging station
        :return: facility
        """
        return self.get_closest_facility(self._agent, self._facilities)

    def start(self):

        self._selected_facility = self._select_facility()

        if self._selected_facility:

            facility_name = self._selected_facility.name
        else:
            facility_name = 'None'

        rospy.logdebug(self._agent_name + ": going to facility: " + facility_name)

        self.do_step()  # this is important to directly answer the request as in the start() base implementation

    def do_step(self):

        if not self._selected_facility:
            self._selected_facility = self._select_facility()

        self.move()

    def unregister(self, terminate_services=True):
        super(GotoChargingBehaviour, self).unregister(terminate_services=terminate_services)
        self._pub_generic_action.unregister()
        self._sub_facility.unregister()


class GotoLocationBehaviour(BehaviourBase):
    """
    Behaviour that goes to the location specified in the published Task for this agent
    """

    def __init__(self, agent_name, task_topic, sim_id, **kwargs):

        super(GotoLocationBehaviour, self) \
            .__init__(requires_execution_steps=True, agent_name=agent_name, facility_topic=task_topic, **kwargs)

        self._agent_name = agent_name
        self._sim_id = sim_id

        self._facility_topic = task_topic

        self._facilities = {}

        self._selected_facility = None

        self.__client = KnowledgeBaseClient(knowledge_base_name=KNOWLEDGE_BASE_NAME)

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action',
                                                   GenericAction, queue_size=10)
        self.my_task = None

        self.params = []

        self._sub_task = rospy.Subscriber(task_topic, Task, self._callback_task)

    def _callback_task(self, msg):
        # Store the facility specified in the Task for this agent
        if msg.agent == self._agent_name and self._sim_id == msg.sim_id:
            self.my_task = msg
            self._selected_facility = msg.facility
            #rospy.logdebug("%s: task: %s; destination: %s",self._agent_name, self.my_task.action, self._selected_facility)
        elif self._sim_id != msg.sim_id:
            rospy.logdebug("%s: my sim-id: %s; msg sim-id: %s", self._agent_name, self._sim_id, msg.sim_id)

    def move(self):

        if self._selected_facility:  # in case we did not find/know a facility
            # if we want to go to a resource node
            if(self.my_task.action == GATHER_STRING):
                #print(self.my_task.facility)
                target=string_to_position(self.my_task.facility)
                self.params = [KeyValue("lat", str(target.lat)),
                               KeyValue("lon", str(target.long))]
                action_generic_simple(publisher=self._pub_generic_action, action_type="goto",
                                      params=self.params)
            # if we want to go to a facility
            else:
                action_goto(facility_name=self._selected_facility, publisher=self._pub_generic_action)
        else:  # backup action recharge agent
            action_generic_simple(publisher=self._pub_generic_action, action_type=RECHARGE_STRING)

    def start(self):
        if self.my_task:
            facility = self.my_task.facility
        else:
            facility = ''
        rospy.logdebug(self._agent_name + "::" + self._name + " " + facility)

        self.do_step()  # this is important to directly answer the request as in the start() base implementation

    def do_step(self):

        self.move()

    def unregister(self, terminate_services=True):
        super(GotoLocationBehaviour, self).unregister(terminate_services=terminate_services)
        self._pub_generic_action.unregister()
        self._sub_task.unregister()


class GenericActionBehaviour(BehaviourBase):
    """
    A simple behaviour for triggering generic MAC actions that just need a type and static parameters
    """

    def __init__(self, name, agent_name, action_type, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAC action
        :param params: optional parameters for the MAC action
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(GenericActionBehaviour, self) \
            .__init__(name=name,
                      requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._action_type = action_type
        self._params = params
        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action',
                                                   GenericAction, queue_size=10)

    def do_step(self):
        rospy.logdebug(self._agent_name + "::" + self._name + " executing: " + self._action_type)
        action_generic_simple(publisher=self._pub_generic_action, action_type=self._action_type, params=self._params)


    def unregister(self, terminate_services=True):
        super(GenericActionBehaviour, self).unregister(terminate_services=terminate_services)
        self._pub_generic_action.unregister()



class ItemActionBehavior(BehaviourBase):
    """
    A behavior that send the action specified in the Task for this agent
    """

    def __init__(self, agent_group, name, agent_name, agent_topic, sim_id, path_planner, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAC action
        :param params: optional parameters for the MAC action
        :param kwargs: more optional parameter that are passed to the bas class
        """

        super(ItemActionBehavior, self) \
            .__init__(name=name,
                      requires_execution_steps=True, **kwargs)
        self.waiting_dic = {}
        self.received_dic = {}
        self.all_tasks = []
        self._agent_name = agent_name
        self._sim_id = sim_id
        self._path_planner = path_planner
        self.request_action = None
        self._action_type = None
        self._my_task = None
        self._params = params
        self.assemble_for = {} #hold information about which agents wants to assemble which item
        self.shops = {} # two nested dicts, ordered by jobs and items # TODO delete old information
        self._this_agent = None
        self.should_amount = 0
        self.workshops = {}
        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action',
                                                   GenericAction, queue_size=10)

        self._pub_finished_task = rospy.Publisher('/finished_task' + agent_group, Task, queue_size=NUMBER_OF_AGENTS, latch=True)
        self._pub_failed_task = rospy.Publisher('/failed_task'     + agent_group, Task, queue_size=NUMBER_OF_AGENTS, latch=True)

        self._task_lock = threading.Lock()

        self._task_completed = False
        self._last_executed_action = None
        self._sub_request_action = rospy.Subscriber(get_bridge_topic_prefix(agent_name) + "request_action", RequestAction, self._callback_request_action)
        rospy.Subscriber('/negotiation' + agent_group, Task, self.task_assignment_callback)
        self._sub_task = rospy.Subscriber("/task" + agent_group, Task, self._callback_task)
        self._sub_shop = rospy.Subscriber("/shop", ShopMsg, self._callback_shop)
        self._sub_workshop = rospy.Subscriber("/workshop", WorkshopMsg, self._callback_workshop)
        self._sub_agent = rospy.Subscriber(agent_topic, Agent, self._callback_agent)

    def _callback_request_action(self, msg):
        self.request_action = msg

    def _callback_shop(self, msg):
        for shop in msg.facilities:
            self.shops[shop.name] = shop

    def _callback_workshop(self, msg):
        for workshop in msg.facilities:
            self.workshops[workshop.name] = workshop

    def _callback_agent(self, msg):
        self._this_agent = msg

    def _callback_task(self, msg):
        if self._sim_id == msg.sim_id:
            with self._task_lock:
                if msg.agent == self._agent_name:
                    self._task_completed = False
                    self._my_task = msg
                    self._action_type = self._my_task.action
                    #TODO add for gather also a mechanism for not getting to many items
                    if (msg.action == BUY_STRING) :
                        if self._this_agent:
                            has_amount = get_item_amount(self._this_agent, self._my_task.item)
                            self.should_amount = msg.amount + has_amount
                if msg.action == ASSEMBLE_STRING:
                    self.assemble_for[msg.task_id] = msg

    def do_step(self):

        if self._task_completed:
            rospy.loginfo("%s: %s %s already completed --> Executing 'recharge'", self._agent_name, self._action_type, self._my_task.item)
            self._idle_action()
            return
        #rospy.logdebug(self._agent_name + "::" + self._name + " executing: " + self._action_type)
        if self._action_type == BUY_STRING:

            shop_amount = self.get_shop_amount(self.shops.get(self._my_task.facility), self._my_task.item)

            has_amount = get_item_amount(self._this_agent, self._my_task.item)

            item_amount = self.should_amount - has_amount

            # check if we really bought everything needed
            if item_amount <= 0:
                self._finalise_task()
                return

            if (shop_amount < item_amount):
                item_amount = shop_amount
                rospy.logdebug("%s:: Could not buy all items of %s. Required %d, Agent loaded %d, in shop %d", self._agent_name,
                              self._my_task.item, self.should_amount, has_amount, shop_amount)

            if (item_amount <= 0):
                self._idle_action()
                return

            # print("behavior", self._agent_name, BUY_STRING, item_amount, shop_amount, has_amount, self.should_amount,
            #      self._my_task.item)
            self._params = [KeyValue("Item", self._my_task.item), KeyValue("Amount", str(item_amount))]

        if self._action_type == ASSIST_ASSEMBLE_STRING:
            assisted_task = self.assemble_for.get(self._my_task.parent_task_id)
            if assisted_task:
                if (self._one_teammate_is_ready_assist_assemble()):
                    self._params = [KeyValue("Agent", assisted_task.agent)]
                else:
                    rospy.logwarn('%s: %s Waiting for team mate', self._agent_name, self._action_type)
                    self._idle_action()
                    return
                # if self._was_executed_successfully():
                #     self._task_completed = True
                #     self._idle_action()
                #     return
                #rospy.loginfo("%s: %s for %s : item %s amount %d", self._agent_name, ASSIST_ASSEMBLE_STRING, assisted_task.agent, self._my_task.item, self._my_task.amount)
                #rospy.loginfo("%s: %s  My task: \n %s \n Assist task: \n%s", self._agent_name, ASSIST_ASSEMBLE_STRING, str(self._my_task), str(assisted_task) )
                # print(self._agent_name,self.assemble_for.get(self._my_task.part_of ),self._my_task.item,self._my_task.amount)
            else:
                # this can happen when the other task was not yet shared ...
                rospy.logdebug("%s: %s: Missing assisted task for %s part of %s (Parent:%s)", self._agent_name, self._action_type, self._my_task.item, self._my_task.part_of, self._my_task.parent_task_id)
                self._idle_action()
                return
        if self._action_type == ASSEMBLE_STRING:

            if (self._one_teammate_is_ready_assemble()):
                self._params = [KeyValue("Item", self._my_task.item)]
            else:
                rospy.logwarn('%s: %s Waiting for team mate', self._agent_name, self._action_type)
                self._idle_action()
                return
            # if self._was_executed_successfully():
            #     self._task_completed = True
            #     self._idle_action()
            #     return
        if self._action_type == DELIVER_JOB_STRING:
            self._params = [KeyValue("Job", self._my_task.job_id)]
            has_amount = get_item_amount(self._this_agent, self._my_task.item)
            if self._was_executed_successfully():
                self._finalise_task()
                return
            elif 0 == has_amount:
                rospy.logwarn("%s: %s: probably completed for %s", self._agent_name, self._action_type, self._my_task.item)
                self._finalise_task()
                return
            elif self._my_task.amount > has_amount:
                rospy.logwarn("%s: %s: not enough items of %s, required amount %d, available %d", self._agent_name,
                              self._action_type, self._my_task.item, self._my_task.amount, has_amount)
                self._idle_action()
                return

        if self._action_type == RETRIEVE_STRING:
            self._params = [KeyValue("item", self._my_task.item),KeyValue("amount", str(self._my_task.amount))]

        if self._action_type == GATHER_STRING:
            self._params = []
            # print(self._agent_name,self._params,self._action_type,"od")
            if self._action_type == self._this_agent.last_action and (self._this_agent.last_action_result == "failed_capacity"
                                                                     or self._this_agent.last_action_result == "failed_wrong_facility"
                                                                     or self._this_agent.last_action_result == "failed_location"):
                self._task_failed()
                return

        if self._action_type: # avoid problems during initialisation with this check
            action_generic_simple(publisher=self._pub_generic_action, action_type=self._action_type, params=self._params)
            self._last_executed_action = self._pub_generic_action, self._action_type, self._params


    def task_assignment_callback(self,msg):

        if (self.received_dic.get(msg.agent) is None):
            self.received_dic[msg.agent] = msg
        else:
            self.waiting_dic[msg.agent] = msg

        if (len(self.received_dic) == NUMBER_OF_AGENTS):
            task_list = self.received_dic.copy()
            self.received_dic = self.waiting_dic.copy()
            self.waiting_dic = {}

            min_task = None

            # find best rating
            # for task in task_list:
            for k, task in task_list.items():
                if min_task is None or task.rating < min_task.rating:
                    min_task = task
                # if several agents have the same rating take the one with the "smaller" name(lower suffix)
                # needed as order might not in all agent the same

                if min_task and task.rating == min_task.rating:
                    if (task.agent < min_task.agent):
                        min_task = task
            # print(task.agent, min_task.item, min_task.rating, min_task.action, min_task.task_id, len(self.open_tasks))

            # if rating is max_rating - let task gets re - assigned
            if (min_task and min_task.rating >= MAX_VALUE):
                return

            if min_task:
                self.all_tasks.append(min_task)

    def _one_teammate_is_ready_assemble(self):
        this_position = self._my_task.facility
        #print("num all ", len(self.all_tasks), self._agent_name)
        for task in self.all_tasks:
            #if the other task is assist_assembling the same item like this agent is assembling
            if self._my_task.task_id == task.parent_task_id:
                #print("task found", task.agent, task.task_depth, self._my_task.task_depth, self._agent_name)
                if not self._agent_not_in_position(task.agent, this_position):
                    return True
        return False


    def _one_teammate_is_ready_assist_assemble(self):
        this_position = self._my_task.facility
        #print("num all ",len(self.all_tasks),self._agent_name)
        for task in self.all_tasks:
            #if the other task is also assist_assembling for the same item or assembling the item
            if(self._my_task.parent_task_id == task.parent_task_id)or(self._my_task.parent_task_id == task.task_id):
                #print("task found",task.agent,task.task_depth,self._my_task.task_depth,self._agent_name)
                if not self._agent_not_in_position(task.agent, this_position):
                    return True
        return False


    def _agent_not_in_position(self, agent, facility):
        position = self.workshops.get(facility).pos
        for entity in self.request_action.entities:
            if(entity.name == agent):
                if self._path_planner.are_same_location(entity.pos,position):
                    #print("in position", entity.name,position.lat,position.long,self._path_planner.air_distance(entity.pos,position),self._agent_name)
                    return False
                else:
                    #print("not in position", entity.name, entity.pos.lat, entity.pos.long,self._path_planner.air_distance(entity.pos,position),self._agent_name)
                    return True
        return True

    def _task_failed(self):
        rospy.logwarn("%s: Task failed: %s",self._agent_name, str(self._my_task))
        self._pub_failed_task.publish(self._my_task)
        self._idle_action()

    def _finalise_task(self):
        """
        set task state to completed and notify about completion
        """
        self._task_completed = True
        #rospy.logwarn("%s: Finished task %s", self._agent_name, str(self._my_task))
        self._pub_finished_task.publish(self._my_task)
        self._idle_action()

    def _was_executed_successfully(self):
        """
        check if the task was already successfully completed
        :return: True if completed successfully
        """
        if self._last_executed_action and \
            self._action_type == self._this_agent.last_action and (self._this_agent.last_action_result == "successful" or \
            self._this_agent.last_action_result == "successful_partial") and \
            self._last_executed_action == (self._pub_generic_action, self._action_type, self._params):
                return True

        return False

    def start(self):
        self.do_step()

    def _idle_action(self):
        action_generic_simple(publisher=self._pub_generic_action, action_type=RECHARGE_STRING,params=[])

    def get_shop_amount(self, shop, item):
        """
        find the number of shop items
        :param shop: the specific shop
        :param item: the specific item
        :return: Int: the amount of the specific item in the specific shop
        """
        for shop_item in shop.items:
            if (shop_item.name == item):
                return shop_item.amount
        return 0

    def unregister(self, terminate_services=True):
        super(ItemActionBehavior, self).unregister(terminate_services=terminate_services)
        self._pub_generic_action.unregister()
        self._pub_finished_task.unregister()
        self._pub_failed_task.unregister()
        self._sub_task.unregister()
        self._sub_shop.unregister()
        self._sub_agent.unregister()


class AbstractFacilitySensor(PassThroughTopicSensor):
    """
    A base class for all sensor implementations that are selecting a facility from a topic based on a reference topic (other facility or agent..)
    """
    __metaclass__ = ABCMeta

    def __init__(self, topic, ref_topic, name=None, message_type=None, initial_value=None, facility_attribute=None,
                 create_log=False, print_updates=False):
        """
        synch function handles the distance value and return a float
        :param topic: see :class:PassThroughTopicSensor
        :param ref_topic: Reference topic that is used to select the considered facility
        :param name: see :class:PassThroughTopicSensor
        :param message_type: see :class:PassThroughTopicSensor
        :param initial_value: see :class:PassThroughTopicSensor
        :param facility_attribute: An attribute that should be taken as the sensor value from the finally selected facility e.g. Position, Rate, ...
                if None the raw/entire facitliy is passed
        :param create_log: see :class:PassThroughTopicSensor
        :param print_updates: see :class:PassThroughTopicSensor
        """
        self._facilities = {}
        super(AbstractFacilitySensor, self).__init__(name=name, topic=topic, message_type=message_type,
                                                     initial_value=initial_value, create_log=create_log,
                                                     print_updates=print_updates)
        self._facility_attribute = facility_attribute

        self._latest_ref_value = None

        ref_message_type = get_topic_type(ref_topic)
        if ref_message_type is not None:
            self._sub_ref = rospy.Subscriber(ref_topic, ref_message_type, self.subscription_callback_ref_topic)
        else:
            rospy.logerr("Could not determine message type of: " + topic)

    def subscription_callback_ref_topic(self, msg):
        self._latest_ref_value = msg

    def update(self, new_value):

        for facility in new_value.facilities:
            self._facilities[facility.name] = facility

        super(AbstractFacilitySensor, self).update(newValue=new_value)

    def sync(self):

        reduced_facility_value = self._reduce_facility(facilities=self._facilities, ref_value=self._latest_ref_value)

        if self._facility_attribute:
            self._value = getattr(reduced_facility_value, self._facility_attribute)
        else:
            self._value = reduced_facility_value
        return self._value

    @abstractmethod
    def _reduce_facility(self, facilities, ref_value):
        """
        This method has to be implemented in order to select a facility msg from all received messages based on the
        reference value
        :param facilities: dict with sensor msgs, key is the facility name
        :param ref_value: the current reference msg
        :return: the selected facility
        """
        raise NotImplementedError()

    def __del__(self):
        super(AbstractFacilitySensor, self).__del__()
        self._sub_ref.unregister()


class ClosestFacilitySensor(AbstractFacilitySensor):
    """
    Facility sensor that determines the closest facility to the reference
    """

    def __init__(self, topic, ref_topic, name=None, message_type=None, initial_value=None, facility_attribute=None,
                 create_log=False, print_updates=False):
        super(ClosestFacilitySensor, self).__init__(name=name, topic=topic, ref_topic=ref_topic,
                                                    message_type=message_type,
                                                    facility_attribute=facility_attribute, initial_value=initial_value,
                                                    create_log=create_log,
                                                    print_updates=print_updates)
        self._closest_facility = None

    @property
    def closest_facility(self):
        return self._closest_facility

    def _reduce_facility(self, facilities, ref_value):
        """
        Determining the closest facility by euclidean distance
        :param facilities: dict of facilities
        :param ref_value: reference value for position comparison
        :return: closest facility
        """

        if ref_value and self._closest_facility:
            min_distance = euclidean_distance(ref_value.pos, self._closest_facility.pos)
        else:
            min_distance = sys.float_info.max

        if ref_value:
            for _, facility in facilities.iteritems():
                distance = euclidean_distance(ref_value.pos, facility.pos)
                if distance < min_distance:
                    min_distance = distance
                    self._closest_facility = facility

        return self._closest_facility


class ClosestFacilityDistanceSensor(ClosestFacilitySensor):
    """
    Facility sensor that determines the closest distance of a facility to the reference
    """

    def __init__(self, topic, ref_topic, name=None, message_type=None, initial_value=None, facility_attribute=None,
                 create_log=False, print_updates=False):
        super(ClosestFacilityDistanceSensor, self).__init__(name=name, topic=topic, ref_topic=ref_topic,
                                                            message_type=message_type,
                                                            facility_attribute=facility_attribute,
                                                            initial_value=initial_value, create_log=create_log,
                                                            print_updates=print_updates)

    def _reduce_facility(self, facilities, ref_value):
        """
        Overwrite the method for returning the distance to the closest facility instead of the facility itself
        :param facilities:
        :param ref_value:
        :return:
        """
        closest_facility = super(ClosestFacilityDistanceSensor, self)._reduce_facility(facilities, ref_value)

        if closest_facility:

            distance = euclidean_distance(ref_value.pos, closest_facility.pos)
        else:
            distance = sys.float_info.max

        return distance

def string_to_position(string):
    """
    turns a string of a Position to the Position object
    :param string: e.g.:'lat: 48.8607292175\nlong: 2.39379000664'
    :return: Position
    """
    pos = Position()
    pos.lat = float(string.split()[1])
    pos.long = float(string.split()[3])
    return pos


class DistanceToLocationSensor2(Sensor):
    """
    Location sensor that determines the closest distance of the location to the reference
    """

    def __init__(self, topic, ref_topic, agent_name, name=None, message_type=None, initial_value=None,
                 facility_attribute=None,
                 create_log=False, print_updates=False):

        super(DistanceToLocationSensor2, self).__init__(name=name)

        self._facility_attribute = facility_attribute
        self.agent_name = agent_name
        self._facilities = {}
        self._latest_ref_value = None
        self._target = None

        self.target_is_facility = True

        self._target_name = None
        self.is_resource_node = False
        # subscriptions to several topics
        self._sub_ref0 = rospy.Subscriber(topic, Task, self.subscription_callback_topic)
        self._sub_ref1 = rospy.Subscriber(ref_topic, Agent, self.subscription_callback_ref_topic)
        self._sub_ref2 = rospy.Subscriber("/shop", ShopMsg, self.subscription_callback_facility)
        self._sub_ref3 = rospy.Subscriber("/storage", StorageMsg, self.subscription_callback_facility)
        self._sub_ref4 = rospy.Subscriber("/workshop", WorkshopMsg, self.subscription_callback_facility)

    def subscription_callback_ref_topic(self, msg):
        self._latest_ref_value = msg

    def subscription_callback_topic(self, msg):
        if (msg.agent == self.agent_name):
            if(msg.action == GATHER_STRING):
                self.is_resource_node=True
            else:
                self.is_resource_node = False
                try:
                    self._target = self._facilities[msg.facility]
                except KeyError:
                    rospy.logdebug('Facility %s not yet available.', msg.facility)
            self._target_name = msg.facility

            try:
                self._target = self._facilities[msg.facility]
            except KeyError:
                rospy.logdebug('Facility %s not yet available.', msg.facility)

    def subscription_callback_facility(self, msg):
        for facility in msg.facilities:
            self._facilities[facility.name] = facility
        if not self._target:
            try:
                self._target = self._facilities[self._target_name]
            except KeyError:
                # might be possible here depending on the topic
                pass

    def update(self, new_value):
        return
        # if (new_value.agent == self.agent_name):
            # self._target = self._facilities[new_value.facility]
            # self._facilities = new_value

            # super(DistanceToLocationSensor2, self).update(newValue=new_value)

    def sync(self):
        distance = float("Inf")
        if(self.is_resource_node):
            pos = string_to_position(self._target_name)
            distance = euclidean_distance(pos, self._latest_ref_value.pos)
        else:
            if (self._target):
                distance = euclidean_distance(self._target.pos, self._latest_ref_value.pos)

        super(DistanceToLocationSensor2, self).update(distance)
        super(DistanceToLocationSensor2, self).sync()
        return distance
