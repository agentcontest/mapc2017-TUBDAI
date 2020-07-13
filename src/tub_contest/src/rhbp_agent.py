#!/usr/bin/env python2
# this is the main file of a rhbp agent instance
# it contains the agents variables, topics subscriber and publisher, as well as the regarding callback function
# Although the job_execution process is start in this file
from __future__ import division # force floating point division when using plain /
from agent_modules_tub_contest import *
from behaviour_components.activators import BooleanActivator, Condition, ThresholdActivator
from behaviour_components.conditions import Negation
from behaviour_components.managers import Manager
from behaviour_components.sensors import SimpleTopicSensor
from mac_ros_bridge.msg import RequestAction, SimStart, SimEnd, Bye, GenericAction
from task_assignment_functions import *
from job_rating_functions import *
from task_rating_functions import *
from job_splitting_functions import *
from list_of_numbers import *
from list_of_strings import *
from graphhopper import PathPlanner
from statistics import Statistics
from threading import Event
import threading
from random import randint

from diagnostic_msgs.msg import KeyValue

import re
import rospy

class RhbpAgent:
    def __init__(self):
        rospy.logdebug("RhbpAgent::init")

        self._agent_name = rospy.get_param('~agent_name', 'TUBDAI1')  # default for debugging 'TUBDAI1'

        self._agent_topic_prefix = get_bridge_topic_prefix(agent_name=self._agent_name)

        # Identify agent group
        group_info = self.get_agent_group_info(self._agent_name)
        self._agent_group = group_info[2]
        # identify the agent number in a team
        self._agentNumber = group_info[1]
        #the default agent is the group leader, that is also responsible for selecting a job
        self._default_agent_name = group_info[0] + group_info[2] #the first agent in the group corresponds with the group number
        self._is_default_agent = (self._default_agent_name == self._agent_name)

        self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1)
        # ensure also max_parallel_behaviours during debugging

        # a client for the Knowledge base for writing and reading
        self.client = KnowledgeBaseClient(knowledge_base_name=KNOWLEDGE_BASE_NAME)

        self._stats_match = Statistics(add_step_column=False)
        self._stats_agent = None

        self._path_planner = None

        # helps ensuring finished agent state reset before continuing with next init in a second simulation
        self._reset_event = Event()
        self._reset_init_agent_state()
        self._reset_event.set()

        self._init_topics_and_services()

    def _init_topics_and_services(self):
        """
        ROS topic subscriber/publisher and service initialisation
        """

        # publisher for communication with other agents depend on their group
        self._pub_task = rospy.Publisher('/task' + self._agent_group, Task, queue_size=NUMBER_OF_AGENTS, latch=True)
        self._pub_neg = rospy.Publisher('/negotiation' + self._agent_group, Task, queue_size=NUMBER_OF_AGENTS, latch=True)
        self._pub_finished_task = rospy.Publisher('/finished_task' + self._agent_group, Task, queue_size=NUMBER_OF_AGENTS, latch=True)
        self._pub_failed = rospy.Publisher('/failed_task' + self._agent_group, Task, queue_size=NUMBER_OF_AGENTS, latch=True)
        self._pub_job_failed = rospy.Publisher('/failed_job' + self._agent_group, Task, queue_size=NUMBER_OF_AGENTS, latch=True)
        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(self._agent_name) + 'generic_action',
                                                   GenericAction, queue_size=1)
        # subscribe to MAC bridge core simulation topics
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)
        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)
        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)
        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)
        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self._callback_generic_action)

        # group has been already considered in KB.
        rospy.Subscriber('/negotiation' + self._agent_group, Task, self.task_assignment_callback)
        rospy.Subscriber('/finished_task' + self._agent_group, Task, self._finished_task_callback)
        rospy.Subscriber('/failed_task' + self._agent_group, Task, self._failed_task_callback)
        rospy.Subscriber('/failed_job' + self._agent_group, Task, self._failed_job_callback)


        #new topics for distributing the same job to every agent
        if self._is_default_agent:
            self._pub_job_taken = rospy.Publisher('/job_taken'+ self._agent_group, TakenJob, queue_size=NUMBER_OF_AGENTS, latch=True)
            self._pub_bidding_taken = rospy.Publisher('/bidding_taken'+ self._agent_group, BiddingTaken, queue_size=NUMBER_OF_AGENTS, latch=True)
            for other_group_id in range(1, NUMBER_OF_GROUPS+1):
                if str(other_group_id) != str(self._agent_group):
                    rospy.Subscriber('/job_taken' + str(other_group_id), TakenJob, self._others_have_job_taken_callback)
                    rospy.Subscriber('/bidding_taken' + str(other_group_id), BiddingTaken, self._others_have_bidding_taken_callback)

        else:
            rospy.Subscriber('/job_taken'+ self._agent_group, TakenJob, self._job_taken_callback)

    def update_perception(self,msg):
        """
        updates all relevant information coming from the Request_Action message
        :param msg: Request_Action
        :return:
        """
        for shop in msg.shops:
            self._shops[shop.name]=shop
        for workshop in msg.workshops:
            self._workshops[workshop.name]=workshop
        for resource in msg.resources:
            self.resource_nodes[resource.name]=resource
        for storage in msg.storages:
            self.storages[storage.name]=storage
        self._this_agent=msg.agent
        if self._stats_agent:
            self._stats_agent.add_value('charge', self._this_agent.charge)
            self._stats_agent.add_value('load', self._this_agent.load)
            self._stats_agent.add_value('last_action', self._this_agent.last_action)
            self._stats_agent.add_value('last_action_result', self._this_agent.last_action_result)
            self._stats_agent.add_value('item_cnt', len(self._this_agent.items))
        for mission in msg.mission_jobs:
            self._mission_jobs[mission.id] = mission
        for auction in msg.auction_jobs:
            self._auction_jobs[auction.job.id] = auction
        for priced in msg.priced_jobs:
            self._priced_jobs[priced.id] = priced

    def _reset_init_agent_state(self):

        """
        Initialising/Resetting all internal agent state variables as required in the beginning of each simulation
        """
        # -----------------------------
        # scenario independent variables
        self._task_lock = threading.Lock()
        self._job_lock = threading.Lock()
        # variable which saves if the sim start message has already received
        self._sim_started = False
        # monitoring if the last action request was answered
        self._received_action_response = False
        # list for all instantiated behaviours in order to simplify destruction
        if hasattr(self,'_behaviours'): #unregister former behaviour instances
            for b in self._behaviours:
                b.unregister()
        self._behaviours = []

        # -----------------------------
        # scenario dependent variables
        self._sim_id = ''
        # list of job_items
        self._job_items = []
        # dictionary which contains the name of the shops as keys and the shops as values
        self._shops = {}
        # dictionary with workshop names as key an the workshops as values
        self._workshops = {}
        # counter for step iteration
        self._step_counter = 0
        # counter used within task ids
        self._task_id_cnt = 0
        # the received mesages from the other agents containing their rated tasks
        self._received_messages = []
        # dictionary with product name as key and list of consumed items as value
        self.products_info = {}
        # dictionary with product name as key and product object as value
        self.products = {}
        # refer to all tools needed to be assembeled
        self._sim_start_info = None

        # list of all accepted jobs
        self._accepted_jobs = []
        # jobs other groups have taken over
        self._other_accepted_jobs = []
        # auctions other groups have taken over
        self._other_biddings = []
        # my active auction
        self._my_auction = ""
        # list of tasks for this agent
        self._my_assigned_tasks = []
        self._my_rated_reused_tasks = {}
        # completed tasks
        self._my_completed_tasks = []
        # failed tasks
        self._my_failed_tasks = []
        # status if the last action had a failed capacity
        self.last_capacity_failed = False
        # if amount should be bigger or equal the number of should state
        self.should_state_bigger = False
        # refers to all posted jobs with job_id as the key
        self._priced_jobs = {}
        # refers to all mission jobs with the job_id as key
        self._mission_jobs = {}
        # refers to all auction jobs with the job_id as key
        self._auction_jobs = {}
        self._won_auction_jobs = []

        # refers to all open jobs. may include expired ones
        # includes the open tasks of the current negotiated job
        self.open_tasks = []
        # list of waiting tasks in case the sending mechanism of open tasks is not in order
        self.waiting_tasks = []
        # the current task of the agent
        self._current_task = None
        # stores the current step
        self._current_step = None
        # an integer which says how many items the agent should have after running his current action
        self.should_state = None
        # contains the Agent object of this agent
        self._this_agent = None
        # saves the current perception
        self.request_action = None
        # saves the step when the last time a task was assigned to this agent
        self.last_assigned_step = 0
        # a dictionary containing for each item what the average price is
        self.item_average_price = {}
        # keeps track what each agent is currently doing (if he has a task or not)
        self.list_of_agent_statuses = {}
        # counter how many round the agents did not have a successful action
        self.rounds_without_action = 0
        # the number of steps how long the current task is running
        self.task_duration = 0
        # a counter of how many jobs have been taken
        self.job_counter = 0
        # counter for how many task have been taken
        self.task_counter = 0

        # dictionary for storing known resource nodes
        self.resource_nodes = {}

        # dictionary for storing known storages
        self.storages = {}

        self.waiting_dic = {}
        self.received_dic = {}

    def _others_have_job_taken_callback(self, taken_job):
        """
        callback for accepted jobs of other groups
        :param taken_job: TakenJob msg
        """
        if taken_job.job.id in self._accepted_jobs:
            rospy.logerr("Job conflict between groups for job %s", taken_job.job.id)
            kill_all_tasks_of_job_id(self, taken_job.job.id, reason=' conflict')

        self._other_accepted_jobs.append(taken_job.job.id)

    def _others_have_bidding_taken_callback(self, taken_bidding):
        """
        callback for bidding taken by other groups
        :param taken_bidding: BiddingTaken msg
        """
        rospy.logwarn("Bidding conflict between groups in auction %s", taken_bidding.job_id)
        if not taken_bidding.job_id in self._other_biddings:
            self._other_biddings.append(taken_bidding.job_id)

    def _job_taken_callback(self, taken_job):
        """
        called if a job was taken
        :param taken_job: TakenJob msg
        """
        if taken_job and self._sim_started:
            if self.products_info:
                with self._job_lock:
                    rospy.logdebug("%s:: bestjob with id %s in step %d", self._agent_name, "bestjob", taken_job.job.id, self._current_step)

                    self._accepted_jobs.append(taken_job.job.id)
                    self.open_tasks.extend(taken_job.tasks)

                    # let the open tasks be rated
                    task_rating(self)
            else:
                rospy.logwarn('Could not take job, perception not yet completely available!')

    def get_agent_group_info(self, agent_name):
        """
        Extract name information from the agent name, like agent_number, group number and base name string
        :param agent_name: the name of the agent
        :return: Str tuple: agent_base_name, agent_number, number of the group
        """
        #TODO nabil here you can change if neccessary the assignment rule
        agent_number = int(re.findall('\d+$', agent_name )[0]) #this rule extracts a multiple digit number at the end of the string

        group_number = str(((agent_number-1) % NUMBER_OF_GROUPS) + 1) # id-1 % groups+1 to start group numbers with 1

        agent_base_name = ''.join(agent_name.rsplit(str(agent_number), 1)) # delete first occurence of agent number in agent name

        return agent_base_name, str(agent_number), group_number

    def _finished_task_callback(self, msg):
        """
        if a message regarding a finished assembling is received,
        check if this agent was currently assisting for this assembling and can get a new task
        :param msg: Task
        """
        if msg.sim_id != self._sim_id:
            rospy.logwarn("_assemble_callback %s!=%s", msg.sim_id, self._sim_id)
            return
        with self._task_lock:
            if (self._current_task and self._sim_started):
                if (self._current_task.action == ASSIST_ASSEMBLE_STRING) and (self._current_task.part_of == msg.item) \
                        and (self._current_task.parent_task_id==msg.task_id) and msg.action == ASSEMBLE_STRING:
                    print("task fullfilled after msg", self._agent_name, self._current_task.action, self._current_task.item,
                          self._current_step)

                    delete_fulfilled_assist_assemble(self)
                    self._current_task = None
                    get_new_task(self)
                elif self._current_task.action == msg.action and self._current_task.task_id == msg.task_id:
                    print("task fullfilled after msg", self._agent_name, self._current_task.action, self._current_task.item,
                          self._current_step)

                    self._current_task = None
                    get_new_task(self)

    def _failed_job_callback(self, msg):
        """
        if an impossible task was received from another agent all tasks with respect to the job_id will be deleted
        :param msg: Task
        """
        if msg.sim_id != self._sim_id:
            rospy.logwarn("_failed_job_callback %s!=%s", msg.sim_id, self._sim_id)
            return
        print("received failed job",self._agent_name,msg.job_id)
        kill_all_tasks_of_job_id(self, msg.job_id, reason=' failed')

    def _failed_task_callback(self, msg):
        """
        if a failed task was received from another agent this function will be invoked
        :param msg: Task
        """
        if msg.sim_id != self._sim_id:
            rospy.logwarn("_failed_task_callback %s!=%s", msg.sim_id, self._sim_id)
            return
        failed_task_mechanism(self, msg)

    def _sim_start_callback(self, msg):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """
        #wait until reset/init is complete

        rospy.loginfo("%s: Received SimStart", self._agent_name)

        self._reset_event.wait() #TODO check if a timeout is necessary here

        if not self._sim_started:  # init only once here

            self._analyze_start_info(msg)

            self._path_planner = PathPlanner(role=self._role, cell_size=msg.cell_size, proximity=self._proximity)

            self._stats_agent = Statistics()

            rospy.logdebug('Setting map to %s', msg.map)
            self._path_planner.set_map(map=msg.map)

            #---------------------------
            # Init behaviours

            agent_recharge_upper_bound = self._role.max_battery

            agent_topic = self._agent_topic_prefix + "agent"

            # write has no task into knowledge base
            publish_has_task_in_knowledge(self, "false")

            #rospy.loginfo(self._agent_name + ": GotoLocationBehaviour")

            # go to the location specified in the current task and published via a topic
            # if no required charging and not at the location already
            goto_location = GotoLocationBehaviour(plannerPrefix=self._agent_name,
                                                        agent_name=self._agent_name, sim_id=self._sim_id, name='goto_location',
                                                        task_topic='/task'+ self._agent_group)
            self._behaviours.append(goto_location)


            # has_task_sensor = KnowledgeSensor(name='has_task',
            #                                  pattern=get_knowledge_base_tuple_has_task(
            #                                      self._agent_name, self._agent_group,'task') + ("true",))

            # has_task_condition = Condition(has_task_sensor, BooleanActivator(desiredValue=True))

            # Condition to check if we are close to (in) a charging station
            at_charge_sensor = ClosestFacilityDistanceSensor(name='at_charge', topic='/charging_station',
                                                             ref_topic=agent_topic)
            # true if the agent is at a charging station
            at_charge_cond = Condition(at_charge_sensor,
                                       ThresholdActivator(thresholdValue=self._proximity,
                                                          isMinimum=False))  # highest activation if the value is below threshold

            # Condition to check if we are close to (in) the location specified in the current task
            at_location_sensor = DistanceToLocationSensor2(name='at_location', topic='/task' + self._agent_group,
                                                           agent_name=self._agent_name, ref_topic=agent_topic)

            at_location_cond = Condition(at_location_sensor,
                                         ThresholdActivator(thresholdValue=self._proximity,
                                                            isMinimum=False))

            goto_location.addPrecondition(Negation(at_location_cond))
            goto_location.addPrecondition(Negation(at_charge_cond))

            #rospy.loginfo(self._agent_name + ": ItemActionBehavior")

            # do the action specified in the current task if at the location
            item_action = ItemActionBehavior(agent_group = self._agent_group, plannerPrefix=self._agent_name,
                                             path_planner=self._path_planner, agent_name=self._agent_name,
                                             name='item_action', sim_id=self._sim_id, agent_topic=agent_topic)

            self._behaviours.append(item_action)

            item_action.addPrecondition(at_location_cond)

            #rospy.loginfo(self._agent_name + ": Recharge behaviour")

            # recharge, but only if critical battery
            recharge = GenericActionBehaviour(plannerPrefix=self._agent_name,
                                                    agent_name=self._agent_name, name=RECHARGE_STRING,
                                                    action_type=RECHARGE_STRING)
            self._behaviours.append(recharge)

            charge_sensor = SimpleTopicSensor(topic=agent_topic, name="charge_sensor", message_attr=CHARGE_STRING)

            # true if the agent needs to recharge, because he has not enough battery to move
            low_battery = Condition(charge_sensor,
                                    ThresholdActivator(thresholdValue=CRITICAL_CHARGING,
                                                       isMinimum=False))
            # highest activation if the value is below threshold

            recharge.addPrecondition(low_battery)
            goto_location.addPrecondition(Negation(low_battery))

            #rospy.loginfo(self._agent_name + ": GotoChargingBehaviour behaviour")

            # goes to a charging station if the agent needs to charge and is not already at one
            goto_charge = GotoChargingBehaviour(plannerPrefix=self._agent_name,
                                                      agent_name=self._agent_name,
                                                      name='goto_charge',
                                                      facility_topic='/charging_station', ref_topic=agent_topic
                                                      )
            self._behaviours.append(goto_charge)
            
            # if the agents charge is below a specified percentage of its full battery than he needs to charge
            required_charge = Condition(charge_sensor,
                                        ThresholdActivator(
                                            thresholdValue=CHARGING_PERCENTAGE * agent_recharge_upper_bound,
                                            isMinimum=False))
            # highest activation if the value is below threshold

            goto_charge.addPrecondition(required_charge)
            goto_charge.addPrecondition(Negation(at_charge_cond))
            goto_charge.addPrecondition(Negation(low_battery))

            #rospy.loginfo(self._agent_name + ": Charge behaviour")

            # charge action if the agent is at a charging station
            charge = GenericActionBehaviour(plannerPrefix=self._agent_name,
                                                  agent_name=self._agent_name, name=CHARGE_STRING,
                                                  action_type=CHARGE_STRING)
            self._behaviours.append(charge)

            # true if the agent has a full battery
            full_battery = Condition(charge_sensor,
                                     ThresholdActivator(thresholdValue=agent_recharge_upper_bound,
                                                        isMinimum=True))
            # highest activation if the value is below threshold

            charge.addPrecondition(at_charge_cond)
            charge.addPrecondition(Negation(full_battery))

            #rospy.loginfo(self._agent_name + ": GotoLocationBehaviour behaviour")

            # if the agent has a full battery and is at a charging station than he goes to the location specified in the current task
            # needed so the agents will always charge until they have full battery
            goto_location_if_full_battery = GotoLocationBehaviour(plannerPrefix=self._agent_name,
                                                                        agent_name=self._agent_name, sim_id=self._sim_id,
                                                                        name='goto_location_full_battery',
                                                                        task_topic='/task' + self._agent_group)
            self._behaviours.append(goto_location_if_full_battery)

            goto_location_if_full_battery.addPrecondition(at_charge_cond)
            goto_location_if_full_battery.addPrecondition(full_battery)
            goto_location.addPrecondition(Negation(required_charge))

            self._stats_match.add_value("completed_jobs", 0)

            # set flag after initialization is completed
            self._sim_started = True

            rospy.loginfo("%s: Agent '%s' started in group %s. Simulation id: %s", self._agent_name, self._role.name, self._agent_group, self._sim_id)

    def _analyze_start_info(self, msg):
        """
        Analyze and extract useful information from the sim start message
        :param msg: sim start message
        :type msg: SimStart
        """
        self._sim_id = msg.simulation_id
        # save the sim start message
        self._sim_start_info = msg
        # save the role of the agent
        self._role = msg.role
        # save the tools that the agent can carry
        self._assemble_tools = msg.tools
        # proximity value
        self._proximity = msg.proximity
        # job too old step threshold
        self._job_too_old_step_threshold = msg.steps * JOB_TOO_OLD_THRESHOLD

        # save the existing items and how they need to be assembled
        for product in msg.products:
            self.products[product.name] = product
            self.products_info[product.name] = product.consumed_items

    def job_execution_process(self):
        """
            Starts the complex job and task assignment process in each agent at the end (or beginning?!?) of each round
            :param
            :return:
        """

        last_results_tuple = get_knowledge_base_tuple_has_task(agent_name=self._agent_name, group=self._agent_group, has_task="last")

        # do not do anything in the first round or the round after the agent has aborted all tasks
        if (self._current_step == 0) or (self.client.exists(last_results_tuple + ("abort",))):
            self.client.update(last_results_tuple + ('*',), last_results_tuple + ("failed",))
            return

        with self._task_lock:
            # check what the results of the last action was
            check_last_action(self)

            # check if the task is fulfilled
            check_if_task_fulfilled(self)

            # delete old/expired jobs
            check_for_old_job(self)

            # check if no agent is moving for a few rounds
            is_nothing_working(self)

            # TODO here we might need to add more checks for a correct start
            # better to do it before the job rating as it also uses shop infos.
            if (self._this_agent is None) and (len(self._workshops)) == 0 and (len(self._shops) == 0):
                rospy.logwarn("%s: Skipping job rating not yet received all perception", self._agent_name)
                return

            # get the best job and only done by one agent for synchronization
            if self._is_default_agent:

                if ENABLE_AUCTIONS:
                    busy_with_auctions, won_auction_job = self.job_bidding_function()
                    rospy.logdebug("MISSION JOBS: " + str(self._mission_jobs.keys()))
                    rospy.logdebug("ACCEPTED JOBS: " + str(self._accepted_jobs))
                else:
                    busy_with_auctions = False
                    won_auction_job       = None

                if won_auction_job:
                    job = won_auction_job
                else:
                    enable_priced_jobs = not busy_with_auctions and ENABLE_PRICED_JOBS
                    job = job_rating(self, with_priced_jobs=enable_priced_jobs)

                if(job):
                    job_tasks = job_splitting(self, job)
                    takenjob = TakenJob(job=job, tasks=job_tasks)
                    if job_is_for_my_group(self, job):
                        self._pub_job_taken.publish(takenjob)
                        self._stats_match.increment_current_value("number_jobs", 1)
                        self.open_tasks.extend(job_tasks)
                        # debug outputs
                        rospy.loginfo("%s:: group: %s, bestjob: \n--------------\n%s\n---------------\n", self._agent_name,
                                      self._agent_group, str(job))
                        #self.print_verbose_job_debug(job_tasks)# let the open tasks be rated
                        task_rating(self)
                        # print(self._agent_name, "There are ", len(self.open_tasks), " open tasks")
                        self._stats_match.increment_current_value("number_tasks", len(self.open_tasks))
                else:
                    rospy.logdebug('Did not find a new job.')


    def print_verbose_job_debug(self, job_tasks):
        # rospy.loginfo("Assembly info: \n%s", str(self.products))
        rospy.logwarn("%s:: bestjob tasks:\n----------------------------\n", self._agent_name)
        for t in job_tasks:
            intent = int(t.task_depth * 2)
            intent_str = '{:>' + str(intent) + '}'
            print(intent_str.format('++++++++++++\n'))
            intent_str_data = '{:>' + str(intent) + '} {:<12} {:<12}\n'
            print(intent_str_data.format(t.action, t.item, t.amount))
            print(intent_str_data.format(t.part_of, t.destination, t.further_action))
            print(intent_str_data.format(t.task_id, t.parent_task_id, t.task_depth))


    def job_bidding_function(self):
        '''
        :return: tuple(True if the agent is busy with some auction bidding, won Job if any)
        '''

        if not self._auction_jobs:
            rospy.logdebug("NO _AUCTION_JOBS " + self._agent_name)
            return False, None
        else:
            rospy.logdebug("YES _AUCTION_JOBS " + self._agent_name)

        processed_auctions = [] # list of processed auction_ids # TODO not yet used

        #TODO no more than on simultaneous auction by one agent

        for auc_id, auc in self._auction_jobs.iteritems():

            auction_time = auc.job.start + auc.auction_time

            rospy.logdebug("Auction: " + auc.job.id +
                         " Agent: " + self._agent_name +
                         " lowest_bid: " + str(auc.lowest_bid) +
                         " current_step: " + str(self._current_step) +
                         " auction_time: " + str(auction_time))

            if auction_time > self._current_step:

                #other group is already bidding
                if auc.job.id in self._other_biddings:
                    rospy.logdebug("\tbidding is done be other agent: " + auc.job.id)
                    processed_auctions.append(auc_id)
                    continue

                # no own auction
                if self._my_auction == "":

                    #feasible?
                    if not check_if_job_is_feasible(self, auc.job):
                        rospy.logdebug("\tauction job not feasible: " + auc.job.id + " " + self._agent_name)
                        processed_auctions.append(auc_id)
                        continue

                    #team free?
                    free_agents_cnt = get_number_of_free_agents(self)
                    if free_agents_cnt < (NUMBER_OF_AGENTS * PERCENTAGE_OF_FREE_AGENTS):
                        rospy.logdebug("\t\tNOT ENOUGH AGENTS in GROUP: " + self._agent_name + " FREE AGENTS: " + str(free_agents_cnt))
                        processed_auctions.append(auc_id)
                        continue

                    #TODO check special item needed (tool) and available

                    #own mission jobs already?
                    num_mission_jobs = len([job for job in self._mission_jobs if job not in self._other_accepted_jobs])
                    if num_mission_jobs > 0:
                        rospy.loginfo("\t\tALREADY MISSION JOB: " + self._agent_name + " NUMBER MISSION JOBS: " + str(num_mission_jobs))
                        processed_auctions.append(auc_id)
                        continue

                    #profitable?
                    item_costs = self.get_avg_job_item_costs(auc)
                    #rospy.loginfo("\t\tITEM COSTS: " + str(item_costs) + " REWARD: " + str(auc.job.reward) + " PROFIT: " + str(auc.job.reward - item_costs))

                    #check if other default agent was faster
                    if auc.job.id in self._other_biddings:
                        rospy.loginfo("\tbidding is done be other agent: " + auc.job.id)
                        processed_auctions.append(auc_id)
                        continue

                    biddingTaken = BiddingTaken(job_id=auc.job.id)
                    self._pub_bidding_taken.publish(biddingTaken)
                    self._my_auction = auc.job.id
                    rospy.loginfo("\tBIDDING TAKEN: " + auc.job.id + " " + self._agent_name)

                    return True, None # we try to bid, do not take another job in the meanwhile

                # my auction?
                elif self._my_auction != auc.job.id:
                    rospy.loginfo("\tBIDDING IN OTHER AUCTION: " + self._my_auction + " THIS AUCTION: " + auc.job.id + " " + " " + self._agent_name)
                    processed_auctions.append(auc_id)
                    continue

                elif auction_time - 1 == self._current_step:
                    # Do the bidding
                    action = GenericAction()
                    action.action_type = BID_FOR_JOB_STRING

                    cost = self.get_avg_job_item_costs(auc)
                    cost = cost + int(cost * BIDDING_REVENUE_OFFSET)

                    if auc.lowest_bid == 0:
                        bid = self._get_bid(cost, auc.job.reward)
                    else:
                        bid = self._get_bid(cost, auc.lowest_bid)

                    rospy.logwarn("Cost: %d, Reward: %d, Bid: %d ,Lowest bid: %d ", cost, auc.job.reward, bid, auc.lowest_bid)

                    action.params = [KeyValue("Job", auc.job.id), KeyValue("Bid", str(bid))]
                    self._pub_generic_action.publish(action)
                    rospy.logwarn("%s: group %s Bidding on %s",self._agent_name, self._agent_group, auc.job.id)
                    return True, None
            else:
                if not auc.job.id in self._won_auction_jobs:
                    self._stats_match.increment_current_value("won_auction_jobs", 1)
                    if auc.job.id == self._my_auction:
                        self._my_auction = ""
                        rospy.logerr("%s: group %s GOT THE AUCTION %s",self._agent_name, self._agent_group, auc.job.id)
                        self._won_auction_jobs.append(auc.job.id)
                        return True, auc.job
                    else:
                        continue
                else:
                    rospy.logdebug("\tJOB ALREADY IN JOB LIST: " + auc.job.id)
                    continue
            processed_auctions.append(auc_id)

        return False, None

        # for a in processed_auctions:
        #     auc = self._auction_jobs.pop(a)

            # if (self._auction_jobs.auction_time):
        #     auction_job = self._auction_jobs.job
        #
        #     auction_job_max_bid = self._auction_jobs.max_bid
        #     auction_job_lowest_bid = self._auction_jobs.lowest_bid
        #     auction_job_auction_time = self._auction_jobs.auction_time
        #
        #     self._auction_job_info.append(auction_job_max_bid)
        #     self._auction_job_info.append(auction_job_lowest_bid,)
        #     self._auction_job_info.append(auction_job_auction_time)
        #
        #     print(auction_job_max_bid, auction_job_lowest_bid, auction_job_auction_time)
        #     #self._auction_jobs_list.append(auction_job)
        #
        # #print(auction_job)
        # if (auction_job):   #self.current_task is None
        #     print("I am here")
        #     bid_task = create_bid_task(self, BID_FOR_JOB_STRING, None, None, auction_job)
        #     print("The bid task!")
        #     print(bid_task)
        #     print("The assigned bid_task")
        #     self._my_assigned_tasks.append(bid_task)
        #     print("splitting auction job")
        #
        # return bid_task

    def _get_bid(self, min, max):
        if min > max:
            return min

        bid = max - randint(2, 100)
        if bid > min:
            return bid
        else:
            return max

    def get_avg_job_item_costs(self, job):
        item_costs = 0
        tasks = []
        for item in job.job.items:
            tasks.extend(split_item(self, item=item, job=job.job, save_tasks=False))
        for task in tasks:
            if (task.action == GET_STRING):
                item_costs += get_average_item_price(self, task.item) * task.amount
        return item_costs

    def task_assignment_callback(self, msg):

        """
        Will be called if all task rating are called and if this agent has best value then he manages the task
        Calls, if still open tasks, again the task_rating.
        :param msg : Task
        """
        if msg.sim_id != self._sim_id:
            rospy.logwarn("task_assignment_callback %s!=%s", msg.sim_id, self._sim_id)
            return

        if not self._sim_started:
            return

        # first find for the current open task the received messages and save into the received messages
        # and save the other messages for further tasks into a waiting list.
        # If the current open task changes then the regarding tasks will be taken from the waiting list
        #rospy.loginfo("%s: start %d da %g", self._agent_name, len(self.received_dic),NUMBER_OF_AGENTS)

        with self._task_lock:

            if(self.received_dic.get(msg.agent) is None):
                self.received_dic[msg.agent] = msg
            else:
                self.waiting_dic[msg.agent] = msg

            if(len(self.received_dic)==NUMBER_OF_AGENTS):
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
                    reassign_task(self, min_task)
                    task_rating(self)
                    return
                #print("mintask",self._agent_name,min_task.agent,min_task.rating,min_task.action,min_task.item,min_task.amount)
                # if this agent has the best rating let him execute it

                #todo save mintask

                if (min_task and min_task.agent == self._agent_name):
                    assign_task(self, min_task)

                if (len(self.open_tasks) > 0):
                    self.open_tasks.pop(0)

                # let other open tasks be rated and assigned
                task_rating(self)


    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        self._reset_event.clear()
        rospy.loginfo("Sim finished:" + str(msg))

        self.finalise_agents_stats(msg.timestamp)

        # Collect match statistics
        # only for the agentA1 as the match statistic is the same for every agent
        if(self._is_default_agent):
            # Export match statistics
            self._write_match_stats(msg)

        rospy.loginfo('%s: Resetting agent state.', self._agent_name)
        self._reset_init_agent_state()  # should be last call in this callback.
        self._reset_event.set()

    def finalise_agents_stats(self, timestamp):
        """
        Finalise and export agent statistics
        :param timestamp:
        """
        if not self._stats_agent:
            return
        self._stats_agent.finalise()
        #rospy.loginfo("Agent Stats '" + self._agent_name + "':\n" + str(self._stats_agent.get_descriptive_stats()))
        time = timestamp
        filename = self._agent_name + '/agent_stat_' + str(time)
        self._stats_agent.write_to_csv(filename=filename)
        self._stats_agent.write_to_plot(filename=filename)

    def _write_match_stats(self, msg):
        """
        this function adds the current values of the match to the statistics class
        and exports them to csv
        """
        if not self._stats_match:
            return
        # if the agent still has a current task, then there is one expired task more
        if(self._current_task):
            self._stats_match.increment_current_value("expired_jobs", 1)

        self._stats_match.add_value('time', msg.timestamp)
        self._stats_match.add_value('ranking', msg.ranking)
        self._stats_match.add_value('score', msg.score)
        self._stats_match.add_value('number_agents', NUMBER_OF_AGENTS)
        self._stats_match.add_value('number_steps', self._sim_start_info.steps)
        self._stats_match.add_value('nw_th', NOTHING_WORKED_THRESHOLD)
        self._stats_match.add_value('jf_th', JOB_FEASIBILITY_THRESHOLD)
        self._stats_match.add_value('last_job', self._job_too_old_step_threshold)
        self._stats_match.add_value('sim_id', self._sim_id)
        self._stats_match.add_value("map", self._sim_start_info.map)
        self._stats_match.add_value("free_agents_percentage", PERCENTAGE_OF_FREE_AGENTS)
        self._stats_match.add_value("groups", NUMBER_OF_GROUPS)
        self._stats_match.increment_current_value("nothing_worked", 0)
        self._stats_match.increment_current_value("expired_jobs", 0)
        self._stats_match.increment_current_value("unsolvable_jobs", 0)
        self._stats_match.increment_current_value("job_mission", 0)
        self._stats_match.increment_current_value("job_priced", 0)
        self._stats_match.increment_current_value("job_auction", 0)
        self._stats_match.increment_current_value("resource_th", GATHER_RESOURCE_AMOUNT_THRESHOLD)


        self._stats_match.finalise()

        # Export match statistics
        filename = self._agent_name + '/match_stats'
        self._stats_match.write_to_csv(filename=filename, append=True)
        # Reset statistics here, because we appended already to file
        self._stats_match = Statistics(add_step_column=False)

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        rospy.loginfo("Simulation finished")
        rospy.signal_shutdown('Shutting down {}  - Simulation server closed'.format(self._agent_name))

    def _callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        self._received_action_response = True

    def _idle_action(self):
        """
        Trigger a suitable idle action on the mac_ros_bridge
        """
        action_generic_simple(publisher=self._pub_generic_action, action_type=RECHARGE_STRING)

    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and planning
        :param msg: the message
        :type msg: RequestAction
        """
        self.request_action = msg

        # calculate deadline
        start_time = rospy.get_rostime()
        safety_offset = rospy.Duration.from_sec(ACTION_REQUEST_SAFETY_OFFSET) # seconds
        deadline_msg = rospy.Time.from_sec(msg.deadline/1000.0)
        current_msg =rospy.Time.from_sec(msg.timestamp/1000.0)
        deadline = start_time + (deadline_msg-current_msg) - safety_offset

        self.request_action = msg

        if self._is_default_agent:
            rospy.loginfo('#################Step%d###############', msg.simulation_step)

        self.update_perception(msg)

        self._received_action_response = False
        self._current_step = msg.simulation_step

        executed_job_process = False

        # self._received_action_response is set to True if a generic action response was received (send by any behaviour)
        while not self._received_action_response and rospy.get_rostime() < deadline:
            # wait until this agent is completely initialised
            if self._sim_started: #we at least wait our max time to get our agent initialised
                if not executed_job_process:
                    self._stats_agent.update_step(step=self._current_step)
                    # complex process at the beginning of each step to determine which jobs to take and which agent will
                    # do the tasks this function will give each agent a list of tasks. Agents post their current task (only one!)
                    # in the topic /task other not posted task should be in a list. once a task is complete the agent
                    # will post the next task of list in the topic as long as the agent has a task he should note that in

                    self.job_execution_process()
                    executed_job_process = True #only do this once a step
                # action send is finally triggered by a selected behaviour
                self._manager.step()
            else:
                rospy.sleep(0.1)

        if self._received_action_response:
            duration = rospy.get_rostime() - start_time
            rospy.logdebug("%s: Decision-making duration %f", self._agent_name, duration.to_sec())
        elif not self._sim_started:
            rospy.logwarn("%s idle_action(): sim not yet started in step %d", self._agent_name, msg.simulation_step)
            self._idle_action()
            return
        else:
            rospy.logwarn("%s: Decision-making timeout", self._agent_name)
            self._idle_action()

if __name__ == '__main__':
    try:
        rospy.init_node('agent_node', anonymous=True)
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
