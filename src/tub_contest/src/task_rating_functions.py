from __future__ import division # force floating point division when using plain /
from fault_tolerance_functions import *
from agent_helper_functions import *
from shop_helper_functions import *
from list_of_numbers import *
from mac_ros_bridge.msg import Position
from agent_modules_tub_contest import get_item_amount



def rate_task(self, task):
    """
    A function to rate a specific task.
    First: if an agent doesn't have enough capacity, he will receive a low
    rating for the certain task.
    If he has enough capacity he will be integrated into the rating process:
    For GET_STRING-tasks:
    For every shop the agents are going to check their rating in regard to their distance.
    (Here, we don't specify a maximum price for a needed item.
    This will later be addressed in further optimizing updates. That's the reason for "Inf" as the
    price condition.) 
    Agents which do have enough free space, will have a much higher rating than the ones, which don't
    have enough space. By this approach we ensure that we have efficient operations
    by the agents.
    For Assemble_String-task:
    Again we focus on the distance, the closer the agent is to a workshop, the higher his rating will be.
    If the agent doesn't have the needed tools, he will be assigned with a low rating.
    :param task: the task which needs to get rated 
    :return: 
    """
    number_of_existing_items = get_amount_of_item_of_agent(self, task.item)
    # check if the agent has the tool already
    if (task.item[0] == "t") & (number_of_existing_items == 1) and (task.action == GET_STRING):
        task.rating = 0
        task.action = BUY_STRING
        return task

    #first check if we might already have the item
    has_amount = get_item_amount(self._this_agent, task.item)

    # assemble tasks might end up in a deadlock because other tasks like assiting are not canceled (yet)
    #TODO it would be better if assembled items could be reused as well
    if has_amount >= task.amount and task.action != ASSEMBLE_STRING:
    #if has_amount >= task.amount:
        # check if the items are needed for currently assigned tasks
        failed_task_amount = get_failed_task_item_amount(self, task)

        rated_task_amount = get_rated_task_item_amount(self, task)

        assigned_amount = new_get_assigned_item_amount(self, task)

        rospy.logwarn("%s: group %s:already has %s. Task amount %d, His amount %d, Rated task amount %d, failed task amount %d, assigned amount %d",
                      self._agent_name, self._agent_group, task.item, task.amount, has_amount, rated_task_amount,
                      failed_task_amount, assigned_amount)

        available_amount = has_amount + assigned_amount - rated_task_amount # plus because giving items as in deliver return a negative number

        if available_amount >= task.amount:
            rospy.logwarn("%s: reusing existing items for %s for %s", self._agent_name, task.task_id, task.item)
            self._my_rated_reused_tasks[task.task_id] = task
            task.action = USE_EXISTING_ITEMS
            task.rating = 0
            return task

    # check if the agent has the required capacity. if not give the task a maximum value
    if (not agent_has_exact_capacity(self, task)):
        if (task.action == GET_STRING):
            task.facility = DEFAULT_SHOP
            if(item_is_in_resource_node(self,task.item)) and (task.amount <= GATHER_RESOURCE_AMOUNT_THRESHOLD):
                task.action = GATHER_STRING
            else:
                task.action = BUY_STRING
            task.rating = MAX_VALUE
        else:
            task.facility = task.best_workshop
            task.rating = MAX_VALUE
        return task

    if (task.action == GET_STRING):
        # if this agent can not use this tool
        if (task.item[0] == "t")and (not tool_is_possible(self, task.item)):
            task.rating = MAX_VALUE
            task.action = BUY_STRING
            task.facility = DEFAULT_SHOP
            return task

        # so the rating is not 0 for other action
        task.rating = 1

        # only if the needed amount is smaller than the threshold then the items can be gathered
        if(task.amount <= GATHER_RESOURCE_AMOUNT_THRESHOLD):
            gather_task = rate_gather_task(self,task)
            if(gather_task is not None):
                return gather_task
        task = rate_buy_task(self,task)

    elif (task.action == ASSEMBLE_STRING):
        task = rate_assemble_task(self, task)

    return task


def get_assigned_item_amount(self, task):
    assigned_amount = 0
    for assigned_task in self._my_failed_tasks:
        if task.item == assigned_task.item:
            assigned_amount += assigned_task.amount
    return assigned_amount

def new_get_assigned_item_amount(self, task):
    assigned_amount = 0
    # calculate how many amounts of the given item would be needed for the alredy assigned tasks
    # e.g. if only deliver_job(amount=3) for the given item is assigned -> return -3
    for assigned_task in self._my_assigned_tasks:
        if task.item == assigned_task.item:
            if(is_get_action(self, task.action)):
                assigned_amount += assigned_task.amount
            else:
                assigned_amount -= assigned_task.amount
    return assigned_amount

def get_failed_task_item_amount(self, task):
    failed_task_amount = 0
    # check if the available amount comes from old failed tasks
    for failed_task in self._my_failed_tasks:
        if task.item == failed_task.item and (
                                failed_task.action == GET_STRING or
                                failed_task.action == GATHER_STRING or
                            failed_task.action == DELIVER_JOB_STRING or
                        failed_task.action == ASSEMBLE_STRING
        ):
            failed_task_amount += failed_task.amount
    return failed_task_amount

def get_rated_task_item_amount(self, task):
    rated_task_amount = 0
    # check if the available amount comes from old failed tasks
    for _,rated_task in self._my_rated_reused_tasks.iteritems():
        if task.item == rated_task.item and (
                                rated_task.action == GET_STRING or
                                rated_task.action == GATHER_STRING or
                            rated_task.action == DELIVER_JOB_STRING or
                        rated_task.action == ASSEMBLE_STRING
        ):
            rated_task_amount += rated_task.amount
    return rated_task_amount

def rate_buy_task(self,task):
    """
    rates a assembling task
    :param self: 
    :param task: 
    :return: Task
    """
    # init variables
    best_shop = self._shops.get(DEFAULT_SHOP)
    best_rating = MAX_VALUE
    
    # check if perception is already available
    if best_shop and self._workshops.get(task.best_workshop):
    
        # get a rating for all shops
        for k, shop in self._shops.items():
            # if shop does not have the item or has not enough items than use the max price
            # might be problematic if the all shops have currently not enough items
            if (get_item_amount_in_shop(self, task.item, shop) >= task.amount):# TODO check if nesseccary
                rating = get_total_steps_for_task(self, task, shop.name, task.best_workshop) * PENALTY_FOR_STEPS
                rating += get_agent_specific_penalty(self, task)
            else:
                rating = MAX_VALUE
            if (best_rating > rating):
                best_shop = shop
                best_rating = rating
    
        # assign best shop and rating
        task.facility = best_shop.name
        task.action = BUY_STRING
        task.rating += best_rating
    else:
        task.facility = DEFAULT_SHOP
        task.action = BUY_STRING
        task.rating = MAX_VALUE
        rospy.logerr("Something unexpected happended, perception not yet available!")
        
    return task

def rate_assemble_task(self,task):
    """
    rates a assembling task
    :param self: 
    :param task: 
    :return: Task
    """

    storage_of_item = get_storage_of_item(self, task.item,task.amount)
    if(storage_of_item is not None):

        print("get item of storage",task.item,self._agent_name)

        task.facility= storage_of_item.name

        if (task.part_of):
            dest = storage_of_item.name
        else:
            dest = task.destination
        task.action = RETRIEVE_STRING
        task.rating += get_total_steps_for_task(self, task, storage_of_item.name, dest) * PENALTY_FOR_STEPS
        task.rating += get_agent_specific_penalty(self, task)

    else:
        task.facility = task.best_workshop

        missing_tools = get_missing_tools(self, task.required_tools)

        this_pos = self._workshops.get(task.best_workshop).pos

        #task.rating += (len(self._my_assigned_tasks) * PENALTY_OPEN_TASKS)
        #task.rating += (len(missing_tools) * PENALTY_MISSING_TOOL)

        # if the item is a part of another item then the destination is a workshop. otherwise it is the storage for the job
        if (task.part_of):
            dest = task.best_workshop
        else:
            dest = task.destination

        task.rating += get_total_steps_for_task(self, task, task.best_workshop, dest) * PENALTY_FOR_STEPS
        task.rating += get_agent_specific_penalty(self, task)
    return task


def rate_gather_task(self,task):
    """
    rates a gather task
    :param self: 
    :param task: 
    :return: 
    """
    # init variables
    best_shop = self._shops.get(DEFAULT_SHOP)
    best_rating = MAX_VALUE
    best_res = None

    # if the item can be found in a resource node than it should be gathered there.
    resource_node_list = get_item_as_resource(self, task.item)
    for res in resource_node_list:
        rating = get_total_steps_for_task(self, task, str(res.pos), task.best_workshop) * PENALTY_FOR_STEPS
        rating += get_agent_specific_penalty(self, task)
        # this_pos = self._workshops.get(DEFAULT_WORKSHOP).pos
        # rating += int(
        #    number_of_required_steps_to_facility(self._role, shop.pos, this_pos)) * PENALTY_FOR_STEPS
        #rating += (len(self._my_assigned_tasks) * PENALTY_OPEN_TASKS)
        if (best_rating > rating):
            best_res = res
            best_rating = rating
    if (best_rating < MAX_VALUE):
        task.facility = str(best_res.pos)
        task.action = GATHER_STRING
        task.rating += best_rating
        return task
    return None


def task_rating(self):
    # TODO agents need to update their future perception so they do not buy at a place without items
    """
    A function that rates one task after another first locally and then publish its rating. 
    After receiving all values the best one is choosen and the task_assignment will be called
    :param tasks[]: the list of open tasks
    :return: 
    """

    if len(self.open_tasks) == 0:
        return False
    else:
        task = self.open_tasks[0]
        task = rate_task(self, task)
        # in case a task from another agent was received and used
        task.agent = self._agent_name
        self._pub_neg.publish(task)


def get_agent_specific_penalty(self,task):
    """
    calculate the penalty of a this agent for taking a specific task
    e.g. only one drone should buy tools
    :param self: 
    :param action: 
    :return: 
    """
    # TODO here could be some penalties included
    if(task.action == "buy") and (task.item[0] == "t") and (self._agent_name == self._default_agent_name):
        return 00000
    return 0

def get_total_steps_for_task(self, task, task_facility, task_destination):
    """
    computes how many assigned steps the agent needs if he would take the given task.
    If the agent would already be at the target of the task because of other tasks a value of 1 will be returned.
    :param self: class
    :param task: the negotiated task
    :return: Integer: the total amount of steps
    """
    # TODO include a computation or estimation of the required steps for charging
    # include the time for the actions. gather actions take longer
    steps = 0
    last_pos = self._this_agent.pos
    task_pos = get_position_of_facility(self, task_facility)
    dest_pos = get_position_of_facility(self, task_destination)
    step_function = self._path_planner.number_of_required_steps_to_facility
    has_task_included = False

    # calculate steps for the current task
    if(self._current_task):
        # get the facility of the current task
        current_task_pos = get_position_of_facility(self, self._current_task.facility)
        # check if the current task comes before of after the negotiated task
        if (self._current_task.task_depth <= task.task_depth):
            # calculate the required steps

            steps += step_function(last_pos, current_task_pos)
            #print("calc1",last_pos,"to",current_task_pos,steps)
        else:
            # if the task does not require moving return 1
            if (last_pos == task_pos):
                return 1
            # calculate the required steps including the task
            steps += step_function(last_pos, task_pos)
            steps += step_function(task_pos, dest_pos)
            steps += step_function(dest_pos, current_task_pos)
            #print("calc2", last_pos, "to", current_task_pos,steps)
            has_task_included = True

        last_pos = current_task_pos
    # iterate over all assigned tasks
    for assigned_task in self._my_assigned_tasks:

        # get the facility of the task
        assigned_task_pos=get_position_of_facility(self, assigned_task.facility)

        # check if the assigned task comes before of after the negotiated task
        if(assigned_task.task_depth <= task.task_depth) or (has_task_included):
            # calculate the required steps
            steps += step_function(last_pos,assigned_task_pos)
            #print("calc3", last_pos, "to", current_task_pos, steps)
        else:
            # if the task does not require moving return 1
            if (last_pos == task_pos):
                return 1
            # calculate the required steps including the task
            steps += step_function(last_pos, task_pos)
            steps += step_function(task_pos, dest_pos)
            steps += step_function(dest_pos, assigned_task_pos)
            #print("calc4", last_pos, "to", current_task_pos, steps)
            has_task_included = True

        # save the position of the assigned task
        last_pos = assigned_task_pos

    # if the task would be done after all already assigned tasks
    if(not has_task_included):
        # if the task does not require moving return 1
        if(last_pos==task_pos):
            return 1
        # calculate the required steps including the task
        steps += step_function(last_pos, task_pos)
        steps += step_function(task_pos, dest_pos)
        has_task_included = True
        #print("calc5", last_pos, "to", current_task_pos, steps)
    # return the result
    return steps

def get_position_of_facility(self, facility):
    """
    delivers the position of a given facility
    :param self: class
    :param facility: String: name of the facility
    :return: Pos : position of the facility
    """

    # if the facility is a workshop
    if(self._workshops.get(facility)):
        return self._workshops.get(facility).pos

    # if the facility is a shop
    if (self._shops.get(facility)):
        return self._shops.get(facility).pos

    # if the facility is a storage
    if (self.storages.get(facility)):
        return self.storages.get(facility).pos
    # if the facility is a resource node already encoded as a string
    if (facility[0:3] == "lat"):
        return string_to_position(facility)

    print("++++++++++++++++++++++++++++++++++","nothing found")
    # if none of the above use position of the agent instead
    return self._this_agent.pos


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


def check_if_job_is_feasible(self, job):
    """
    A function to check whether a job can be fulfilled in time, by comparing it with our feasability_threshold. 
    :param job: the job which needs to be assessed if our agents can fulfill in time       
    :return: bool TRUE if agent has the item and the amount of the given item
    """
    if ((job.end - self._current_step) <= JOB_FEASIBILITY_THRESHOLD) or (self._current_step > self._job_too_old_step_threshold):
        return False
    else:
        return True

