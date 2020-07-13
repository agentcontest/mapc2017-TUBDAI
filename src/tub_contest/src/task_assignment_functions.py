from __future__ import division # force floating point division when using plain /
from fault_tolerance_functions import *
from agent_helper_functions import *
from knowledge_functions import *
from mac_ros_bridge.msg import *
from tub_contest.msg import *
from list_of_numbers import *


def get_new_task_id(self, job_id):
    task_id = 'task_'+job_id + '_' + str(self._task_id_cnt)
    self._task_id_cnt += 1
    return task_id


def make_copy_of_task(self, task):
    """
    A function used in the process to split tasks. Copies a certain task.
    :param self: object instance
    :param task:the task object which needs to be copied
    :return: return a copy of the given task
    """
    new_task = Task()
    new_task.required_tools = task.required_tools
    new_task.action = task.action
    new_task.amount = task.amount
    new_task.part_of = task.part_of
    new_task.further_action = task.further_action
    new_task.facility = task.facility
    new_task.job_id = task.job_id
    new_task.task_depth = task.task_depth
    new_task.parent_task_id = task.parent_task_id
    new_task.agent = task.agent
    new_task.item = task.item
    new_task.rating = task.rating
    new_task.best_workshop = task.best_workshop
    new_task.sim_id = task.sim_id
    new_task.task_id = get_new_task_id(self, task.job_id)
    return new_task


def reassign_task(self, task):
    """
    This function is used to split tasks into 2 smaller tasks by dividing the amount of the needed item by the half.
    Hint: If the number (amount) is uneven, e.g. amount = 7, then we split to task with amount = 3 and amount = 4.
    :param self: object instance
    :param task: the task which needs to reassigned
    :return: bool True if the tool is owned by the role of the agent
    """
    #

    if self._is_default_agent:
        print("Split task ", self._agent_name, task.action, task.item, task.amount, len(self.open_tasks))
    new_list = []
    task1 = make_copy_of_task(self, task)
    task1.amount = int(task.amount / 2)
    task2 = make_copy_of_task(self, task)
    task2.amount = int(task.amount / 2)

    if (task1.amount + task2.amount != task.amount):
        task1.amount += 1
    # neccessary for the task rating
    if (task1.action == BUY_STRING) or (task1.action == GATHER_STRING):
        task1.action = GET_STRING
    if (task2.action == BUY_STRING) or (task2.action == GATHER_STRING):
        task2.action = GET_STRING

    # append the new tasks to the list
    new_list.append(task1)
    if (task2.amount > 0):
        new_list.append(task2)
    if (len(self.open_tasks) > 0):
        self.open_tasks.pop(0)

    new_list.extend(self.open_tasks)
    self.open_tasks = new_list[:]

    if (task2.amount == 0):
        self._stats_match.increment_current_value("unsolvable_jobs", 1)
        print(task.job_id, "Not solvable")
        # self.job_has_failed_menchanism(task)
        kill_all_tasks_of_job_id(self, task.job_id, reason=' not solvable')
        return


def assign_task(self, task):
    """
    A function to assign a task at any given moment to an agent and by doing so,
    also enables to "update" the agent with a new task.
    Prints the new task before assigning the new task.
    :param self: object instance
    :param task: the new, to-be-given task for an agent
    :return:
    """
    print("-----------------------------------------------------------------------")
    print("assigned task", self._agent_name, task.action, task.item, task.amount, task.facility,task.task_depth, task.task_id, task.parent_task_id, task.further_action, self._current_step, task.job_id)
    # split tasks into smaller tasks
    # e.g.: get item task
    self.last_assigned_step = self._current_step

    new_task = Task()
    new_task.task_id = get_new_task_id(self, task.job_id)
    new_task.sim_id = self._sim_id
    new_task.amount = task.amount
    new_task.parent_task_id = task.parent_task_id # the new further tasks are getting the same parent to correctly determine the agent the have to assist
    new_task.task_depth = task.task_depth + 0.5
    new_task.job_id = task.job_id
    new_task.item = task.item
    new_task.agent = self._agent_name

    if task.further_action == DELIVER_JOB_STRING:
        new_task.action = task.further_action
        new_task.facility = task.destination
        self._my_assigned_tasks.append(new_task)
    elif task.further_action == ASSIST_ASSEMBLE_STRING:
        task.destination = task.best_workshop
        new_task.action = task.further_action
        new_task.facility = task.best_workshop
        new_task.part_of = task.part_of
        self._my_assigned_tasks.append(new_task)
    else:
        rospy.logerr('Unexpected further action: %s', task.further_action)

    # only add tasks that needs to be done (e.g. if the agent has already the tool than the rating is -1)
    if task.rating > 0:
        self._my_assigned_tasks.append(task)
    else:
        print(self._agent_name, task.action, task.item, "already task fulfilled")

    get_new_task(self)


def set_should_state(self, task):
    """
    sets the right amount of items that the agent should have after finishing the specified task
    :param self: object instance
    :param task: the task to consider
    :return:
    """
    if (not task):
        return
    if (task.action == BUY_STRING) or (task.action == GATHER_STRING):
        self.should_state = get_amount_of_item(self, task.item) + task.amount
        self.should_state_bigger = True
    if (task.action == ASSEMBLE_STRING)or task.action == RETRIEVE_STRING:
        self.should_state = get_amount_of_item(self, task.item) + task.amount
        self.should_state_bigger = True
    if (task.action == ASSIST_ASSEMBLE_STRING):
        # Assist assemble should only be checked via received msg from other agent calling for assistants to assemble
        self.should_state = MAX_VALUE
        self.should_state_bigger = True
    if (task.action == DELIVER_JOB_STRING):
        self.should_state = get_amount_of_item(self, task.item) - task.amount
        self.should_state_bigger = False

def get_minimum_task(self):
    """
    a helper function to return the lowest (best) task to do for an agent
    :param self: object instance
    :return:
    """
    min_value = MAX_VALUE
    min_task = None
    for task in self._my_assigned_tasks:
        if (task.task_depth < min_value):
            min_value = task.task_depth
            min_task = task
    if (self._current_task):
        if self._current_task.task_depth <= min_value:
            min_task = self._current_task
    return min_task


def get_conflict_free_task(self):
    """
    If an agent has a task to assist assemble item x and to assemble item x he would normally 
    try to do the assisting first, because of a lower ranking of this task. As this would result in a deadlock, because
    he would wait for himself, the assisting will be deleted
    
    :param self: object instance
    :return:
    """
    for task in self._my_assigned_tasks:
        # print("conflict",task.task_id,self._current_task.task_id+0.5,task.action,self._current_task.action,task.item,self._current_task.part_of)
        if (task.task_id == self._current_task.parent_task_id) and (task.action == ASSEMBLE_STRING) and (
                    task.item == self._current_task.part_of) and (self._current_task.action == ASSIST_ASSEMBLE_STRING):
            #rospy.logdebug("%s::conflict detected for task:\n%s\n------\n%s", self._agent_name, str(self._current_task), str(task))
            self._my_assigned_tasks.remove(task)
            # delete also further tasks with the same depth as the assisting
            for further_task in self._my_assigned_tasks[:]:
                if further_task.parent_task_id == self._current_task.parent_task_id:
                    self._my_assigned_tasks.remove(further_task)
            return task
    return self._current_task


def get_new_task(self):
    """
    :param self: object instance
    should be called if this agent has finished his task and want to fullfill a new one from his schedule
    :return:
    """

    self.task_duration = 0
    if (len(self._my_assigned_tasks) == 0):
        self._current_task = None
        publish_has_task_in_knowledge(self, "false")
        self.should_state = None
        return

    if (self._current_task):
        self._my_assigned_tasks.append(self._current_task)

    self._current_task = get_minimum_task(self)

    self._my_assigned_tasks.remove(self._current_task)

    self._current_task = get_conflict_free_task(self)

    if (self._current_task is not None):
        publish_has_task_in_knowledge(self, "true")
        set_should_state(self, self._current_task)
        self._pub_task.publish(self._current_task)
    else:
        publish_has_task_in_knowledge(self, "false")

    if (self._current_task.action == BUY_STRING) and (self._current_task.item[0] == "t"):
        if (agent_has_tool(self, self._current_task.item)):
            print("tool already bought", self._agent_name, self._current_task.action, self._current_task.item,
                  self._current_task.facility, self._current_task.amount, self._current_task.part_of,
                  self._current_task.task_id)
            self._current_task = None
            get_new_task(self)
            return

    print("current_task", self._agent_name, self._current_task.action, self._current_task.item,
          self._current_task.facility, self._current_task.amount, self._current_task.part_of,
          self._current_task.task_id, self._current_task.job_id)


def check_if_task_fulfilled(self):
    """
    :param self: object instance
    :return: /
    """
    if (self._current_task)and(self.last_assigned_step):
        if (agent_has_item_and_amount(self, self._current_task.item, self.should_state, self.should_state_bigger))\
                and self._this_agent.last_action_result == "successful":
            # if the agent fulfills the task within 2 round after assignment, it is a bug and should be ignored
            if(self.last_assigned_step + 2 >=  self._current_step):
                print("failed check for task fulfilled",self._agent_name, self._current_task.action, self._current_task.item,
                  self._current_task.amount, get_amount_of_item_of_agent(self,self._current_task.item), self._current_step)
                #print("current",self._agent_name, self._current_task, self.should_state,
                #      get_amount_of_item_of_agent(self,self._current_task.item), self._this_agent)
                print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                return
            print("task fullfilled", self._agent_name, self._current_task.action, self._current_task.item,
                  self._current_task.amount, get_amount_of_item_of_agent(self,self._current_task.item), self._current_step)
            # send to other agents that this assembling is finished
            if (self._current_task.action == ASSEMBLE_STRING):
                self._pub_finished_task.publish(self._current_task)
            if (self._current_task.action == ASSIST_ASSEMBLE_STRING):
                delete_fulfilled_assist_assemble(self)

            self._current_task = None
            get_new_task(self)


def delete_fulfilled_assist_assemble(self):
    """
    called if a current assist assemble was fulfilled. then it will be checked if
    the agent has more assist_assemble that were though the one assist_assemble already fulfilled as well
    :param self: object instance
    :return:
    """
    for task in self._my_assigned_tasks[:]:
        if task.parent_task_id == self._current_task.parent_task_id:
            self._my_assigned_tasks.remove(task)
