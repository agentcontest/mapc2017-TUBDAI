from __future__ import division # force floating point division when using plain /
from list_of_strings import *
from list_of_numbers import *
import rospy

def get_amount_of_item_of_agent(self, item):

    """
    A function that returns how much of a certain item i an agent has in possession.
    :param item: the item to check for.
    :param self: object instance
    :return: bool True if the tool is owned by the role of the agent
    """
    for this_item in self._this_agent.items:
        if (this_item.name == item):
            return this_item.amount
    return 0


def better_agent_has_capacity(self, item, amount):
    """
    Note: Not in use yet
    looks through the agent assigned capacity and identifies if it is possible to buy also the specified item and amount 
    :param self: object instance
    :param item: returns items for a specific msg with item info
    :param amount: returns the amount of items (for example, an agent needs to buy at a shop)
    :return: 
    """
    free_capacity = self._role.max_load - self._this_agent.load
    needed_capacity = self.products.get(item).volume * amount
    assigned_capacity = 0
    if (self._current_task):
        if (self._current_task.action == BUY_STRING) and (self.products.get(self._current_task.item)):
            assigned_capacity += self._current_task.amount * self.products.get(self._current_task.item).volume
    for task in self._my_assigned_tasks:
        if (task.task_depth <= task.task_depth):  # TODO might be problematic if there is a conflict assemble
            if (task.action == BUY_STRING) and (self.products.get(task.item)):
                assigned_capacity += task.amount * self.products.get(task.item).volume
            if (task.action == RETRIEVE_STRING) and (self.products.get(task.item)):
                assigned_capacity += task.amount * self.products.get(task.item).volume
            if (task.action == ASSEMBLE_STRING) and (self.products.get(task.item)):
                assigned_capacity += task.amount * self.products.get(task.item).volume
            if (task.action == DELIVER_JOB_STRING) and (self.products.get(task.item)):
                assigned_capacity -= task.amount * self.products.get(task.item).volume
            if (task.action == ASSIST_ASSEMBLE_STRING) and (self.products.get(task.item)):
                assigned_capacity -= task.amount * self.products.get(task.item).volume
    if (free_capacity >= (needed_capacity + assigned_capacity)):  # Incorrect check of free_capacity
        return True
    return False

def calculate_new_capacity(self, capacity, task):
    """
    calculates for a given task what would be the new capacity after executing the task
    :param self: 
    :param task: 
    :return: 
    """
    if(task.action == ASSIST_ASSEMBLE_STRING)and(task.item[0] == "t"):
        return capacity
    if(is_get_action(self, task.action)):
        return capacity + (task.amount * self.products.get(task.item).volume)
    else:
        return capacity - (task.amount * self.products.get(task.item).volume)

def is_get_action(self,action):
    """
    returns True if the given action is a get action. False otherwise
    :param self: 
    :param action: String
    :return: 
    """
    if(action == BUY_STRING) or (action == GATHER_STRING) \
            or (action == ASSEMBLE_STRING) or (action == GET_STRING) or (action == RETRIEVE_STRING):
        return True
    return False

def agent_has_exact_capacity(self, task):
    max = self._role.max_load
    capacity= self._this_agent.load
    has_task_included = False

#    print(capacity, "  ", task.item, task.amount, task.action)
    # calculate steps for the current task
    if (self._current_task):
        # check if the current task comes before of after the negotiated task
        if (self._current_task.task_depth <= task.task_depth):
            # calculate the required capacity
            capacity = calculate_new_capacity(self,capacity,self._current_task)
            #print(capacity, "  ", task.item, task.amount, task.action,self._current_task.item,self._current_task.amount,"curr")
        else:
            capacity = calculate_new_capacity(self,capacity,task)
            #print(capacity, "  ", task.item, task.amount, task.action, task.item, task.amount,"curr2")
            # check if the new capacity would be over the max value
            if(capacity>max):
                return False
            capacity = calculate_new_capacity(self, capacity, self._current_task)
            #print(capacity, "  ", task.item, task.amount, task.action, self._current_task.item, self._current_task.amount,"curr3")
            has_task_included = True
        if (capacity > max):
            return False
    # iterate over all assigned tasks
    for assigned_task in self._my_assigned_tasks:

        # check if the assigned task comes before of after the negotiated task
        if (assigned_task.task_depth <= task.task_depth) or (has_task_included):
            # calculate the new capacity
            capacity = calculate_new_capacity(self, capacity, assigned_task)
            #print(capacity, "  ", task.item, task.amount, task.action, assigned_task.item, assigned_task.amount, "curr4")
        else:
            has_task_included = True
            capacity = calculate_new_capacity(self, capacity, task)
            #print(capacity, "  ", task.item, task.amount, task.action, task.item, task.amount, "curr5")
            #  check if the new capacity would be over the max value
            if (capacity > max):
                return False
            capacity = calculate_new_capacity(self, capacity, assigned_task)
            #print(capacity, "  ", task.item, task.amount, task.action, assigned_task.item, assigned_task.amount, "curr6")
            has_task_included = True

        if (capacity > max):
            return False

    # if the task would be done after all already assigned tasks
    if (not has_task_included):
        capacity = calculate_new_capacity(self, capacity, task)
        #print(capacity, "  ", task.item, task.amount, task.action, task.item, task.amount, "curr7")
        has_task_included = True
    if (capacity > max):
        return False
    # return the result
    #print("++++++++++++++++++++++++++++++++++++++++++++++")
    return True

def agent_has_capacity(self, item, amount):  # agent helper
    """
    looks through the agent assigned capacity and identifies if it is possible to buy also the specified item and amount 
    :param item: the item 
    :param amount: the amount of the given item
    :return: bool True if he has enough space
    """

    if (item[0] == "t") and (agent_has_tool(self, item)):
        return True

    if not self._role or not self._this_agent: #we don't know yet as the perception is not available
        return False

    free_capacity = self._role.max_load - self._this_agent.load

    needed_capacity = self.products.get(item).volume * amount
    assigned_capacity = 0
    if (self._current_task):
        if (self._current_task.action == BUY_STRING) and (self.products.get(self._current_task.item)):
            assigned_capacity += self._current_task.amount * self.products.get(self._current_task.item).volume
        if (self._current_task.action == ASSEMBLE_STRING) and (self.products.get(self._current_task.item)):
            assigned_capacity += self._current_task.amount * self.products.get(self._current_task.item).volume
    for task in self._my_assigned_tasks:
        if (task.action == BUY_STRING) and (self.products.get(task.item)):
            assigned_capacity += task.amount * self.products.get(task.item).volume
        if (task.action == ASSEMBLE_STRING) and (self.products.get(task.item)):
            assigned_capacity += task.amount * self.products.get(task.item).volume
    if (free_capacity >= (needed_capacity + assigned_capacity)):  # Incorrect check of free_capacity
        return True
    return False


def agent_has_tools(self, tools):  # agent_hepler
    """
    A function to check whether an agent has certain tools (for job execution)
    :param self: object instance
    :param tools: the agent will be checked if he has the tools in his possession 
    :return: bool True if he has the tools
    """

    if (self._this_agent is None):
        return False
    for tool in tools:
        has_tool = False
        for this_tool in self._this_agent.items:
            if (this_tool.name == tool):
                has_tool = True
        if (not has_tool):
            return False
    return True


def agent_has_tool(self, tool):
    """
    A function to check whether an agent has one specific tools (for job execution)
    :param self: object instance
    :param tool: the agent will be checked if he has the tool in his possession 
    :return: bool True if he has the tool
    """

    has_tool = False
    if self._this_agent:
        for this_tool in self._this_agent.items:
            if (this_tool.name == tool):
                has_tool = True
    return has_tool


def get_missing_tools(self, tools):
    """
    A function that returns if an agent has a specific tool. Used in "all_tools_possible()".
    :param self: object instance
    :param tools: the list of tools
    :return: all needed tools which are still needed (for a job)
    """
    missing_tools = []
    if (self._this_agent is None):
        return False
    for tool in tools:
        has_tool = False
        for this_tool in self._this_agent.items:
            if (this_tool.name == tool):
                has_tool = True
        if (not has_tool):
            missing_tools.append(tool)
    return missing_tools


def tool_is_possible(self, tool):
    """
    A function that returns if an agent has a specific tool. Used in "all_tools_possible()".
    :param self: object instance
    :param tool: the list of tools
    :return: bool True if the tool is owned by the role of the agent
    """
    for possible_tool in self._role.tools:
        if (possible_tool.name == tool):
            return True
    return False


def get_amount_of_item(self, item_name):  # agent_helper_functions
    """
    this is a helper function which returns the amount of item that this agent carries
    :param self: object instance
    :param item_name: the name of specific item
    :return: the amount of that item
    """

    for item in self._this_agent.items:
        if (item.name == item_name):
            return item.amount
    return 0


# deprecated
def all_tools_possible(self, tools):
    """
    A function that returns if an agent has the needed tools for assembling an item
    :param self: object instance
    :param tools: the list of tools
    :return: bool True  if all tools are owned by the agent
    """
    for tool in tools:
        if (not self.tool_is_possible(tool)):
            return False
    return True


def agent_has_item_and_amount(self, item, amount, bigger):
    """
    this is a helper function which returns if the agent already has the 
    amount of the item in his possession
    :param self: object instance
    :param item: the needed item 
    :param amount: the needed amount of the  item
    :param bigger: returns if amount of items an agent has is higher then needed amount (boolean)
    :return: bool TRUE if agent has the item and the amount of the given item
    """

    this_amount = get_amount_of_item(self, item)
    # if (self._current_task.action == Assist_Assemble_String):
    if (bigger):
        if (this_amount >= amount):
            return True
    else:
        if (this_amount <= amount):
            # if(self._current_task.action==Assist_Assemble_String):
            #    print("check true", self._agent_name, self.should_state, this_amount)
            return True
    return False
