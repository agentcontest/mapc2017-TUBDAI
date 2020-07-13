from agent_modules_tub_contest import get_knowledge_base_tuple_has_task
from list_of_strings import *


def write_last_action_into_knowledge(self):
    """
    A function to to update the last action into the "Knowledge Base".
    :param self: object instance
    :return:
    """
    last_knowledge = get_knowledge_base_tuple_has_task(agent_name=self._agent_name, group=self._agent_group,
                                                       has_task="last")
    if self._this_agent and self._this_agent.last_action_result:
        result = self._this_agent.last_action_result
        action = self._this_agent.last_action
        if (result == "successful") and (action == NO_ACTION_STRING):
            result = "failed"
        if (result == "successful") and (action == RECHARGE_STRING):
            result = "failed"
        if (result == "failed_item_amount") and (action == BUY_STRING):
            result = "successful"
        if (result == "partial_success") and (action == GATHER_STRING):
            result = "successful"
        if (result == "failed_facility_state") and (action == CHARGE_STRING):
            result = "successful"
    else:
        result = "successful"
    try:
        self.client.update(last_knowledge + ('*',), last_knowledge + (result,))
    except Exception as e:
        print(e.message, "Knowledge base has failed")


def get_number_of_successful_agents(self):
    """
    A function that returns whose last actions were successful.
    :param self: object instance
    :return: number of free agents
    """
    tuple = get_knowledge_base_tuple_has_task(agent_name="*", group=self._agent_group, has_task="last") + ("successful",)
    facts = self.client.all(pattern=tuple)

    return len(facts) if facts else 0


def get_number_of_free_agents(self):
    """
    A function to determine how many agents are not having a task to do in this step.
    :param self: object instance
    :return:
    """
    # Using new function over topics instead of knowledge base
    #return get_number_of_free_agents_with_statuses(self)

    # alternative new kb version
    tuple = get_knowledge_base_tuple_has_task(agent_name="*", group=self._agent_group, has_task="task") + ("false",)

    facts = self.client.all(pattern=tuple)

    return len(facts) if facts else 0


def publish_has_task_in_knowledge(self, bool):
    """
    Writes into the knowledge base if this agent has a task.
    :param self: object instance
    :param bool:True if this agent has a task and False if not
    :return:
    """
    try:
        self._task_knowledge = get_knowledge_base_tuple_has_task(agent_name=self._agent_name, group=self._agent_group, has_task="task")
        self.client.update(self._task_knowledge + ('*',), self._task_knowledge + (bool,))

    except Exception as e:
        print("Failed knowledge update!")
        print(e.args, e.message)

    return


def get_number_of_free_agents_with_statuses(self):
    """
    Return the number of agents with tasks.
    :param self: object instance
    :return: int free: the number if agents with tasks
    """
    free = 0

    for k, status in self.list_of_agent_statuses.items():

        if (status == "false"):
            free += 1

    return free
