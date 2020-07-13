from __future__ import division # force floating point division when using plain /
import task_assignment_functions
from knowledge_functions import write_last_action_into_knowledge, get_number_of_successful_agents, \
    publish_has_task_in_knowledge, get_number_of_free_agents_with_statuses
from task_assignment_functions import *
from task_rating_functions import *
from agent_modules_tub_contest import get_knowledge_base_tuple_has_task
from list_of_strings import *
from list_of_numbers import *


def is_nothing_working(self):
    """
    A function to resolve problems: If all agents are having problems and they're passing the
    "nothing_worked_threshold", then all actions/jobs/tasks by all agents will be disrupted.
    """

    write_last_action_into_knowledge(self)

    successful_agent_cnt = get_number_of_successful_agents(self)

    if self._is_default_agent and self.rounds_without_action > 0:
        rospy.logwarn("Successfull agents %d in step %d, rounds without action %d", successful_agent_cnt, self._current_step, self.rounds_without_action)

    if (successful_agent_cnt == 0):
        if (self.rounds_without_action >= NOTHING_WORKED_THRESHOLD) and (self._current_step < self._job_too_old_step_threshold):
            rospy.logwarn("%s:: group %s: nothing worked -> abort!, %d rounds without action, current step: %d", self._agent_name, self._agent_group, self.rounds_without_action, self._current_step)
            self.rounds_without_action = 0
            reset_agent_state(self)
        elif (len(self._accepted_jobs) > 0):
            self.rounds_without_action += 1
    else:
        self.rounds_without_action = 0


def reset_agent_state(self):
    """
    A function to reset all agents, by "deleting" all tasks.
    """
    if(get_number_of_free_agents_with_statuses(self) < NUMBER_OF_AGENTS):
        self._stats_match.increment_current_value("nothing_worked", 1)
    print(self._agent_name, "Resetting agent state")
    self.open_tasks = []
    self._my_assigned_tasks = []
    self._current_task = None

    for msg in self._received_messages[:]:
        if (msg.job_id in self._accepted_jobs):
            try:
                self._received_messages.remove(msg)
            except Exception as e:
                print(e.message)

    for msg in self.waiting_tasks[:]:
        if (msg.job_id in self._accepted_jobs):
            try:
                self.waiting_tasks.remove(msg)
            except Exception as e:
                print(e.message)

    last_knowledge = get_knowledge_base_tuple_has_task(agent_name=self._agent_name, group=self._agent_group,
                                                       has_task="last")
    self.client.update(last_knowledge + ('*',), last_knowledge + ("abort",))

    task_assignment_functions.get_new_task(self)


def check_last_action(self):
    """
    A function to check for the last action performed by the the agent
    Especially needed to maintain a certain robustness to early recognize problems and to react to problems.
    """
    if not self._this_agent:
        return
    if not self._current_task:
        return
    if (self._this_agent.last_action_result == "failed_capacity") and (self._this_agent.last_action == BUY_STRING) and (
            self.last_capacity_failed):
        current_task_has_failed(self, "failed_capacity")

    if (self._this_agent.last_action_result == "failed_capacity") and (self._this_agent.last_action == BUY_STRING):
        self.last_capacity_failed = True
    else:
        self.last_capacity_failed = False

    if (self._this_agent.last_action_result == "failed_job_status" or self._this_agent.last_action_result == "failed_unknown_job" ) and (self._this_agent.last_action == DELIVER_JOB_STRING):
        print("job status failed", self._agent_name, self._current_task.job_id)
        self._pub_job_failed.publish(self._current_task)


def current_task_has_failed(self, reason=""):
    """
    Invoked if the current task is not solvable anymore
    Send the the tasks to the other agents so it gets assigned again
    and gets a new task if there are open ones
    :return:
    """

    if (self._current_task):
        print("------------------------------------")
        print("The task failed", self._agent_name, self._current_task.action, self._current_task.item,
              self._current_task.amount, reason)
        print("------------------------------------")
    else:
        return

    if (self._current_task.action == BUY_STRING):
        self._pub_failed.publish(self._current_task)
    self._current_task = None
    task_assignment_functions.get_new_task(self)


def kill_all_tasks_of_job_id(self, id, reason=''):
    """
    Used if a job is not active anymore. then it kills all tasks with the assigned job
    Starts getting a new task if the old one was deleted
    :param id: the specific id of the job
    :type id: Job
    """
    if self._is_default_agent:
        print("kill tasks of job with id", id, self._agent_name, reason)
    for task in self.open_tasks[:]:
        if (task.job_id == id):
            self.open_tasks.remove(task)

    for task in self._my_assigned_tasks[:]:
        if (task.job_id == id):
            self._my_assigned_tasks.remove(task)

    if (self._current_task):
        if (self._current_task.job_id == id):
            self._current_task = None
            task_assignment_functions.get_new_task(self)

    for task in self.waiting_tasks[:]:
        if (task.job_id == id):
            self.waiting_tasks.remove(task)

    for task in self._received_messages[:]:
        if (task.job_id == id):
            self._received_messages.remove(task)

    if (self._current_task):
        if (self._current_task.job_id == id):
            self._current_task = None
            task_assignment_functions.get_new_task(self)


def job_has_failed_mechanism(self, task):
    """
    If a job is not possible then the agent who notices that will send the impossible task to the other agents
    :param task: the failed task
    """
    print("send job has failed",self._agent_name, task.job_id)
    self._pub_job_failed.publish(task)


def failed_task_mechanism(self, task):
    """
    Gets called if a task is received as failed. Then this procedure puts this task at the beginning of all open tasks
    so it can get reassigned
    :param task: the failed task
    """
    if self._is_default_agent:
        print("Failed task", self.open_tasks, task)
    if (task.action == BUY_STRING):
        task.action = GET_STRING
    self.open_tasks.insert(0, task)
    task_rating(self)

    if self._current_task and task.task_id == self._current_task.task_id:
        self._current_task = None
        task_assignment_functions.get_new_task(self)
