from __future__ import division # force floating point division when using plain /
from fault_tolerance_functions import *
from shop_helper_functions import *
from job_splitting_functions import *
from knowledge_functions import *
from list_of_numbers import *
from list_of_strings import *
import rospy
import re


def rate_job(self, job):
    """
    this function takes a job and gives a rating regarding the expected profit and required time
    :param self: object instance
    :param job: Job
    :return: Int: a rating for the specified job
    """
    reward = job.reward
    item_costs = 0
    tasks = []
    for item in job.items:
        tasks.extend(split_item(self, item=item, job=job, save_tasks=False))
    for task in tasks:
        if (task.action == GET_STRING):
            item_costs += get_average_item_price(self, task.item) * task.amount
            # if(task.action==Assemble_String):
    rating = reward - item_costs + job.fine
    # divide by number of all tasks to get average gain of each task
    rating = rating / len(tasks)
    return rating


def job_is_for_my_group(self, job):
    """
    evaluates the job and assigns it regarding a given formula to one of the groups
    :param self: class
    :param job: the evaluated job
    :return: bool if this job belongs to the agent's group
    """
    return not job.id in self._other_accepted_jobs

def job_rating(self, with_priced_jobs=ENABLE_PRICED_JOBS):
    """
    A function that first checks for all possible jobs, rates these and chooses the best one
    :param self: object instance
    :return: Job: the job that needs to be completed
    """
    self._step_counter += 1
    # if there are there are open tasks do not get a new job

    if (len(self.open_tasks) > 0):
        return None

    free_agents_cnt = get_number_of_free_agents(self)

    if (free_agents_cnt >= NUMBER_OF_AGENTS * PERCENTAGE_OF_FREE_AGENTS):
        best_job = None
        best_rating = 0

        for k, job in self._mission_jobs.items():
            if (k not in self._accepted_jobs) and job_is_for_my_group(self, job) and (check_if_job_is_feasible(self, job)) :
                rate = rate_job(self, job)

                if (best_rating < rate):
                    best_job = job
                    best_rating = rate

        if (best_job) and (best_rating > 0):#TODO best_rating should maybe even bigger than 0
            self._accepted_jobs.append(best_job.id)
            self._stats_match.increment_current_value("job_mission", 1)
            return best_job

        if with_priced_jobs:

            for k, job in self._priced_jobs.items():
                if (k not in self._accepted_jobs) and job_is_for_my_group(self, job) and (check_if_job_is_feasible(self, job)) :
                    rate = rate_job(self, job)

                    if (best_rating < rate):
                        best_job = job
                        best_rating = rate

            if (best_job) and (best_rating > 0):
                self._accepted_jobs.append(best_job.id)
                self._stats_match.increment_current_value("job_priced", 1)
                return best_job

            # TODO auction?
    else:
        rospy.logdebug('Not enough free agents, only %d', free_agents_cnt)
    return None


def check_for_old_job(self):
    """
    Checks for expired job and removes associated tasks by calling kill_all_tasks_of_job_id procedure
    Should be called in the beginning of every step
    :param self: object instance
    :return: "kill tasks"
    """
    if self._this_agent and self._this_agent.last_action_result == "successful" and self._this_agent.last_action == DELIVER_JOB_STRING:
        # job_id = self._current_task.job_id
        #
        # #first check missions
        #
        # job = self._mission_jobs.pop(job_id)
        #
        # #next check regular jobs
        # if not job:
        #     job = self._priced_jobs.pop(job_id)

        rospy.logwarn("%s:: group %s, Completed job.", self._agent_name, self._agent_group)
        self._stats_match.increment_current_value("completed_jobs", 1)
        # self._stats_match.increment_current_value("earned_reward", job.reward)
        # kill_all_tasks_of_job_id(self, job_id)

    # TODO check for already achieved jobs

    for k, job in self._mission_jobs.items():
        if (self._current_step > job.end) :
            id = job.id
            self._mission_jobs.pop(k)
            if(check_if_job_still_active(self,id)):
                self._stats_match.increment_current_value("expired_jobs", 1)
            kill_all_tasks_of_job_id(self, id, reason=' expired')

    for k, job in self._priced_jobs.items():
        if (self._current_step > job.end) or (job_already_finished(self,k)):
            id = job.id
            self._priced_jobs.pop(k)
            if (check_if_job_still_active(self, id)) :
                self._stats_match.increment_current_value("expired_jobs", 1)
            kill_all_tasks_of_job_id(self, id, reason=' expired')
            # TODO add auctions?

def job_already_finished(self,id):
    """
    goes through all available jobs and returns true if the given id is not found
    :param self:
    :param id: job id
    :return: True if job is not available anymore
    """
    for job in self.request_action.priced_jobs:
        if(job.id == id):
            return False
    rospy.loginfo("%s already finished ", id)
    return True


def check_if_job_still_active(self,id):
    """
    goes through all list and dicts which could contain elements of the given job and return True is something was found
    :param self: class
    :param id: job_id
    :return: True if something related to the job was found
    """

    for task in self.open_tasks[:]:
        if (task.job_id == id):
            return True

    for task in self._my_assigned_tasks[:]:
        if (task.job_id == id):
            return True

    if (self._current_task):
        if (self._current_task.job_id == id):
            return True

    for task in self.waiting_tasks[:]:
        if (task.job_id == id):
            return True

    for task in self._received_messages[:]:
        if (task.job_id == id):
            return True

    if (self._current_task):
        if (self._current_task.job_id == id):
            return True
    return False