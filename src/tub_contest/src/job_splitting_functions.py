from __future__ import division # force floating point division when using plain /
from tub_contest.msg import Task
from list_of_strings import *
from list_of_numbers import *
from shop_helper_functions import item_is_in_resource_node,item_is_in_storage
from agent_modules_tub_contest import euclidean_distance
from task_assignment_functions import get_new_task_id

def list_to_string(self, list1):
    """
    turns a list of tools to a list of strings
    :param self: object instance
    :param list1: list of tools
    :return: list of string
    """
    new_list = []
    for tool in list1:
        new_list.append(tool.name)
    return new_list


def is_assembled(self, item):  # job_splititng
    """
    A helper function that checks if an item can be bought directly or needs to be assembled
    :param self: object instance
    :param item: the specific item
    :return: boolean: true if item is already assembled, otherwise false
    """
    if len(self.products_info.get(item.name)) == 0:
        return True

    return False


def job_splitting(self, job):
    """
    this procedure splits the defined job into all needed tasks
    :param self: object instance
    :param job:
    :return:
    """
    tasks = []

    for item in job.items:
        self.task_counter += 1
        tasks.extend(split_item(self, item=item, job=job, amount=item.amount))
    # if self._is_default_agent:
    #    print("Task order", tasks)
    #self.part_of_for_task_id = None
    if self._is_default_agent:
        print("best workshop",job.id,job.storage_name,get_closest_workshop(self,job).name)
    # if self._is_default_agent:
    #    print("tasks", tasks)
    tasks = sort_tasks(self, tasks)
    return tasks


def split_item(self, item, job, part_of=None, parent_task=None, save_tasks=True, depth=0, amount=1):
    """
    recursive function to find all necessary tasks including assembled and that
    also need to be assembled for an specific item
    :param self: object instance
    :param item: Item
    :param job: Job
    :param part_of: Item
    :param save_tasks: Task[]
    :param depth: Depth level of item assemble
    :param amount: amount of items
    :return: Task[]
    """
    new_tasks = []
    is_in_resource_node=False
    if (is_assembled(self, item)): # item is already assembled
        if (part_of is None):
            task = create_task(self, GET_STRING, DELIVER_JOB_STRING, item, job)
            new_tasks.append(task)
        else:
            task = create_task(self, GET_STRING, ASSIST_ASSEMBLE_STRING, item, job)
            if parent_task: task.parent_task_id = parent_task.task_id
            task.part_of = part_of.name
            task.amount = amount
            new_tasks.append(task)
    else:
        # create task regarding the item can be delivered or should be used for assembling
        if (part_of is None):
            task = create_task(self, ASSEMBLE_STRING, DELIVER_JOB_STRING, item, job)
            task.amount = amount
        else:
            task = create_task(self, ASSEMBLE_STRING, ASSIST_ASSEMBLE_STRING, item, job)
            task.part_of = part_of.name
            task.amount = amount
            if parent_task: task.parent_task_id = parent_task.task_id

        # if the item is in a storage or in a resource node take it from there and do not assemble it
        if (item_is_in_resource_node(self, item.name)) or (item_is_in_storage(self,item.name,item.amount)):
            # task.action = GATHER_STRING
            print("item can be found in a storage",item.name,self._agent_name)
            is_in_resource_node = True



        task.required_tools = list_to_string(self, self.products.get(item.name).required_tools)
        new_tasks.append(task)

        # only do assist assemble related tasks if the item can not be found in a resource node
        if(not is_in_resource_node):
            # add a task for each required tool
            for tool in self.products.get(item.name).required_tools:
                sub_task = create_task(self, GET_STRING, ASSIST_ASSEMBLE_STRING, item, job)
                sub_task.item = tool.name
                sub_task.amount = 1
                sub_task.part_of = item.name
                sub_task.parent_task_id = task.task_id

                new_tasks.append(sub_task)

            for dict_item in self.products_info.get(item.name):
                new_tasks.extend(
                    split_item(self, item=dict_item, job=job, part_of=item, parent_task=task, save_tasks=save_tasks, depth=depth + 1, amount=amount * dict_item.amount))

    # give each task a task_id
    if save_tasks:
        # self.task_order.append(tool.name)
        for tool in self.products.get(item.name).required_tools:
            for task in new_tasks:
                if task.item == tool.name:
                    task.task_depth = self.task_counter * 10 - depth - 1
        for task in new_tasks:
            if task.item == item.name:
                task.task_depth = self.task_counter * 10 - depth
    return new_tasks


def get_closest_workshop(self,job):
    """
    finds to a given job the closest workshop
    :param self: class
    :param job: Job
    :return: Workshop: returns the closest workshops
    """
    destination = self.storages.get(job.storage_name)
    best_workshop = self._workshops.get("workshop0")

    if not destination or not best_workshop:
        raise Exception('Perception not yet complete storage: {}, workshop: {}', destination, best_workshop)

    min_distance = euclidean_distance(best_workshop.pos,destination.pos)
    for k,workshop in self._workshops.items():
        distance = euclidean_distance(workshop.pos,destination.pos)
        if(distance < min_distance):
            best_workshop = workshop
    return best_workshop


def sort_tasks(self, tasks):
    """
    return the sorted list, regarding the task_id, for a given task list
    :param self: class
    :param tasks: list of tasks
    :return: sorted task list
    """
    sorted_list = sorted(tasks, key=lambda x: x.task_depth)
    #if(self._is_default_agent):
    #    for t in sorted_list:
    #        print(t.task_id,t.item,t.amount)
    return sorted_list


def create_task(self, action, further_action, item, job):
    """
    A function to create a task.
    :param self: object instance
    :param action: the action for the server.
    :param further_action: the more defined action for the agent
    :param item: needed item
    :param job: the job of the task
    :return: task
    """

    task = Task()
    task.agent = self._agent_name
    task.facility = "test"
    task.action = action
    task.destination = job.storage_name
    task.further_action = further_action
    task.item = item.name
    task.amount = item.amount
    task.steps = 1
    task.job_id = job.id
    task.sim_id = self._sim_id
    task.task_id = get_new_task_id(self, task.job_id)

    try:
        closest_workshop = get_closest_workshop(self,job).name
    except Exception:
        closest_workshop = DEFAULT_WORKSHOP

    task.best_workshop = closest_workshop
    task.rating = 99999
    return task
