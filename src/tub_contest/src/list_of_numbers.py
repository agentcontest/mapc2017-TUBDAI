# this file contains a list of integer constants

# a max value for the rating process
MAX_VALUE = 99999999

# threshold for job feasibility. the job must be at least active for the given number to be taken
JOB_FEASIBILITY_THRESHOLD = 100

# penalty if an agent has an open task
PENALTY_OPEN_TASKS = 10000

# penalty if an agent has a missing tool for assembling
PENALTY_MISSING_TOOL = 1000

# penalty for the number of steps the agent needs to fulfill its task
PENALTY_FOR_STEPS = 10

# the percentage of the max battery when the agent needs to charge
CHARGING_PERCENTAGE = 0.3

# the threshold when an agent needs to recharge as he will not be able to move otherwise
CRITICAL_CHARGING = 12

# Gives after which number of round without a successful action the agent will abort there current job
NOTHING_WORKED_THRESHOLD = 10

# the number of agents per team. needs to be changed if the team size changes. NUMBER_OF_AGENTS*NUMBER_OF_GROUPS=Number of agents controlled/started
NUMBER_OF_AGENTS = 7
# the number of groups in one simulation.
NUMBER_OF_GROUPS = 4

# describes the percentage of the number of agents that need to be free in order to take a new job
PERCENTAGE_OF_FREE_AGENTS = 1.0

# if the currrent step is greater than this threshold (percentage of simulation steps
# (e.g. 0.9 with 1000 steps means after 900 steps no more jobs are accepted)) then no further jobs are taken
JOB_TOO_OLD_THRESHOLD = 0.9

# if a needed item, which can be gathered, is needed below this threshold then it will be gathered. Otherwise bought
GATHER_RESOURCE_AMOUNT_THRESHOLD = 5

# Offset in seconds substracted from the calculated response deadline to make sure the answer arrives in time
ACTION_REQUEST_SAFETY_OFFSET = 0.2

# battery charging consumed per goto step
REQUIRED_CHARGE_PER_STEP = 10

# Agent recharge rate
RECHARGE_RATE = 5

BIDDING_REVENUE_OFFSET = 0.1

ENABLE_AUCTIONS = False

ENABLE_PRICED_JOBS = True
