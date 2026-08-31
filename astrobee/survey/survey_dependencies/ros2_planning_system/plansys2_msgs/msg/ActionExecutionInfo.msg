int8 NOT_EXECUTED=1
int8 EXECUTING=2
int8 FAILED=3
int8 SUCCEEDED=4
int8 CANCELLED=5

int8 status

time start_stamp
time status_stamp

string action_full_name

string action
string[] arguments

# predicted duration of the action from the planner
duration duration

float32 completion
string message_status
