#!/usr/bin/python3
import rospy
from std_srvs.srv import Trigger
from termcolor import colored


# First, define a dictionary that maps actions to functions
action_to_fn = {
    'switch-airflow-off1': 'airflow_off',
    'switch-airflow-on1': 'airflow_on',
}


def execute(name):
    try:
        globals()[action_to_fn[name]]()
    except KeyError:
        print(colored('Unable to find action {} in custom actions.'.format(name) +
                      'Continuing without executing it.', color='red'))
    except RuntimeError:
        print(colored('Failed to execute the custom method {}.'.format(action_to_fn[name]) +
                      'Skipping action {}'.format(name)))
    # Add any success print here, if needed


def airflow_off():
    rospy.wait_for_service('airflow_off', timeout=rospy.Duration(secs=5))
    response = rospy.ServiceProxy('airflow_off', Trigger)()
    if not response.success:
        raise RuntimeError('Failed to execute airflow off')


def airflow_on():
    rospy.wait_for_service('airflow_on', timeout=rospy.Duration(secs=5))
    response = rospy.ServiceProxy('airflow_on', Trigger)()
    if not response.success:
        raise RuntimeError('Failed to execute airflow on')
