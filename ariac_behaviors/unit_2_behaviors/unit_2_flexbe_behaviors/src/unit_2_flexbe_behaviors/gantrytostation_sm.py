#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 15 2021
@author: Eric
'''
class GantryToStationSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(GantryToStationSM, self).__init__()
		self.name = 'GantryToStation'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		table = 'ariac_unit2_tables'
		# x:1045 y:284, x:668 y:478
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['station_id'])
		_state_machine.userdata.station_id = ''
		_state_machine.userdata.move_group = 'gantry_torso'
		_state_machine.userdata.namespace = '/ariac/gantry'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.action_topic = '/move_group'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:213 y:59
			OperatableStateMachine.add('LookUpGroupConfig',
										LookupFromTableState(parameter_name=table, table_name='stations', index_title='station', column_title='robot_group'),
										transitions={'found': 'LookUpStationConfig', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'station_id', 'column_value': 'config_name_group'})

			# x:373 y:52
			OperatableStateMachine.add('LookUpStationConfig',
										LookupFromTableState(parameter_name=table, table_name='stations', index_title='station', column_title='robot_config'),
										transitions={'found': 'MoveToStation', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'station_id', 'column_value': 'config_name'})

			# x:632 y:198
			OperatableStateMachine.add('MoveToGroup',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'MoveToGroup', 'planning_failed': 'WaitRetry', 'control_failed': 'WaitRetry', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_group', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:880 y:111
			OperatableStateMachine.add('MoveToStation',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'WaitRetry_2', 'control_failed': 'WaitRetry_2', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:611 y:128
			OperatableStateMachine.add('WaitRetry',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveToGroup'},
										autonomy={'done': Autonomy.Off})

			# x:885 y:29
			OperatableStateMachine.add('WaitRetry_2',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveToStation'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
