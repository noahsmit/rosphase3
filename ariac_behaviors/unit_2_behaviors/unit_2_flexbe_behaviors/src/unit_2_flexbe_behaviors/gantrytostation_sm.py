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
from ariac_support_flexbe_states.equal_state import EqualState
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
		# x:1522 y:494, x:668 y:478
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['station_id', 'index'])
		_state_machine.userdata.station_id = ''
		_state_machine.userdata.move_group = 'gantry_torso'
		_state_machine.userdata.namespace = '/ariac/gantry'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.index = 0
		_state_machine.userdata.NULL = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:169 y:44
			OperatableStateMachine.add('LookUpGroupConfig',
										LookupFromTableState(parameter_name=table, table_name='stations', index_title='station', column_title='robot_group'),
										transitions={'found': 'LookUpStationConfig', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'station_id', 'column_value': 'config_name_group'})

			# x:366 y:52
			OperatableStateMachine.add('LookUpStationConfig',
										LookupFromTableState(parameter_name=table, table_name='stations', index_title='station', column_title='robot_config'),
										transitions={'found': 'CheckEqual', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'station_id', 'column_value': 'config_name'})

			# x:824 y:147
			OperatableStateMachine.add('MoveToGroup',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'MoveToStation', 'planning_failed': 'WaitRetry', 'control_failed': 'WaitRetry', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_group', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1447 y:187
			OperatableStateMachine.add('MoveToStation',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'WaitRetry_2', 'control_failed': 'WaitRetry_2', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:842 y:247
			OperatableStateMachine.add('WaitRetry',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveToGroup'},
										autonomy={'done': Autonomy.Off})

			# x:1490 y:37
			OperatableStateMachine.add('WaitRetry_2',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveToStation'},
										autonomy={'done': Autonomy.Off})

			# x:633 y:30
			OperatableStateMachine.add('CheckEqual',
										EqualState(),
										transitions={'true': 'MoveToGroup', 'false': 'MoveToStation'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'index', 'value_b': 'NULL'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
