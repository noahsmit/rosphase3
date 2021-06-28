#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.compute_grasp_ariac_state import ComputeGraspAriacState
from ariac_flexbe_states.detect_part_camera_ariac_state import DetectPartCameraAriacState
from ariac_flexbe_states.get_gripper_status_state import GetGripperStatusState
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.message_state import MessageState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from ariac_support_flexbe_states.equal_state import EqualState
from ariac_support_flexbe_states.text_to_float_state import TextToFloatState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 15 2021
@author: Eric Bolier
'''
class pick_part_unit_2SM(Behavior):
	'''
	Picks part from AGV at the right station
	'''


	def __init__(self):
		super(pick_part_unit_2SM, self).__init__()
		self.name = 'pick_part_unit_2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		table = 'ariac_unit2_tables'
		# x:34 y:349, x:583 y:240
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part', 'station_id'])
		_state_machine.userdata.part = ''
		_state_machine.userdata.location_type = ''
		_state_machine.userdata.index_value = ''
		_state_machine.userdata.message_1 = 'MSG: part location found'
		_state_machine.userdata.message_2 = 'MSG: frame found'
		_state_machine.userdata.station_id = ''
		_state_machine.userdata.index = 0
		_state_machine.userdata.ref_frame = 'world'
		_state_machine.userdata.move_group = 'gantry_full'
		_state_machine.userdata.namespace = '/ariac/gantry'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.tool_link = 'gantry_arm_ee_link'
		_state_machine.userdata.rotation = 0
		_state_machine.userdata.move_group_arm = 'gantry_arm'
		_state_machine.userdata.TRUE = True

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:76 y:174
			OperatableStateMachine.add('LookUpCameraFrame',
										LookupFromTableState(parameter_name=table, table_name='stations', index_title='station', column_title='camera_frame'),
										transitions={'found': 'MessageCF', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'station_id', 'column_value': 'camera_frame'})

			# x:87 y:477
			OperatableStateMachine.add('CheckAttached',
										EqualState(),
										transitions={'true': 'finished', 'false': 'DetectPart'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'attached', 'value_b': 'TRUE'})

			# x:1141 y:426
			OperatableStateMachine.add('ComputeMSG',
										MessageState(),
										transitions={'continue': 'ActivateGripper'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'joint_values'})

			# x:929 y:329
			OperatableStateMachine.add('ComputePos',
										ComputeGraspAriacState(joint_names=['gantry_arm_elbow_joint', 'gantry_arm_shoulder_lift_joint', 'gantry_arm_shoulder_pan_joint', 'gantry_arm_wrist_1_joint', 'gantry_arm_wrist_2_joint', 'gantry_arm_wrist_3_joint']),
										transitions={'continue': 'ComputeMSG', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group_arm', 'namespace': 'namespace', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1025 y:55
			OperatableStateMachine.add('DetectPart',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'PoseMSG', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic', 'camera_frame': 'camera_frame', 'part': 'part', 'pose': 'pose'})

			# x:266 y:494
			OperatableStateMachine.add('GetGripperStatus',
										GetGripperStatusState(topic_name='/ariac/gantry/arm/gripper/state'),
										transitions={'continue': 'CheckAttached', 'fail': 'failed'},
										autonomy={'continue': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'enabled': 'enabled', 'attached': 'attached'})

			# x:498 y:44
			OperatableStateMachine.add('GetOffset',
										LookupFromTableState(parameter_name=table, table_name='parts', index_title='part', column_title='offset'),
										transitions={'found': 'LookUpPick', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'part', 'column_value': 'offset_text'})

			# x:318 y:21
			OperatableStateMachine.add('LookUpCameraTopic',
										LookupFromTableState(parameter_name=table, table_name='stations', index_title='station', column_title='camera_topic'),
										transitions={'found': 'GetOffset', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'station_id', 'column_value': 'camera_topic'})

			# x:679 y:39
			OperatableStateMachine.add('LookUpPick',
										LookupFromTableState(parameter_name=table, table_name='stations', index_title='station', column_title='pick'),
										transitions={'found': 'MsgCT', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'station_id', 'column_value': 'pick'})

			# x:93 y:50
			OperatableStateMachine.add('MessageCF',
										MessageState(),
										transitions={'continue': 'LookUpCameraTopic'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'camera_frame'})

			# x:473 y:604
			OperatableStateMachine.add('MoveBack',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'GetGripperStatus', 'planning_failed': 'Wait', 'control_failed': 'Wait', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'pick', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:944 y:589
			OperatableStateMachine.add('MoveToPart',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'Wait', 'planning_failed': 'Wait', 'control_failed': 'Wait'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'namespace': 'namespace', 'move_group': 'move_group_arm', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:939 y:247
			OperatableStateMachine.add('MoveToPick',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'ComputePos', 'planning_failed': 'WaitRetry_2', 'control_failed': 'WaitRetry_2', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'pick', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:894 y:168
			OperatableStateMachine.add('MsgCT',
										MessageState(),
										transitions={'continue': 'TextToFloatOffset'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'pick'})

			# x:1133 y:150
			OperatableStateMachine.add('PoseMSG',
										MessageState(),
										transitions={'continue': 'MoveToPick'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'pose'})

			# x:850 y:52
			OperatableStateMachine.add('TextToFloatOffset',
										TextToFloatState(),
										transitions={'done': 'DetectPart'},
										autonomy={'done': Autonomy.Off},
										remapping={'text_value': 'offset_text', 'float_value': 'offset'})

			# x:723 y:594
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=0.1),
										transitions={'done': 'MoveBack'},
										autonomy={'done': Autonomy.Off})

			# x:1204 y:253
			OperatableStateMachine.add('WaitRetry_2',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveToPick'},
										autonomy={'done': Autonomy.Off})

			# x:935 y:426
			OperatableStateMachine.add('ActivateGripper',
										VacuumGripperControlState(enable=True, service_name='/ariac/gantry/arm/gripper/control'),
										transitions={'continue': 'MoveToPart', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
