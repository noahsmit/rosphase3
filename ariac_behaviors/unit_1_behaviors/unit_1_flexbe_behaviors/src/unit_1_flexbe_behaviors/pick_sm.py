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
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.message_state import MessageState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from ariac_logistics_flexbe_states.get_material_locations import GetMaterialLocationsState
from ariac_support_flexbe_states.get_item_from_list_state import GetItemFromListState
from ariac_support_flexbe_states.text_to_float_state import TextToFloatState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 10 2021
@author: Noah Smit
'''
class PickSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(PickSM, self).__init__()
		self.name = 'Pick'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		table = 'ariac_unit1_tables'
		# x:563 y:248, x:547 y:195
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part', 'pose', 'agv_id'])
		_state_machine.userdata.location_type = ''
		_state_machine.userdata.part = ''
		_state_machine.userdata.agv_id = ''
		_state_machine.userdata.pose = ''
		_state_machine.userdata.index = 0
		_state_machine.userdata.move_group = 'kitting_arm'
		_state_machine.userdata.namespace = '/ariac/kitting'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.ref_frame = 'linear_arm_actuator'
		_state_machine.userdata.tool_link = 'ee_link'
		_state_machine.userdata.rotation = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:73 y:24
			OperatableStateMachine.add('GetLocation',
										GetMaterialLocationsState(),
										transitions={'continue': 'GetItemFromList'},
										autonomy={'continue': Autonomy.Off},
										remapping={'part': 'part', 'location_type': 'location_type', 'material_locations': 'material_locations'})

			# x:1022 y:224
			OperatableStateMachine.add('DetectPart',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'MessagePose', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'camera_topic': 'camera_topic', 'camera_frame': 'camera_frame', 'part': 'part', 'pose': 'pose'})

			# x:88 y:168
			OperatableStateMachine.add('GetItemFromList',
										GetItemFromListState(),
										transitions={'done': 'MessageBin', 'invalid_index': 'failed'},
										autonomy={'done': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'list': 'material_locations', 'index': 'index', 'item': 'bin'})

			# x:824 y:424
			OperatableStateMachine.add('GripperOn',
										VacuumGripperControlState(enable=True, service_name='/ariac/kitting/arm/gripper/control'),
										transitions={'continue': 'MoveToPart', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:984 y:24
			OperatableStateMachine.add('LookUpCameraFrame',
										LookupFromTableState(parameter_name=table, table_name='bins', index_title='bin', column_title='camera_frame'),
										transitions={'found': 'LookUpPosition', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'camera_frame'})

			# x:485 y:24
			OperatableStateMachine.add('LookUpCameraTopic',
										LookupFromTableState(parameter_name=table, table_name='bins', index_title='bin', column_title='camera_topic'),
										transitions={'found': 'LookUpPartOffset', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'camera_topic'})

			# x:661 y:21
			OperatableStateMachine.add('LookUpPartOffset',
										LookupFromTableState(parameter_name=table, table_name='parts', index_title='part', column_title='offset'),
										transitions={'found': 'TextToFloat', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'part', 'column_value': 'offset_text'})

			# x:1135 y:24
			OperatableStateMachine.add('LookUpPosition',
										LookupFromTableState(parameter_name=table, table_name='bins', index_title='bin', column_title='robot_config'),
										transitions={'found': 'MoveToPosition', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'bin', 'column_value': 'position'})

			# x:297 y:21
			OperatableStateMachine.add('MessageBin',
										MessageState(),
										transitions={'continue': 'LookUpCameraTopic'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'bin'})

			# x:1043 y:324
			OperatableStateMachine.add('MessagePose',
										MessageState(),
										transitions={'continue': 'ComputePartPosition'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'pose'})

			# x:280 y:394
			OperatableStateMachine.add('MoveBack',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'position', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:621 y:424
			OperatableStateMachine.add('MoveToPart',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'Wait', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'namespace': 'namespace', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1034 y:124
			OperatableStateMachine.add('MoveToPosition',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'DetectPart', 'planning_failed': 'WaitRetry', 'control_failed': 'WaitRetry', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'position', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:805 y:6
			OperatableStateMachine.add('TextToFloat',
										TextToFloatState(),
										transitions={'done': 'LookUpCameraFrame'},
										autonomy={'done': Autonomy.Off},
										remapping={'text_value': 'offset_text', 'float_value': 'offset'})

			# x:458 y:411
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveBack'},
										autonomy={'done': Autonomy.Off})

			# x:1207 y:124
			OperatableStateMachine.add('WaitRetry',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveToPosition'},
										autonomy={'done': Autonomy.Off})

			# x:1029 y:424
			OperatableStateMachine.add('ComputePartPosition',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'GripperOn', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'namespace': 'namespace', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
