#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.add_offset_to_pose_state import AddOffsetToPoseState
from ariac_flexbe_states.compute_grasp_ariac_state import ComputeGraspAriacState
from ariac_flexbe_states.get_object_pose import GetObjectPoseState
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 10 2021
@author: Noah
'''
class PlaceSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(PlaceSM, self).__init__()
		self.name = 'Place'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		table = 'ariac_unit1_tables'
		# x:1233 y:590, x:523 y:274
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['agv_id', 'offset_pose'])
		_state_machine.userdata.agv_id = ''
		_state_machine.userdata.ref_frame = 'linear_arm_actuator'
		_state_machine.userdata.move_group = 'kitting_arm'
		_state_machine.userdata.namespace = '/ariac/kitting/'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.offset_pose = 0
		_state_machine.userdata.rotation = 0
		_state_machine.userdata.tool_link = 'ee_link'
		_state_machine.userdata.offset = 0.2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:85 y:24
			OperatableStateMachine.add('LookUpConfig',
										LookupFromTableState(parameter_name=table, table_name='agvs', index_title='agv', column_title='config'),
										transitions={'found': 'LookUpTray', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'agv_id', 'column_value': 'config_name'})

			# x:1179 y:174
			OperatableStateMachine.add('ComputePlace',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'MoveToPlace', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'namespace': 'namespace', 'tool_link': 'tool_link', 'pose': 'output_pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:925 y:86
			OperatableStateMachine.add('GetPose',
										GetObjectPoseState(),
										transitions={'continue': 'AddOffsetXYZ', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'frame': 'tray', 'pose': 'pose'})

			# x:1174 y:374
			OperatableStateMachine.add('GripperOff',
										VacuumGripperControlState(enable=False, service_name='/ariac/kitting/arm/gripper/control'),
										transitions={'continue': 'MoveBack', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:285 y:24
			OperatableStateMachine.add('LookUpTray',
										LookupFromTableState(parameter_name=table, table_name='agvs', index_title='agv', column_title='tray'),
										transitions={'found': 'MoveToAGV', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'agv_id', 'column_value': 'tray'})

			# x:1184 y:474
			OperatableStateMachine.add('MoveBack',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'WaitRetry3', 'control_failed': 'WaitRetry3', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:534 y:24
			OperatableStateMachine.add('MoveToAGV',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'GetPose', 'planning_failed': 'WaitRetry', 'control_failed': 'WaitRetry', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1171 y:274
			OperatableStateMachine.add('MoveToPlace',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'GripperOff', 'planning_failed': 'WaitRetry2', 'control_failed': 'WaitRetry2'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'namespace': 'namespace', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:792 y:8
			OperatableStateMachine.add('WaitRetry',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveToAGV'},
										autonomy={'done': Autonomy.Off})

			# x:1462 y:320
			OperatableStateMachine.add('WaitRetry2',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveToPlace'},
										autonomy={'done': Autonomy.Off})

			# x:1457 y:474
			OperatableStateMachine.add('WaitRetry3',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveBack'},
										autonomy={'done': Autonomy.Off})

			# x:1188 y:87
			OperatableStateMachine.add('AddOffsetXYZ',
										AddOffsetToPoseState(),
										transitions={'continue': 'ComputePlace'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'pose', 'offset_pose': 'offset_pose', 'output_pose': 'output_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
