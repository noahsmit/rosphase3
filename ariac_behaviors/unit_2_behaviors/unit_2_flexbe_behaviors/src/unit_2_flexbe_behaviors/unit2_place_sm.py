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
from ariac_flexbe_states.message_state import MessageState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 18 2021
@author: Eric
'''
class unit2_placeSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(unit2_placeSM, self).__init__()
		self.name = 'unit2_place'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		table = 'ariac_unit2_tables'
		# x:1183 y:402, x:599 y:383
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pose', 'part', 'station_id'])
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.namespace = '/ariac/gantry'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.pose = ''
		_state_machine.userdata.part = ''
		_state_machine.userdata.station_id = ''
		_state_machine.userdata.move_group_arm = 'gantry_arm'
		_state_machine.userdata.tool_link = 'gantry_arm_ee_link'
		_state_machine.userdata.offset2 = 0.1
		_state_machine.userdata.rotation = 0
		_state_machine.userdata.frame = ''
		_state_machine.userdata.ref_frame = 'world'
		_state_machine.userdata.offset = 0.4
		_state_machine.userdata.move_group = 'gantry_full'
		_state_machine.userdata.config_name = 'gantry_drop_as1'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:66 y:260
			OperatableStateMachine.add('LookUpFrame',
										LookupFromTableState(parameter_name=table, table_name='stations', index_title='station', column_title='frame'),
										transitions={'found': 'Test', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'station_id', 'column_value': 'frame'})

			# x:501 y:36
			OperatableStateMachine.add('ComputePlace',
										ComputeGraspAriacState(joint_names=['gantry_arm_elbow_joint', 'gantry_arm_shoulder_lift_joint', 'gantry_arm_shoulder_pan_joint', 'gantry_arm_wrist_1_joint', 'gantry_arm_wrist_2_joint', 'gantry_arm_wrist_3_joint']),
										transitions={'continue': 'MoveToPlace', 'failed': 'GripperOff'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group_arm', 'namespace': 'namespace', 'tool_link': 'tool_link', 'pose': 'output_pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1119 y:46
			OperatableStateMachine.add('GripperOff',
										VacuumGripperControlState(enable=False, service_name='/ariac/gantry/arm/gripper/control'),
										transitions={'continue': 'finished', 'failed': 'GripperOff'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:856 y:191
			OperatableStateMachine.add('MoveToPlace',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'GripperOff', 'planning_failed': 'WaitRetry2', 'control_failed': 'WaitRetry2'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'namespace': 'namespace', 'move_group': 'move_group_arm', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:493 y:124
			OperatableStateMachine.add('PosMSG',
										MessageState(),
										transitions={'continue': 'ComputePlace'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'output_pose'})

			# x:91 y:24
			OperatableStateMachine.add('Test',
										GetObjectPoseState(),
										transitions={'continue': 'AddOffset', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame', 'frame': 'frame', 'pose': 'briefcase_pose'})

			# x:926 y:331
			OperatableStateMachine.add('WaitRetry2',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveToPlace'},
										autonomy={'done': Autonomy.Off})

			# x:286 y:24
			OperatableStateMachine.add('AddOffset',
										AddOffsetToPoseState(),
										transitions={'continue': 'PosMSG'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_pose': 'briefcase_pose', 'offset_pose': 'pose', 'output_pose': 'output_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
