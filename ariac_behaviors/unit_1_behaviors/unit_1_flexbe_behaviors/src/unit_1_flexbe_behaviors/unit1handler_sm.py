#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.message_state import MessageState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from ariac_flexbe_states.start_assignment_state import StartAssignment
from ariac_logistics_flexbe_states.get_order_state import GetOrderState
from flexbe_states.wait_state import WaitState
from unit_1_flexbe_behaviors.kittinghandler_sm import KittingHandlerSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 10 2021
@author: Noah Smit
'''
class Unit1HandlerSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(Unit1HandlerSM, self).__init__()
		self.name = 'Unit1Handler'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(KittingHandlerSM, 'KittingHandler')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1019 y:131, x:553 y:383
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.config_name_home = 'kitting_bin_group_a'
		_state_machine.userdata.move_group = 'kitting_arm'
		_state_machine.userdata.namespace = '/ariac/kitting/'
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.startmessage = '[+] Started!'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:53 y:201
			OperatableStateMachine.add('Start',
										StartAssignment(),
										transitions={'continue': 'StartMessage'},
										autonomy={'continue': Autonomy.Off})

			# x:699 y:107
			OperatableStateMachine.add('KittingHandler',
										self.use_behavior(KittingHandlerSM, 'KittingHandler'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'kitting_shipments': 'kitting_shipments', 'number_of_kitting_shipments': 'number_of_kitting_shipments'})

			# x:304 y:128
			OperatableStateMachine.add('MoveHome',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'GetOrder', 'planning_failed': 'WaitRetry', 'control_failed': 'WaitRetry', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_home', 'move_group': 'move_group', 'namespace': 'namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:112 y:133
			OperatableStateMachine.add('StartMessage',
										MessageState(),
										transitions={'continue': 'MoveHome'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'startmessage'})

			# x:321 y:21
			OperatableStateMachine.add('WaitRetry',
										WaitState(wait_time=0.5),
										transitions={'done': 'MoveHome'},
										autonomy={'done': Autonomy.Off})

			# x:500 y:118
			OperatableStateMachine.add('GetOrder',
										GetOrderState(),
										transitions={'order_found': 'KittingHandler', 'no_order_found': 'failed'},
										autonomy={'order_found': Autonomy.Off, 'no_order_found': Autonomy.Off},
										remapping={'order_id': 'order_id', 'kitting_shipments': 'kitting_shipments', 'number_of_kitting_shipments': 'number_of_kitting_shipments', 'assembly_shipments': 'assembly_shipments', 'number_of_assembly_shipments': 'number_of_assembly_shipments'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
