#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.get_agv_status_state import GetAgvStatusState
from ariac_flexbe_states.notify_kitting_shipment_ready_state import NotifyKittingShipmentState
from ariac_support_flexbe_states.equal_state import EqualState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 15 2021
@author: Noah
'''
class AGVHandlerSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(AGVHandlerSM, self).__init__()
		self.name = 'AGVHandler'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:617 y:492, x:351 y:405
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['shipment_type', 'agv_id', 'assembly_station_name'])
		_state_machine.userdata.shipment_type = ''
		_state_machine.userdata.agv_id = ''
		_state_machine.userdata.assembly_station_name = ''
		_state_machine.userdata.TRUE = True
		_state_machine.userdata.done = 'ready_to_deliver'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:134 y:42
			OperatableStateMachine.add('NotifyKittingReady',
										NotifyKittingShipmentState(),
										transitions={'continue': 'QualityControl', 'fail': 'failed', 'service_timeout': 'failed'},
										autonomy={'continue': Autonomy.Off, 'fail': Autonomy.Off, 'service_timeout': Autonomy.Off},
										remapping={'agv_id': 'agv_id', 'shipment_type': 'shipment_type', 'assembly_station_name': 'assembly_station_name', 'success': 'success', 'message': 'message'})

			# x:695 y:62
			OperatableStateMachine.add('GetStatus',
										GetAgvStatusState(),
										transitions={'continue': 'CheckDone', 'fail': 'failed'},
										autonomy={'continue': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'agv_id': 'agv_id', 'agv_state': 'agv_state'})

			# x:399 y:57
			OperatableStateMachine.add('QualityControl',
										EqualState(),
										transitions={'true': 'GetStatus', 'false': 'failed'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'success', 'value_b': 'TRUE'})

			# x:607 y:217
			OperatableStateMachine.add('CheckDone',
										EqualState(),
										transitions={'true': 'finished', 'false': 'QualityControl'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'agv_state', 'value_b': 'done'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
