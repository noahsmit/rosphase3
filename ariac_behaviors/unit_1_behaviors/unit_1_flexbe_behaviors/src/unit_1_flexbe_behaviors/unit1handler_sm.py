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
		# x:1019 y:131, x:933 y:290
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['index', 'order_id', 'kitting_shipments', 'number_of_kitting_shipments'], output_keys=['agv_id', 'station_id', 'shipment_type', 'number_of_products'])
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.startmessage = '[+] Started!'
		_state_machine.userdata.index = 0
		_state_machine.userdata.agv_id = 0
		_state_machine.userdata.station_id = ''
		_state_machine.userdata.shipment_type = ''
		_state_machine.userdata.order_id = 1
		_state_machine.userdata.kitting_shipments = ''
		_state_machine.userdata.number_of_kitting_shipments = ''
		_state_machine.userdata.number_of_products = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:243 y:74
			OperatableStateMachine.add('StartMessage',
										MessageState(),
										transitions={'continue': 'KittingHandler'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'startmessage'})

			# x:570 y:71
			OperatableStateMachine.add('KittingHandler',
										self.use_behavior(KittingHandlerSM, 'KittingHandler'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'kitting_shipments': 'kitting_shipments', 'number_of_kitting_shipments': 'number_of_kitting_shipments', 'index': 'index', 'agv_id': 'agv_id', 'station_id': 'station_id', 'shipment_type': 'shipment_type', 'number_of_products_1': 'number_of_products_1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
