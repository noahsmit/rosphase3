#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_logistics_flexbe_states.get_kitting_shipment_from_order_state import GetKittingShipmentFromOrderState
from ariac_support_flexbe_states.add_numeric_state import AddNumericState
from ariac_support_flexbe_states.equal_state import EqualState
from unit_1_flexbe_behaviors.productshandler_sm import ProductsHandlerSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 10 2021
@author: Noah
'''
class KittingHandlerSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(KittingHandlerSM, self).__init__()
		self.name = 'KittingHandler'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(ProductsHandlerSM, 'ProductsHandler')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:533 y:540, x:383 y:240
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['kitting_shipments', 'number_of_kitting_shipments'])
		_state_machine.userdata.kitting_index = 0
		_state_machine.userdata.kitting_shipments = []
		_state_machine.userdata.ONE = 1
		_state_machine.userdata.number_of_kitting_shipments = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:157 y:74
			OperatableStateMachine.add('GetKitting',
										GetKittingShipmentFromOrderState(),
										transitions={'continue': 'ProductsHandler', 'invalid_index': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'kitting_shipments': 'kitting_shipments', 'kitting_index': 'kitting_index', 'shipment_type': 'shipment_type', 'products': 'products', 'agv_id': 'agv_id', 'station_id': 'station_id', 'number_of_products': 'number_of_products'})

			# x:174 y:374
			OperatableStateMachine.add('IncreaseKI',
										AddNumericState(),
										transitions={'done': 'GetKitting'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'kitting_index', 'value_b': 'ONE', 'result': 'kitting_index'})

			# x:470 y:71
			OperatableStateMachine.add('ProductsHandler',
										self.use_behavior(ProductsHandlerSM, 'ProductsHandler'),
										transitions={'finished': 'CheckEqual', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'shipment_type': 'shipment_type', 'products': 'products', 'agv_id': 'agv_id', 'station_id': 'station_id', 'number_of_products': 'number_of_products'})

			# x:474 y:374
			OperatableStateMachine.add('CheckEqual',
										EqualState(),
										transitions={'true': 'finished', 'false': 'IncreaseKI'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'kitting_index', 'value_b': 'number_of_kitting_shipments'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
