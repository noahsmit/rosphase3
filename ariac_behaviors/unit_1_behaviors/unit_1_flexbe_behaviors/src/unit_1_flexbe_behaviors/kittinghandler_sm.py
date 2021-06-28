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
		# x:833 y:90, x:333 y:590
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['kitting_shipments', 'number_of_kitting_shipments', 'index'], output_keys=['agv_id', 'station_id', 'shipment_type', 'number_of_products_1'])
		_state_machine.userdata.kitting_index = 0
		_state_machine.userdata.kitting_shipments = []
		_state_machine.userdata.ONE = 1
		_state_machine.userdata.number_of_kitting_shipments = 0
		_state_machine.userdata.MINUSONE = -1
		_state_machine.userdata.index = 0
		_state_machine.userdata.agv_id = 0
		_state_machine.userdata.station_id = ''
		_state_machine.userdata.shipment_type = ''
		_state_machine.userdata.number_of_products_1 = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:157 y:74
			OperatableStateMachine.add('GetKitting',
										GetKittingShipmentFromOrderState(),
										transitions={'continue': 'ProductsHandler', 'invalid_index': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'kitting_shipments': 'kitting_shipments', 'kitting_index': 'kitting_index', 'shipment_type': 'shipment_type', 'products': 'products', 'agv_id': 'agv_id', 'station_id': 'station_id', 'number_of_products': 'number_of_products_1'})

			# x:470 y:71
			OperatableStateMachine.add('ProductsHandler',
										self.use_behavior(ProductsHandlerSM, 'ProductsHandler'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'shipment_type': 'shipment_type', 'products': 'products', 'agv_id': 'agv_id', 'number_of_products_1': 'number_of_products_1', 'index': 'index'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
