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
from ariac_logistics_flexbe_states.get_assembly_shipment_from_order_state import GetAssemblyShipmentFromOrderState
from ariac_support_flexbe_states.equal_state import EqualState
from unit_2_flexbe_behaviors.gantrytogroup_sm import GantryToGroupSM
from unit_2_flexbe_behaviors.gantrytostation_sm import GantryToStationSM
from unit_2_flexbe_behaviors.unit_2_product_sm import unit_2_productSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 10 2021
@author: Eric Bolier
'''
class ProcessOrder_unit2SM(Behavior):
	'''
	Process the order from the unit 2 module
	'''


	def __init__(self):
		super(ProcessOrder_unit2SM, self).__init__()
		self.name = 'ProcessOrder_unit2'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(GantryToGroupSM, 'GantryToGroup')
		self.add_behavior(GantryToStationSM, 'GantryToStation')
		self.add_behavior(unit_2_productSM, 'unit_2_product')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		table = 'ariac_unit2_tables'
		# x:1172 y:484, x:309 y:379
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['number_of_assembly_shipments', 'assembly_shipments', 'index'], output_keys=['number_of_products_2'])
		_state_machine.userdata.message_1 = 'MSG: assembly_index recieved'
		_state_machine.userdata.agv_id = ''
		_state_machine.userdata.assembly_index = 0
		_state_machine.userdata.assembly_shipments = []
		_state_machine.userdata.number_of_assembly_shipments = 0
		_state_machine.userdata.index = 0
		_state_machine.userdata.number_of_products_2 = 0
		_state_machine.userdata.NULL = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:117 y:22
			OperatableStateMachine.add('GetAssemblyShipment',
										GetAssemblyShipmentFromOrderState(),
										transitions={'continue': 'msg1', 'invalid_index': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'assembly_shipments': 'assembly_shipments', 'assembly_index': 'assembly_index', 'shipment_type': 'shipment_type', 'products': 'products', 'shipment_type': 'shipment_type', 'station_id': 'station_id', 'number_of_products': 'number_of_products_2'})

			# x:920 y:21
			OperatableStateMachine.add('GantryToGroup',
										self.use_behavior(GantryToGroupSM, 'GantryToGroup'),
										transitions={'finished': 'GantryToStation', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'station_id': 'station_id'})

			# x:920 y:221
			OperatableStateMachine.add('GantryToStation',
										self.use_behavior(GantryToStationSM, 'GantryToStation'),
										transitions={'finished': 'unit_2_product', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'station_id': 'station_id'})

			# x:393 y:24
			OperatableStateMachine.add('msg1',
										MessageState(),
										transitions={'continue': 'CheckFirstRound'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'products'})

			# x:920 y:371
			OperatableStateMachine.add('unit_2_product',
										self.use_behavior(unit_2_productSM, 'unit_2_product'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'number_of_products_2': 'number_of_products_2', 'shipment_type': 'shipment_type', 'products': 'products', 'station_id': 'station_id', 'index': 'index'})

			# x:618 y:28
			OperatableStateMachine.add('CheckFirstRound',
										EqualState(),
										transitions={'true': 'GantryToGroup', 'false': 'GantryToStation'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'index', 'value_b': 'NULL'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
