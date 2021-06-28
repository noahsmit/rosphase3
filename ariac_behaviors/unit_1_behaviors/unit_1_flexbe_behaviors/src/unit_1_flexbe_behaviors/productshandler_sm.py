#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_logistics_flexbe_states.get_part_from_products_state import GetPartFromProductsState
from unit_1_flexbe_behaviors.pick_sm import PickSM
from unit_1_flexbe_behaviors.place_sm import PlaceSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 10 2021
@author: Noah
'''
class ProductsHandlerSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(ProductsHandlerSM, self).__init__()
		self.name = 'ProductsHandler'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(PickSM, 'Pick')
		self.add_behavior(PlaceSM, 'Place')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:933 y:340, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['shipment_type', 'products', 'agv_id', 'number_of_products_1', 'index'])
		_state_machine.userdata.products = []
		_state_machine.userdata.agv_id = ''
		_state_machine.userdata.station_id = ''
		_state_machine.userdata.number_of_products_1 = 0
		_state_machine.userdata.shipment_type = ''
		_state_machine.userdata.index = 0
		_state_machine.userdata.ONE = 1
		_state_machine.userdata.MINUSONE = -1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:127 y:70
			OperatableStateMachine.add('GetPartFromProducts',
										GetPartFromProductsState(),
										transitions={'continue': 'Pick', 'invalid_index': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'products': 'products', 'index': 'index', 'type': 'part', 'pose': 'pose'})

			# x:620 y:71
			OperatableStateMachine.add('Pick',
										self.use_behavior(PickSM, 'Pick'),
										transitions={'finished': 'Place', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'part': 'part', 'pose': 'pose', 'agv_id': 'agv_id'})

			# x:620 y:321
			OperatableStateMachine.add('Place',
										self.use_behavior(PlaceSM, 'Place'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'agv_id': 'agv_id', 'offset_pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
