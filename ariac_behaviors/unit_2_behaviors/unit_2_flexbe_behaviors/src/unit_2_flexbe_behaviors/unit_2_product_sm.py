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
from ariac_logistics_flexbe_states.get_part_from_products_state import GetPartFromProductsState
from ariac_support_flexbe_states.add_numeric_state import AddNumericState
from ariac_support_flexbe_states.equal_state import EqualState
from unit_2_flexbe_behaviors.pick_part_unit_2_sm import pick_part_unit_2SM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 14 2021
@author: Eric Bolier
'''
class unit_2_productSM(Behavior):
	'''
	processes the product
	'''


	def __init__(self):
		super(unit_2_productSM, self).__init__()
		self.name = 'unit_2_product'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(pick_part_unit_2SM, 'pick_part_unit_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:725 y:479, x:450 y:206
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['number_of_products', 'shipment_type', 'products', 'station_id'])
		_state_machine.userdata.number_of_products = 0
		_state_machine.userdata.station_id = ''
		_state_machine.userdata.shipment_type = ''
		_state_machine.userdata.products = []
		_state_machine.userdata.ONE = 1
		_state_machine.userdata.index = 0
		_state_machine.userdata.message_01 = 'MSG: increment added'
		_state_machine.userdata.station_id = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:103 y:33
			OperatableStateMachine.add('GetPart',
										GetPartFromProductsState(),
										transitions={'continue': 'msg2', 'invalid_index': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'products': 'products', 'index': 'index', 'type': 'part', 'pose': 'pose'})

			# x:111 y:498
			OperatableStateMachine.add('Increment',
										AddNumericState(),
										transitions={'done': 'msg_1'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'index', 'value_b': 'ONE', 'result': 'result'})

			# x:381 y:21
			OperatableStateMachine.add('msg2',
										MessageState(),
										transitions={'continue': 'pick_part_unit_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'station_id'})

			# x:122 y:258
			OperatableStateMachine.add('msg_1',
										MessageState(),
										transitions={'continue': 'GetPart'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'message_01'})

			# x:656 y:49
			OperatableStateMachine.add('pick_part_unit_2',
										self.use_behavior(pick_part_unit_2SM, 'pick_part_unit_2'),
										transitions={'finished': 'CheckEqual', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'part': 'part', 'pose': 'pose', 'station_id': 'station_id'})

			# x:669 y:264
			OperatableStateMachine.add('CheckEqual',
										EqualState(),
										transitions={'true': 'finished', 'false': 'Increment'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'number_of_products', 'value_b': 'ONE'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
