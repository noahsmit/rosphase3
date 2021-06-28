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
from unit_2_flexbe_behaviors.processorder_unit2_sm import ProcessOrder_unit2SM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 10 2021
@author: Eric Bolier
'''
class unit_2_behaviour_v1SM(Behavior):
	'''
	Behaviour for unit 2
	'''


	def __init__(self):
		super(unit_2_behaviour_v1SM, self).__init__()
		self.name = 'unit_2_behaviour_v1'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(ProcessOrder_unit2SM, 'ProcessOrder_unit2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1408 y:250, x:694 y:304
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['order_id', 'assembly_shipments', 'number_of_assembly_shipments', 'index'], output_keys=['number_of_products_2'])
		_state_machine.userdata.part_type = ''
		_state_machine.userdata.material_locations = []
		_state_machine.userdata.message_1 = 'MSG: unit2 initialise complete'
		_state_machine.userdata.message_2 = 'MSG: order recieved'
		_state_machine.userdata.order_id = 1
		_state_machine.userdata.assembly_shipments = ''
		_state_machine.userdata.number_of_assembly_shipments = ''
		_state_machine.userdata.index = 0
		_state_machine.userdata.number_of_products_2 = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:393 y:74
			OperatableStateMachine.add('InitialiseSucces',
										MessageState(),
										transitions={'continue': 'ProcessOrder_unit2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'message_1'})

			# x:970 y:71
			OperatableStateMachine.add('ProcessOrder_unit2',
										self.use_behavior(ProcessOrder_unit2SM, 'ProcessOrder_unit2'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'number_of_assembly_shipments': 'number_of_assembly_shipments', 'assembly_shipments': 'assembly_shipments', 'index': 'index', 'number_of_products_2': 'number_of_products_2'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
