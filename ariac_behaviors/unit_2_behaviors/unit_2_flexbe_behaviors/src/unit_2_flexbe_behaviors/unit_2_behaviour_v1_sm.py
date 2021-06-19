#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.end_assignment_state import EndAssignment
from ariac_flexbe_states.message_state import MessageState
from ariac_logistics_flexbe_states.get_order_state import GetOrderState
from unit_2_flexbe_behaviors.initialise_behaviour_unit_2_sm import Initialise_behaviour_unit_2SM
from unit_2_flexbe_behaviors.processorder_unit2_sm import ProcessOrder_unit2SM
from unit_2_flexbe_behaviors.unit2_test_positions_sm import Unit2TestPositionsSM
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
		self.add_behavior(Initialise_behaviour_unit_2SM, 'Initialise_behaviour_unit_2')
		self.add_behavior(ProcessOrder_unit2SM, 'ProcessOrder_unit2')
		self.add_behavior(Unit2TestPositionsSM, 'Unit2 Test Positions')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1408 y:250, x:694 y:304
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_type = ''
		_state_machine.userdata.material_locations = []
		_state_machine.userdata.message_1 = 'MSG: unit2 initialise complete'
		_state_machine.userdata.message_2 = 'MSG: order recieved'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:110 y:71
			OperatableStateMachine.add('Initialise_behaviour_unit_2',
										self.use_behavior(Initialise_behaviour_unit_2SM, 'Initialise_behaviour_unit_2'),
										transitions={'finished': 'InitialiseSucces', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:623 y:74
			OperatableStateMachine.add('GetOrder',
										GetOrderState(),
										transitions={'order_found': 'MessageAssemblyShipments', 'no_order_found': 'failed'},
										autonomy={'order_found': Autonomy.Off, 'no_order_found': Autonomy.Off},
										remapping={'order_id': 'order_id', 'kitting_shipments': 'kitting_shipments', 'number_of_kitting_shipments': 'number_of_kitting_shipments', 'assembly_shipments': 'assembly_shipments', 'number_of_assembly_shipments': 'number_of_assembly_shipments'})

			# x:393 y:74
			OperatableStateMachine.add('InitialiseSucces',
										MessageState(),
										transitions={'continue': 'GetOrder'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'message_1'})

			# x:797 y:64
			OperatableStateMachine.add('MessageAssemblyShipments',
										MessageState(),
										transitions={'continue': 'ProcessOrder_unit2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'assembly_shipments'})

			# x:970 y:71
			OperatableStateMachine.add('ProcessOrder_unit2',
										self.use_behavior(ProcessOrder_unit2SM, 'ProcessOrder_unit2'),
										transitions={'finished': 'End', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'number_of_assembly_shipments': 'number_of_assembly_shipments', 'assembly_shipments': 'assembly_shipments'})

			# x:1008 y:457
			OperatableStateMachine.add('Unit2 Test Positions',
										self.use_behavior(Unit2TestPositionsSM, 'Unit2 Test Positions'),
										transitions={'finished': 'Unit2 Test Positions', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1343 y:74
			OperatableStateMachine.add('End',
										EndAssignment(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
