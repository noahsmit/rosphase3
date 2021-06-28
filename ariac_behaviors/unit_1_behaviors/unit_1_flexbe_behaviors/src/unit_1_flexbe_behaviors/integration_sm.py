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
from ariac_flexbe_states.start_assignment_state import StartAssignment
from ariac_logistics_flexbe_states.get_order_state import GetOrderState
from ariac_support_flexbe_states.add_numeric_state import AddNumericState
from ariac_support_flexbe_states.equal_state import EqualState
from ariac_support_flexbe_states.greater_numeric_state import GreaterNumericState
from flexbe_states.wait_state import WaitState
from unit_1_flexbe_behaviors.agvhandler_sm import AGVHandlerSM
from unit_1_flexbe_behaviors.unit1handler_sm import Unit1HandlerSM
from unit_2_flexbe_behaviors.initialise_behaviour_unit_2_sm import Initialise_behaviour_unit_2SM
from unit_2_flexbe_behaviors.unit_2_behaviour_v1_sm import unit_2_behaviour_v1SM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 24 2021
@author: Noah Smit Eric Bolier
'''
class IntegrationSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(IntegrationSM, self).__init__()
		self.name = 'Integration'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(AGVHandlerSM, 'AGVHandler1')
		self.add_behavior(Initialise_behaviour_unit_2SM, 'Initialise_behaviour_unit_2')
		self.add_behavior(Unit1HandlerSM, 'Unit1Handler')
		self.add_behavior(unit_2_behaviour_v1SM, 'unit_2_behaviour_v1')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1539 y:652, x:558 y:317
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.index = 0
		_state_machine.userdata.ONE = 1
		_state_machine.userdata.MINUSONE = -1
		_state_machine.userdata.number_of_products_1 = 0
		_state_machine.userdata.index2 = 0
		_state_machine.userdata.result2 = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:64 y:28
			OperatableStateMachine.add('Start',
										StartAssignment(),
										transitions={'continue': 'Wait'},
										autonomy={'continue': Autonomy.Off})

			# x:1141 y:287
			OperatableStateMachine.add('Add1',
										AddNumericState(),
										transitions={'done': 'Add1_2_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'index', 'value_b': 'ONE', 'result': 'index'})

			# x:1165 y:161
			OperatableStateMachine.add('Add1_2',
										AddNumericState(),
										transitions={'done': 'unit_2_behaviour_v1'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'index2', 'value_b': 'ONE', 'result': 'index2'})

			# x:823 y:290
			OperatableStateMachine.add('Add1_2_2',
										AddNumericState(),
										transitions={'done': 'GreaterThanNOF'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'index2', 'value_b': 'ONE', 'result': 'index2'})

			# x:1398 y:213
			OperatableStateMachine.add('CheckEqual',
										EqualState(),
										transitions={'true': 'CheckEqual_2', 'false': 'Add1'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'index', 'value_b': 'result'})

			# x:1398 y:293
			OperatableStateMachine.add('CheckEqual_2',
										EqualState(),
										transitions={'true': 'End', 'false': 'Add1_2'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'index2', 'value_b': 'result2'})

			# x:1546 y:523
			OperatableStateMachine.add('End',
										EndAssignment(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:253 y:99
			OperatableStateMachine.add('GetOrder',
										GetOrderState(),
										transitions={'order_found': 'GreaterThanNOF', 'no_order_found': 'failed'},
										autonomy={'order_found': Autonomy.Off, 'no_order_found': Autonomy.Off},
										remapping={'order_id': 'order_id', 'kitting_shipments': 'kitting_shipments', 'number_of_kitting_shipments': 'number_of_kitting_shipments', 'assembly_shipments': 'assembly_shipments', 'number_of_assembly_shipments': 'number_of_assembly_shipments'})

			# x:853 y:29
			OperatableStateMachine.add('GreaterThanNOF',
										GreaterNumericState(),
										transitions={'true': 'End', 'false': 'unit_2_behaviour_v1'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'index2', 'value_b': 'result2'})

			# x:49 y:423
			OperatableStateMachine.add('Initialise_behaviour_unit_2',
										self.use_behavior(Initialise_behaviour_unit_2SM, 'Initialise_behaviour_unit_2'),
										transitions={'finished': 'GetOrder', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1401 y:36
			OperatableStateMachine.add('NOP-1',
										AddNumericState(),
										transitions={'done': 'NOP-1_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'number_of_products_1', 'value_b': 'MINUSONE', 'result': 'result'})

			# x:1395 y:120
			OperatableStateMachine.add('NOP-1_2',
										AddNumericState(),
										transitions={'done': 'CheckEqual'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'number_of_products_2', 'value_b': 'MINUSONE', 'result': 'result2'})

			# x:574 y:36
			OperatableStateMachine.add('Unit1Handler',
										self.use_behavior(Unit1HandlerSM, 'Unit1Handler'),
										transitions={'finished': 'Unit1Handler', 'failed': 'Unit1Handler'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'index': 'index', 'order_id': 'order_id', 'kitting_shipments': 'kitting_shipments', 'number_of_kitting_shipments': 'number_of_kitting_shipments', 'agv_id': 'agv_id1', 'station_id': 'station_id1', 'shipment_type': 'shipment_type1', 'number_of_products': 'number_of_products_1'})

			# x:117 y:191
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=0.5),
										transitions={'done': 'Initialise_behaviour_unit_2'},
										autonomy={'done': Autonomy.Off})

			# x:1100 y:50
			OperatableStateMachine.add('unit_2_behaviour_v1',
										self.use_behavior(unit_2_behaviour_v1SM, 'unit_2_behaviour_v1'),
										transitions={'finished': 'NOP-1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'order_id': 'order_id', 'assembly_shipments': 'assembly_shipments', 'number_of_assembly_shipments': 'number_of_assembly_shipments', 'index': 'index', 'number_of_products_2': 'number_of_products_2'})

			# x:1183 y:551
			OperatableStateMachine.add('AGVHandler1',
										self.use_behavior(AGVHandlerSM, 'AGVHandler1'),
										transitions={'finished': 'AGVHandler1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'shipment_type': 'shipment_type1', 'agv_id': 'agv_id1', 'assembly_station_name': 'station_id1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
