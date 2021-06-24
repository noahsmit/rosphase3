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
from flexbe_states.wait_state import WaitState
from unit_1_flexbe_behaviors.agvhandler_sm import AGVHandlerSM
from unit_1_flexbe_behaviors.unit1_initialize_sm import Unit1_initializeSM
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
		self.add_behavior(Unit1_initializeSM, 'Unit1_initialize')
		self.add_behavior(unit_2_behaviour_v1SM, 'unit_2_behaviour_v1')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1532 y:593, x:558 y:317
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.index = 0
		_state_machine.userdata.ONE = 1
		_state_machine.userdata.MINUSONE = -1
		_state_machine.userdata.number_of_products = 2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:64 y:28
			OperatableStateMachine.add('Start',
										StartAssignment(),
										transitions={'continue': 'Wait'},
										autonomy={'continue': Autonomy.Off})

			# x:742 y:336
			OperatableStateMachine.add('Add1',
										AddNumericState(),
										transitions={'done': 'Unit1Handler'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'index', 'value_b': 'ONE', 'result': 'index'})

			# x:996 y:456
			OperatableStateMachine.add('CheckEqual',
										EqualState(),
										transitions={'true': 'AGVHandler1', 'false': 'Add1'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'index', 'value_b': 'result'})

			# x:1449 y:306
			OperatableStateMachine.add('End',
										EndAssignment(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:564 y:20
			OperatableStateMachine.add('GetOrder',
										GetOrderState(),
										transitions={'order_found': 'Unit1Handler', 'no_order_found': 'failed'},
										autonomy={'order_found': Autonomy.Off, 'no_order_found': Autonomy.Off},
										remapping={'order_id': 'order_id', 'kitting_shipments': 'kitting_shipments', 'number_of_kitting_shipments': 'number_of_kitting_shipments', 'assembly_shipments': 'assembly_shipments', 'number_of_assembly_shipments': 'number_of_assembly_shipments'})

			# x:186 y:109
			OperatableStateMachine.add('Initialise_behaviour_unit_2',
										self.use_behavior(Initialise_behaviour_unit_2SM, 'Initialise_behaviour_unit_2'),
										transitions={'finished': 'Unit1_initialize', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:972 y:208
			OperatableStateMachine.add('NOP-1',
										AddNumericState(),
										transitions={'done': 'CheckEqual'},
										autonomy={'done': Autonomy.Off},
										remapping={'value_a': 'number_of_products', 'value_b': 'MINUSONE', 'result': 'result'})

			# x:766 y:49
			OperatableStateMachine.add('Unit1Handler',
										self.use_behavior(Unit1HandlerSM, 'Unit1Handler'),
										transitions={'finished': 'unit_2_behaviour_v1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'index': 'index', 'order_id': 'order_id', 'kitting_shipments': 'kitting_shipments', 'number_of_kitting_shipments': 'number_of_kitting_shipments', 'agv_id': 'agv_id1', 'station_id': 'station_id1', 'shipment_type': 'shipment_type1'})

			# x:370 y:63
			OperatableStateMachine.add('Unit1_initialize',
										self.use_behavior(Unit1_initializeSM, 'Unit1_initialize'),
										transitions={'finished': 'GetOrder', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:39 y:120
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=0.5),
										transitions={'done': 'Initialise_behaviour_unit_2'},
										autonomy={'done': Autonomy.Off})

			# x:1030 y:71
			OperatableStateMachine.add('unit_2_behaviour_v1',
										self.use_behavior(unit_2_behaviour_v1SM, 'unit_2_behaviour_v1'),
										transitions={'finished': 'NOP-1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'order_id': 'order_id', 'assembly_shipments': 'assembly_shipments', 'number_of_assembly_shipments': 'number_of_assembly_shipments'})

			# x:1212 y:300
			OperatableStateMachine.add('AGVHandler1',
										self.use_behavior(AGVHandlerSM, 'AGVHandler1'),
										transitions={'finished': 'End', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'shipment_type': 'shipment_type1', 'agv_id': 'agv_id1', 'assembly_station_name': 'station_id1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
