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
from ariac_logistics_flexbe_states.get_material_locations import GetMaterialLocationsState
from ariac_support_flexbe_states.get_item_from_list_state import GetItemFromListState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 10 2021
@author: Noah Smit
'''
class PickSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(PickSM, self).__init__()
		self.name = 'Pick'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:563 y:248, x:330 y:238
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part', 'pose', 'agv_id'])
		_state_machine.userdata.location_type = ''
		_state_machine.userdata.part = ''
		_state_machine.userdata.agv_id = ''
		_state_machine.userdata.pose = ''
		_state_machine.userdata.index = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:128 y:99
			OperatableStateMachine.add('GetLocation',
										GetMaterialLocationsState(),
										transitions={'continue': 'GetItemFromList'},
										autonomy={'continue': Autonomy.Off},
										remapping={'part': 'part', 'location_type': 'location_type', 'material_locations': 'material_locations'})

			# x:648 y:137
			OperatableStateMachine.add('ms',
										MessageState(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'item'})

			# x:433 y:79
			OperatableStateMachine.add('GetItemFromList',
										GetItemFromListState(),
										transitions={'done': 'ms', 'invalid_index': 'failed'},
										autonomy={'done': Autonomy.Off, 'invalid_index': Autonomy.Off},
										remapping={'list': 'material_locations', 'index': 'index', 'item': 'item'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
