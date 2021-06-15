#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.lookup_from_table import LookupFromTableState
from ariac_flexbe_states.message_state import MessageState
from ariac_logistics_flexbe_states.get_material_locations import GetMaterialLocationsState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 15 2021
@author: Eric Bolier
'''
class pick_part_unit_2SM(Behavior):
	'''
	Picks part from AGV at the right station
	'''


	def __init__(self):
		super(pick_part_unit_2SM, self).__init__()
		self.name = 'pick_part_unit_2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		table = 'ariac_unit2_tables'
		# x:922 y:489, x:583 y:240
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part', 'pose', 'station_id'])
		_state_machine.userdata.part = ''
		_state_machine.userdata.pose = ''
		_state_machine.userdata.location_type = ''
		_state_machine.userdata.index_value = ''
		_state_machine.userdata.message_1 = 'MSG: part location found'
		_state_machine.userdata.message_2 = 'MSG: frame found'
		_state_machine.userdata.station_id = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:123 y:74
			OperatableStateMachine.add('GetPartLocation',
										GetMaterialLocationsState(),
										transitions={'continue': 'msg1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'part': 'part', 'location_type': 'location_type', 'material_locations': 'material_locations'})

			# x:634 y:74
			OperatableStateMachine.add('LookUpCameraFrame',
										LookupFromTableState(parameter_name=table, table_name='stations', index_title='station', column_title='camera_topic'),
										transitions={'found': 'msg2', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'station_id', 'column_value': 'camera_topic'})

			# x:393 y:74
			OperatableStateMachine.add('msg1',
										MessageState(),
										transitions={'continue': 'LookUpCameraFrame'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'message_1'})

			# x:852 y:73
			OperatableStateMachine.add('msg2',
										MessageState(),
										transitions={'continue': 'GetOffset'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'message_2'})

			# x:831 y:226
			OperatableStateMachine.add('GetOffset',
										LookupFromTableState(parameter_name=table, table_name='parts', index_title='part', column_title='offset'),
										transitions={'found': 'finished', 'not_found': 'failed'},
										autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'index_value': 'part', 'column_value': 'column_value'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
