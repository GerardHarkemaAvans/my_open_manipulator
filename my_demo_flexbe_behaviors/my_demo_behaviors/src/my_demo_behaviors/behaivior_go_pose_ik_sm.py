#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.moveit_to_joints_dyn_state import MoveitToJointsDynState
from miscellaneous_flexbe_states.get_tf_transform_state import GetTFTransformState
from miscellaneous_flexbe_states.message_state import MessageState
from open_manipulator_moveit_flexbe_states.ik_get_joints_from_pose_state import IkGetJointsFromPose
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on December 12, 2021
@author: Gerard Harkema
'''
class behaivior_go_pose_ikSM(Behavior):
	'''
	Demo of a behavior using moveit for the open-manipulator robot
	'''


	def __init__(self):
		super(behaivior_go_pose_ikSM, self).__init__()
		self.name = 'behaivior_go_pose_ik'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
		action_topic = "move_group"
		arm_group = 'arm'
		# x:983 y:40, x:530 y:236
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['frames'])
		_state_machine.userdata.offset = 0.01
		_state_machine.userdata.rotation = 1.57
		_state_machine.userdata.move_group_prefix = ''
		_state_machine.userdata.ee_link = "end_effector_link"
		_state_machine.userdata.frames = []
		_state_machine.userdata.group_name = 'arm'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:73 y:24
			OperatableStateMachine.add('get_pose_transform',
										GetTFTransformState(),
										transitions={'done': 'pose_message', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'frames': 'frames', 'transform': 'part_pose'})

			# x:776 y:24
			OperatableStateMachine.add('go_pose',
										MoveitToJointsDynState(move_group=arm_group, action_topic=action_topic),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:497 y:24
			OperatableStateMachine.add('ik_calculate_pose_joints',
										IkGetJointsFromPose(joint_names=joint_names, time_out=5.0),
										transitions={'continue': 'go_pose', 'failed': 'failed', 'time_out': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'time_out': Autonomy.Off},
										remapping={'group_name': 'group_name', 'move_group_prefix': 'move_group_prefix', 'tool_link': 'ee_link', 'pose': 'part_pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:273 y:24
			OperatableStateMachine.add('pose_message',
										MessageState(severity=Logger.REPORT_HINT, header="Pose of testpoint"),
										transitions={'continue': 'ik_calculate_pose_joints'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'part_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
