#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from my_demo_behaviors.behaivior_go_pose_ik_sm import behaivior_go_pose_ikSM
from open_manipulator_moveit_flexbe_states.srdf_state_to_moveit import SrdfStateToMoveit as open_manipulator_moveit_flexbe_states__SrdfStateToMoveit
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on December 12, 2021
@author: Gerard Harkema
'''
class behaivior_mainSM(Behavior):
	'''
	Demo of a behavior using moveit for the open-manipulator robot
	'''


	def __init__(self):
		super(behaivior_mainSM, self).__init__()
		self.name = 'behaivior_main'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(behaivior_go_pose_ikSM, 'behaivior_go_pose_ik')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		arm_group = 'arm'
		action_topic = "move_group"
		# x:1283 y:40, x:637 y:271
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.move_group_prefix = ''
		_state_machine.userdata.move_group = "move_group"
		_state_machine.userdata.ee_link = "end_effector_link"
		_state_machine.userdata.frames = ["world" , "ik_testpoint"]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:47 y:24
			OperatableStateMachine.add('go_home',
										open_manipulator_moveit_flexbe_states__SrdfStateToMoveit(config_name='home', move_group=arm_group, action_topic=action_topic, robot_name="", tolerance=0.0),
										transitions={'reached': 'go_left', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:297 y:24
			OperatableStateMachine.add('go_left',
										open_manipulator_moveit_flexbe_states__SrdfStateToMoveit(config_name='left', move_group=arm_group, action_topic=action_topic, robot_name="", tolerance=0.0),
										transitions={'reached': 'behaivior_go_pose_ik', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:997 y:24
			OperatableStateMachine.add('go_resting',
										open_manipulator_moveit_flexbe_states__SrdfStateToMoveit(config_name='resting', move_group=arm_group, action_topic=action_topic, robot_name="", tolerance=0.0),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:748 y:23
			OperatableStateMachine.add('go_right',
										open_manipulator_moveit_flexbe_states__SrdfStateToMoveit(config_name='right', move_group=arm_group, action_topic=action_topic, robot_name="", tolerance=0.0),
										transitions={'reached': 'go_resting', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:571 y:21
			OperatableStateMachine.add('behaivior_go_pose_ik',
										self.use_behavior(behaivior_go_pose_ikSM, 'behaivior_go_pose_ik'),
										transitions={'finished': 'go_right', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'frames': 'frames'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
