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
from open_manipulator_moveit_flexbe_states.srdf_state_to_moveit import SrdfStateToMoveit as open_manipulator_moveit_flexbe_states__SrdfStateToMoveit
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on December 12, 2021
@author: Gerard Harkema
'''
class moveit_flexbe_demoSM(Behavior):
	'''
	Demo of a behavior using moveit
	'''


	def __init__(self):
		super(moveit_flexbe_demoSM, self).__init__()
		self.name = 'moveit_flexbe_demo'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		arm_group = 'arm'
		joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
		action_topic = "move_group"
		gripper_group = 'gripper'
		# x:99 y:284, x:530 y:236
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.captured_pointcloud = []
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.pick_configuration = []
		_state_machine.userdata.offset = 0.01
		_state_machine.userdata.rotation = 1.57
		_state_machine.userdata.move_group_prefix = ''
		_state_machine.userdata.move_group = "move_group"
		_state_machine.userdata.ee_link = "end_effector_link"
		_state_machine.userdata.world = "world"
		_state_machine.userdata.frames = ["world" , "ik_testpoint"]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:26 y:24
			OperatableStateMachine.add('GoHome',
										open_manipulator_moveit_flexbe_states__SrdfStateToMoveit(config_name='home', move_group=arm_group, action_topic=action_topic, robot_name="", tolerance=0.0),
										transitions={'reached': 'GoLeft', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:297 y:24
			OperatableStateMachine.add('GoLeft',
										open_manipulator_moveit_flexbe_states__SrdfStateToMoveit(config_name='left', move_group=arm_group, action_topic=action_topic, robot_name="", tolerance=0.0),
										transitions={'reached': 'getTransformFromTestpoint', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:47 y:424
			OperatableStateMachine.add('GoResting',
										open_manipulator_moveit_flexbe_states__SrdfStateToMoveit(config_name='resting', move_group=arm_group, action_topic=action_topic, robot_name="", tolerance=0.0),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:297 y:424
			OperatableStateMachine.add('GoRight',
										open_manipulator_moveit_flexbe_states__SrdfStateToMoveit(config_name='right', move_group=arm_group, action_topic=action_topic, robot_name="", tolerance=0.0),
										transitions={'reached': 'GoResting', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:997 y:24
			OperatableStateMachine.add('IkCalculateJointsFromPose',
										IkGetJointsFromPose(joint_names=joint_names, time_out=5.0),
										transitions={'continue': 'MoveToTestpoint', 'failed': 'failed', 'time_out': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'time_out': Autonomy.Off},
										remapping={'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'tool_link': 'ee_link', 'pose': 'part_pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1026 y:424
			OperatableStateMachine.add('MoveToTestpoint',
										MoveitToJointsDynState(move_group=arm_group, action_topic=action_topic),
										transitions={'reached': 'GoRight', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:773 y:24
			OperatableStateMachine.add('PoseMessage',
										MessageState(severity=Logger.REPORT_HINT, header="Pose of testpoint"),
										transitions={'continue': 'IkCalculateJointsFromPose'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'part_pose'})

			# x:570 y:24
			OperatableStateMachine.add('getTransformFromTestpoint',
										GetTFTransformState(),
										transitions={'done': 'PoseMessage', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'frames': 'frames', 'transform': 'part_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
