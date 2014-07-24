#!/usr/bin/env bash

rostopic pub -1 /gazebo/wam/wam/planning_scene moveit_msgs/PlanningScene "
  name: ''
  robot_state: 
    joint_state: 
      header: 
        seq: 0
        stamp: 
          secs: 0
          nsecs: 0
        frame_id: ''
      name: []
      position: []
      velocity: []
      effort: []
    multi_dof_joint_state: 
      header: 
        seq: 0
        stamp: 
          secs: 0
          nsecs: 0
        frame_id: ''
      joint_names: []
      transforms: []
      twist: []
      wrench: []
    attached_collision_objects: []
    is_diff: False
  robot_model_name: ''
  fixed_frame_transforms: []
  allowed_collision_matrix: 
    entry_names: ['ring1', 'wam/base_link', 'wam/forearm_link', 'wam/hand/bhand_palm_link', 'wam/hand/finger_1/dist_link', 'wam/hand/finger_1/med_link', 'wam/hand/finger_1/prox_link', 'wam/hand/finger_2/dist_link', 'wam/hand/finger_2/med_link', 'wam/hand/finger_2/prox_link', 'wam/hand/finger_3/dist_link', 'wam/hand/finger_3/med_link', 'wam/shoulder_pitch_link', 'wam/shoulder_yaw_link', 'wam/upper_arm_link', 'wam/wrist_pitch_link', 'wam/wrist_yaw_link']
    entry_values: 
      - 
        enabled: [True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True, True]
      - 
        enabled: [True, False, False, False, False, False, False, False, False, False, False, False, False, False, True, False, False]
      - 
        enabled: [True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]
      - 
        enabled: [True, False, False, False, False, False, True, False, False, True, False, False, False, False, False, True, True]
      - 
        enabled: [True, False, False, False, False, True, False, False, False, False, False, False, False, False, False, False, False]
      - 
        enabled: [True, False, False, False, True, False, False, False, False, False, False, False, False, False, False, False, False]
      - 
        enabled: [True, False, False, True, False, False, False, False, False, False, False, False, False, False, False, False, False]
      - 
        enabled: [True, False, False, False, False, False, False, False, True, False, False, False, False, False, False, False, False]
      - 
        enabled: [True, False, False, False, False, False, False, True, False, False, False, False, False, False, False, False, False]
      - 
        enabled: [True, False, False, True, False, False, False, False, False, False, False, False, False, False, False, False, False]
      - 
        enabled: [True, False, False, False, False, False, False, False, False, False, False, True, False, False, False, False, False]
      - 
        enabled: [True, False, False, False, False, False, False, False, False, False, True, False, False, False, False, False, False]
      - 
        enabled: [True, False, False, False, False, False, False, False, False, False, False, False, False, True, True, False, False]
      - 
        enabled: [True, False, False, False, False, False, False, False, False, False, False, False, True, False, True, False, False]
      - 
        enabled: [True, True, False, False, False, False, False, False, False, False, False, False, True, True, False, False, False]
      - 
        enabled: [True, False, False, True, False, False, False, False, False, False, False, False, False, False, False, False, True]
      - 
        enabled: [True, False, False, True, False, False, False, False, False, False, False, False, False, False, False, True, False]
    default_entry_names: []
    default_entry_values: []
  link_padding: []
  link_scale: []
  object_colors: []
  world: 
    collision_objects: []
    octomap: 
      header: 
        seq: 0
        stamp: 
          secs: 0
          nsecs: 0
        frame_id: ''
      origin: 
        position: 
          x: 0.0
          y: 0.0
          z: 0.0
        orientation: 
          x: 0.0
          y: 0.0
          z: 0.0
          w: 0.0
      octomap: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs: 0
          frame_id: ''
        binary: False
        id: ''
        resolution: 0.0
        data: []
  is_diff: True

"
