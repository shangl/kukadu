-- installing kukie control queue (left and right arm)
INSERT INTO `hardware` (`hardware_id`, `hardware_name`, `hardware_class`) VALUES ('1', 'KukieControlQueue', '200');

-- left arm
INSERT INTO `hardware_instances` (`instance_id`, `instance_name`, `hardware_id`) VALUES ('1', 'kukie_left_arm', '1');
INSERT INTO `kukie_hardware` (`hardware_instance_id`, `deg_of_freedom`, `frequency`, `name_prefix`) VALUES ('1', '7', '100', 'left_arm');
-- left arm joints
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('1', '0', 'arm_0_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('1', '1', 'arm_1_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('1', '2', 'arm_2_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('1', '3', 'arm_3_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('1', '4', 'arm_4_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('1', '5', 'arm_5_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('1', '6', 'arm_6_joint');

-- right arm
INSERT INTO `hardware_instances` (`instance_id`, `instance_name`, `hardware_id`) VALUES ('2', 'kukie_right_arm', '1');
INSERT INTO `kukie_hardware` (`hardware_instance_id`, `deg_of_freedom`, `frequency`, `name_prefix`) VALUES ('2', '7', '100', 'right_arm');
-- right arm joints
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('2', '7', 'arm_0_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('2', '8', 'arm_1_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('2', '9', 'arm_2_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('2', '10', 'arm_3_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('2', '11', 'arm_4_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('2', '12', 'arm_5_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('2', '13', 'arm_6_joint');

-- installing kukie hand
INSERT INTO `hardware` (`hardware_id`, `hardware_name`, `hardware_class`) VALUES ('2', 'KukieHand', '100');

-- left hand
INSERT INTO `hardware_instances` (`instance_id`, `instance_name`, `hardware_id`) VALUES ('3', 'kukiehand_left', '2');
INSERT INTO `kukie_hardware` (`hardware_instance_id`, `deg_of_freedom`, `frequency`, `name_prefix`) VALUES ('3', '7', '100', 'left_sdh');
-- left hand joints
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('3', '14', 'knuckle_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('3', '15', 'finger_12_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('3', '16', 'finger_13_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('3', '17', 'thumb_2_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('3', '18', 'thumb_3_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('3', '19', 'finger_22_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('3', '20', 'finger_23_joint');

-- right hand
INSERT INTO `hardware_instances` (`instance_id`, `instance_name`, `hardware_id`) VALUES ('4', 'kukiehand_right', '2');
INSERT INTO `kukie_hardware` (`hardware_instance_id`, `deg_of_freedom`, `frequency`, `name_prefix`) VALUES ('4', '7', '100', 'right_sdh');
-- right hand joints
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('4', '0', 'knuckle_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('4', '1', 'finger_12_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('4', '2', 'finger_13_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('4', '3', 'thumb_2_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('4', '4', 'thumb_3_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('4', '5', 'finger_22_joint');
INSERT INTO `hardware_joints` (`hardware_instance_id`, `joint_id`, `joint_name`) VALUES ('4', '6', 'finger_23_joint');

-- vrep localizer
INSERT INTO `pose_estimators` (`estimator_id`, `class_name`) VALUES ('1', 'VrepLocalizer');
