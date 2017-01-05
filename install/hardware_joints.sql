CREATE TABLE IF NOT EXISTS `hardware_joints` (
  `hardware_instance_id` int(11) NOT NULL,
  `joint_id` int(11) NOT NULL,
  `joint_name` varchar(255) NOT NULL,
  PRIMARY KEY (`hardware_instance_id`,`joint_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
