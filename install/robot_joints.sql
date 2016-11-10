CREATE TABLE IF NOT EXISTS `robot_joints` (
  `robot_id` int(11) NOT NULL,
  `joint_id` int(11) NOT NULL,
  `joint_name` varchar(255) NOT NULL,
  PRIMARY KEY (`robot_id`,`joint_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
