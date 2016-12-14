CREATE TABLE IF NOT EXISTS `controlled_joints` (
  `skill_id` int(11) NOT NULL,
  `joint_id` int(11) NOT NULL,
  PRIMARY KEY (`skill_id`,`joint_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
