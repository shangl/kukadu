CREATE TABLE IF NOT EXISTS `skill_dmp_g` (
  `skill_id` bigint(20) NOT NULL,
  `joint_id` int(11) NOT NULL,
  `position` double NOT NULL,
  PRIMARY KEY (`skill_id`,`joint_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
