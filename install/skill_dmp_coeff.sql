CREATE TABLE IF NOT EXISTS `skill_dmp_coeff` (
  `skill_id` bigint(20) NOT NULL,
  `joint_id` int(11) NOT NULL,
  `my` double NOT NULL,
  `sigma` double NOT NULL,
  `coeff` double NOT NULL,
  PRIMARY KEY (`skill_id`,`joint_id`,`my`,`sigma`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
