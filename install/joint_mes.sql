CREATE TABLE IF NOT EXISTS `joint_mes` (
  `robot_id` smallint(6) NOT NULL,
  `joint_id` smallint(6) NOT NULL,
  `time_stamp` bigint(20) NOT NULL DEFAULT '0',
  `position` float DEFAULT NULL,
  `velocity` float DEFAULT NULL,
  `acceleration` float DEFAULT NULL,
  `frc` float DEFAULT NULL,
  PRIMARY KEY (`robot_id`,`joint_id`,`time_stamp`),
  UNIQUE KEY `robot_id` (`robot_id`,`joint_id`,`time_stamp`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
