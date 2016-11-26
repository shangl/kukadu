CREATE TABLE IF NOT EXISTS `joint_mes` (
  `robot_id` SMALLINT NOT NULL,
  `joint_id` SMALLINT NOT NULL,
  `time_stamp` BIGINT,
  `position` float DEFAULT NULL,
  `velocity` float DEFAULT NULL,
  `acceleration` float DEFAULT NULL,
  `frc` float DEFAULT NULL,
  PRIMARY KEY (`robot_id`, `joint_id`,`time_stamp`),
  UNIQUE KEY `robot_id` (`robot_id`, `joint_id`,`time_stamp`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
