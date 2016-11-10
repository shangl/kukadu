CREATE TABLE IF NOT EXISTS `joint_mes` (
  `joint_id` int(11) NOT NULL,
  `time_stamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  `position` float NOT NULL,
  `velocity` float NOT NULL,
  `acceleration` int(11) NOT NULL,
  `frc` int(11) NOT NULL,
  PRIMARY KEY (`joint_id`,`time_stamp`),
  UNIQUE KEY `joint_id` (`joint_id`,`time_stamp`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
