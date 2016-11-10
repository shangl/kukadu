CREATE TABLE IF NOT EXISTS `robot` (
  `robot_id` bigint(20) unsigned NOT NULL AUTO_INCREMENT,
  `robot_name` varchar(255) NOT NULL,
  PRIMARY KEY (`robot_id`),
  UNIQUE KEY `robot_id` (`robot_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1 AUTO_INCREMENT=1;
