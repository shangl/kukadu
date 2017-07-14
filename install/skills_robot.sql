CREATE TABLE IF NOT EXISTS `skills_robot` (
  `skill_id` bigint(20) unsigned not null,
  `robot_config_id` int(11) not null,
  PRIMARY KEY (`skill_id`, `robot_config_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1 AUTO_INCREMENT=1 ;
