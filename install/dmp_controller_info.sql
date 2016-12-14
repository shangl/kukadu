CREATE TABLE IF NOT EXISTS `dmp_controller_info` (
  `skill_id` int(11) NOT NULL,
  `data_start_time` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  `data_end_time` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00',
  `use_joint_mode` tinyint(1) NOT NULL DEFAULT '1',
  PRIMARY KEY (`skill_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
