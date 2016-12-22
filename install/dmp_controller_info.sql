CREATE TABLE IF NOT EXISTS `dmp_controller_info` (
  `skill_id` int(11) NOT NULL,
  `data_start_time` bigint(20) NOT NULL,
  `data_end_time` bigint(20) NOT NULL,
  `use_joint_mode` tinyint(1) NOT NULL DEFAULT '1',
  PRIMARY KEY (`skill_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
