CREATE TABLE IF NOT EXISTS `robot_config` (
  `robot_config_id` bigint(20) unsigned not null,
  `hardware_instance_id` int(11) not null,
  `order_id` int(11) not null,
  PRIMARY KEY (`robot_config_id`, `hardware_instance_id`, `order_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1 AUTO_INCREMENT=1 ;
