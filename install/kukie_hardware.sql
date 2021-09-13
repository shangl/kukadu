CREATE TABLE IF NOT EXISTS `kukie_hardware` (
  `hardware_instance_id` int(11) NOT NULL,
  `deg_of_freedom` smallint(6) NOT NULL,
  `frequency` float NOT NULL,
  `name_prefix` varchar(255) NOT NULL,
  PRIMARY KEY (`hardware_instance_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
