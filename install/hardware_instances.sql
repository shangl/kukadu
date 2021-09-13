CREATE TABLE IF NOT EXISTS `hardware_instances` (
  `instance_id` int(11) NOT NULL,
  `instance_name` varchar(255) NOT NULL,
  `hardware_id` int(11) NOT NULL,
  PRIMARY KEY (`instance_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
