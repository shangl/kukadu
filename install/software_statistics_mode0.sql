CREATE TABLE IF NOT EXISTS `software_statistics_mode0` (
  `function_id` int(11) NOT NULL,
  `start_timestamp` bigint(20) NOT NULL,
  `end_timestamp` bigint(20) DEFAULT NULL,
  PRIMARY KEY (`function_id`,`start_timestamp`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
