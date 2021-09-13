CREATE TABLE IF NOT EXISTS `software_statistics_mode1` (
  `function_id` int(11) NOT NULL,
  `start_timestamp` bigint(20) NOT NULL,
  `end_timestamp` bigint(20) DEFAULT NULL,
  `cnt` smallint(6) default null,
  PRIMARY KEY (`function_id`,`start_timestamp`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
