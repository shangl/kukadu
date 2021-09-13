CREATE TABLE IF NOT EXISTS `controller_executions` (
  `controller_id` bigint(20) NOT NULL,
  `start_timestamp` bigint(20) NOT NULL,
  `end_timestamp` bigint(20) DEFAULT NULL,
  `successful` tinyint(1) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
