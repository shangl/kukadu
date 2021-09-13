CREATE TABLE IF NOT EXISTS `objects` (
  `object_id` smallint(6) NOT NULL,
  `object_name` varchar(255) NOT NULL,
  PRIMARY KEY (`object_id`),
  UNIQUE KEY `object_id` (`object_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
