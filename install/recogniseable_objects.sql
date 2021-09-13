CREATE TABLE IF NOT EXISTS `recogniseable_objects` (
  `estimator_id` smallint(6) NOT NULL,
  `object_id` smallint(6) NOT NULL,
  PRIMARY KEY (`estimator_id`,`object_id`),
  UNIQUE KEY `recogniseable_objects_id` (`estimator_id`,`object_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
