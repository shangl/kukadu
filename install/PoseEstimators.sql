CREATE TABLE IF NOT EXISTS `pose_estimators` (
  `estimator_id` smallint(6) NOT NULL,
  `class_name` varchar(255) NOT NULL,
  PRIMARY KEY (`estimator_id`),
  UNIQUE KEY `estimator_id` (`estimator_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
