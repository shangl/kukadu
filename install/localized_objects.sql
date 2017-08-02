CREATE TABLE IF NOT EXISTS `localized_objects` (
  `object_id` smallint(6) NOT NULL,
  `pose_estimator_id` smallint(6) NOT NULL,
  `x_position` float NOT NULL,
  `y_position` float NOT NULL,
  `z_position` float NOT NULL,
  `x_orientation` float NOT NULL,
  `y_orientation` float NOT NULL,
  `z_orientation` float NOT NULL,
  `w_orientation` float NOT NULL,
  `timestamp` bigint(20) NOT NULL,
  `frame_id` int(11) NOT NULL,  
  PRIMARY KEY (`object_id`, `pose_estimator_id`, `x_position`, `y_position`, `z_position`, `x_orientation`, `y_orientation`, `z_orientation`, `w_orientation`, `timestamp`, `frame_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
