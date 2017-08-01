CREATE TABLE IF NOT EXISTS `localize_objects` (
  `object_id` smallint(6) NOT NULL,
  `pose_estimator_id` smallint(6) NOT NULL,
  `x_coordinate` float NOT NULL,
  `y_coordinate` float NOT NULL,
  `z_coordinate` float NOT NULL,
  `quat` float NOT NULL,
  `timestamp` bigint(20) NOT NULL,
  `frame_id` int(11) NOT NULL,  
  PRIMARY KEY (`object_id`, `pose_estimator_id`, `x_coordinate`, `y_coordinate`, `z_coordinate`, `quat`, `timestamp`, `frame_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
