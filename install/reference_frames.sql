CREATE TABLE IF NOT EXISTS `reference_frames` (
  `frame_id` int(11) NOT NULL,
  `robot_id` bigint(20) NOT NULL,
  `frame_name` varchar(250) NOT NULL,
  PRIMARY KEY (`frame_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
