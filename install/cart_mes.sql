CREATE TABLE IF NOT EXISTS `cart_mes` (
  `cart_mes_id` bigint(20) NOT NULL,
  `time_stamp` bigint(20) NOT NULL,
  `robot_id` smallint(6) NOT NULL,
  `reference_frame_id` int(11) NOT NULL,
  `link_id` int(11) NOT NULL,
  PRIMARY KEY (`cart_mes_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
