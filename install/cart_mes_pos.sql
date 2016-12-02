CREATE TABLE IF NOT EXISTS `cart_mes_pos` (
  `robot_id` smallint(6) NOT NULL,
  `timestamp` bigint(20) NOT NULL,
  `reference_frame` varchar(150) NOT NULL,
  `link_name` varchar(150) NOT NULL,
  `cart_pos_x` float NOT NULL,
  `cart_pos_y` float NOT NULL,
  `cart_pos_z` float NOT NULL,
  `cart_rot_x` float NOT NULL,
  `cart_rot_y` float NOT NULL,
  `cart_rot_z` float NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
