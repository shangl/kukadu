CREATE TABLE IF NOT EXISTS `cart_mes_pos` (
  `cart_mes_id` bigint(20) NOT NULL,
  `cart_pos_x` float NOT NULL,
  `cart_pos_y` float NOT NULL,
  `cart_pos_z` float NOT NULL,
  `cart_rot_x` float NOT NULL,
  `cart_rot_y` float NOT NULL,
  `cart_rot_z` float NOT NULL,
  PRIMARY KEY (`cart_mes_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
