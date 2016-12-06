CREATE TABLE IF NOT EXISTS `cart_mes_frc` (
  `cart_mes_id` bigint(20) NOT NULL,
  `cart_frc_x` float DEFAULT NULL,
  `cart_frc_y` float DEFAULT NULL,
  `cart_frc_z` float DEFAULT NULL,
  `cart_trq_x` float DEFAULT NULL,
  `cart_trq_y` float DEFAULT NULL,
  `cart_trq_z` float DEFAULT NULL,
  `cart_abs_frc` float DEFAULT NULL,
  PRIMARY KEY (`cart_mes_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
