CREATE TABLE IF NOT EXISTS `links` (
  `link_id` int(11) NOT NULL,
  `robot_id` smallint(6) NOT NULL,
  `link_name` varchar(150) NOT NULL,
  PRIMARY KEY (`link_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
