CREATE TABLE IF NOT EXISTS `software_classes` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `module_id` smallint(6) NOT NULL,
  `name` varchar(255) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
