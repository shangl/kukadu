CREATE TABLE IF NOT EXISTS `controller_types` (
  `controller_id` bigint(20) unsigned NOT NULL AUTO_INCREMENT,
  `controller_implementation_class` varchar(255) NOT NULL,
  `augmented_info_table` varchar(255) NOT NULL,
  PRIMARY KEY (`controller_id`),
  UNIQUE KEY `controller_id` (`controller_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1 AUTO_INCREMENT=1 ;
