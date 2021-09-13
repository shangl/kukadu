CREATE TABLE IF NOT EXISTS `controller_types` (
  `controller_id` bigint(20) unsigned NOT NULL AUTO_INCREMENT,
  `controller_implementation_class` varchar(255) NOT NULL,
  `augmented_info_table` varchar(255) DEFAULT NULL,
  `is_playable` tinyint(1) NOT NULL DEFAULT '0',
  PRIMARY KEY (`controller_id`),
  UNIQUE KEY `controller_id` (`controller_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1 AUTO_INCREMENT=1 ;
