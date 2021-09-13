CREATE TABLE IF NOT EXISTS `skills` (
  `skill_id` bigint(20) unsigned NOT NULL AUTO_INCREMENT,
  `label` varchar(255) NOT NULL,
  `controller_type` int(11) NOT NULL,
  PRIMARY KEY (`skill_id`),
  UNIQUE KEY `id` (`skill_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1 AUTO_INCREMENT=1 ;
