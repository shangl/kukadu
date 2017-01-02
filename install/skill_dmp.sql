CREATE TABLE IF NOT EXISTS `skill_dmp` (
  `skill_id` bigint(20) NOT NULL,
  `tau` double NOT NULL,
  `az` double NOT NULL,
  `bz` double NOT NULL,
  `ax` double NOT NULL,
  `step_size` double NOT NULL,
  `tol_abs_err` double NOT NULL,
  `tol_rel_err` double NOT NULL,
  `tmax` double NOT NULL,
  PRIMARY KEY (`skill_id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
