library(spartan)
# Set a folder where the parameter value samples should be output to
FILEPATH <- "/home/jsw512/Documents/Hormone/Robust_Test"
# Set the names of the parameters for which values are being generated for
PARAMETERS <- c("D")
# The calibrated values, or baseline values, of each stated parameter
BASELINE <- c(300)
# Parameter Value Information
# You can specify this in two ways:
# 1. The minimum and maximum of each parameter, and increment over which
# sampling should be increased.
# 2. A string list of values that parameter should be assigned in sampling
# Example of 1:
PMIN <- c(0)
PMAX <- c(600)
PINC <- c(10)
PARAMVALS <- NULL
# Example of 2:
#PARAMVALS <- c("0, 50, 90","0.10, 0.3, 0.8","0.10, 0.25, 0.4",
#"0.015, 0.04, 0.08", "0.1, 0.5, 0.9","0.25, 1.25, 2.0, 3.0, 5.0")
# If using method 1, PARAMVALS must be set to NULL. If using method 2, PMIN,
# PMAX, and PINC must be set to NULL
oat_parameter_sampling(FILEPATH, PARAMETERS, BASELINE, PMIN, PMAX,
                       PINC, PARAMVALS)

