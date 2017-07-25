library(spartan)
# Folder containing the example simulation results. Make sure the folder is unzipped
FILEPATH <- "/home/jsw512/Documents/Hormone/D_Value_Tests_2/Sensitivity/Maze/"
# Array of the parameters to be analysed.
# Note only two of the six here for download size reasons
PARAMETERS <- c("D")
# Similar to the sampling function discussed above, there are two ways to specify
# parameter value information in the analysis. Ensure you are using the appropriate
# method, setting these to NULL if using the alternative (see comments in sampling
# function description).
# Method 1:
PMIN <- c(0)
PMAX <- c(600)
PINC <- c(10)
PARAMVALS <- NULL
BASELINE <- c(300)
MEASURES <- c("Area_Mapped")
# What each measure represents. Used in graphing results
MEASURE_SCALE <- c("m^2")
RESULTFILENAME <- "Hormone_Test.csv"
OUTPUTCOLSTART <- 2
OUTPUTCOLEND <- 2
ALTERNATIVEFILENAME <- NULL
# Either 1: The name of the CSV file containing all simulation output (see description
# that follows in this section) or name to give the summary file that spartan generates
CSV_FILE_NAME <- "SummaryFile.csv"
# Number of replicate runs performed for each parameter value set
NUMRUNSPERSAMPLE <- 100
# The results of the A-Test comparisons of each parameter value against that of the
# parameters baseline value are output as a file. This sets the name of this file.
# Current versions of spartan output this to a CSV file
ATESTRESULTSFILENAME <- "EgSet_ATests.csv"
# A-Test result value either side of 0.5 at which the difference between two sets of
# results is significant
ATESTSIGLEVEL <- 0.23
# Timepoints being analysed. Must be NULL if no timepoints being analysed, or else
# be an array of timepoints. Scale sets the measure of these timepoints
TIMEPOINTS <- NULL; TIMEPOINTSCALE <- "Ticks"

for (m in seq(200,8000,100)){
  TIMEPOINTS = cbind(TIMEPOINTS,m)
}
# Example Timepoints, if being used:
oat_processParamSubsets(FILEPATH, PARAMETERS, NUMRUNSPERSAMPLE, MEASURES,RESULTFILENAME, ALTERNATIVEFILENAME, OUTPUTCOLSTART, OUTPUTCOLEND,CSV_FILE_NAME, BASELINE, PMIN, PMAX, PINC, PARAMVALS,TIMEPOINTS, TIMEPOINTSCALE)
oat_csv_result_file_analysis(FILEPATH, CSV_FILE_NAME, PARAMETERS, BASELINE,
                             MEASURES, ATESTRESULTSFILENAME, PMIN, PMAX, PINC,
                             PARAMVALS, TIMEPOINTS, TIMEPOINTSCALE)
oat_graphATestsForSampleSize(FILEPATH, PARAMETERS, MEASURES, ATESTSIGLEVEL,
                             ATESTRESULTSFILENAME, BASELINE, PMIN, PMAX, PINC, PARAMVALS, TIMEPOINTS,
                             TIMEPOINTSCALE)
oat_plotResultDistribution(FILEPATH, PARAMETERS, MEASURES, MEASURE_SCALE,
                           CSV_FILE_NAME, BASELINE, PMIN, PMAX, PINC, PARAMVALS, TIMEPOINTS,
                           TIMEPOINTSCALE)
