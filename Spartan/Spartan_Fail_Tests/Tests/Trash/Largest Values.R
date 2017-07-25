
# Set a folder where the parameter value samples should be output to
FILEPATH <- "/home/jsw512/Documents/Hormone/Robust_Test_No_Attract_Time_Based/"
# Set the names of the parameters for which values are being generated for

numRows = 61
allMax <- NULL
CheckFile = NULL
rowOut <- NULL

for (i in seq(200,8000,100)){
  CheckFile <- read.csv(paste(FILEPATH, "EgSet_ATests_", i, ".csv", sep = ""))
  maxVal = 0
  for (j in 1:numRows) {
    rNumb = CheckFile[j,3]
    if (rNumb>maxVal) maxVal = rNumb;
  }
  
  rowOut = cbind(i, maxVal)
  allMax = rbind(allMax, rowOut)
}


library(ggplot2)
pdf(file = paste(FILEPATH, "/Tests/GraphMaxATest.pdf" ,sep = ""))
plot(allMax, type = "o", col = 1, xlab = "Ticks", ylab = "Atest Score", main = paste("Maximum ATest Scores for each time point", sep = ""))
grid(nx = NULL, col = "lightgray", lty = "dotted",
     lwd = par("lwd"), equilogs = TRUE)
dev.off()
