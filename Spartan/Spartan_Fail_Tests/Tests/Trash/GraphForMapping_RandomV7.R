FolderTarget <- "/home/jsw512/Documents/Hormone/D_Value_Tests_2/Sensitivity/Maze/D/"

noRows = 80
noRuns = 100
#numRobots = 10
robColours = c('red','blue','green','black','grey','orange','coral','purple','cyan', 'chocolate')
#robID = 0

AllResults = NULL
k = 0
max = 0
min = 300
maxParam = 0
minParam = 0
Params = NULL
for(i in seq(0,600,10)){
  Result = NULL
  ParamResults = NULL
  for (j in 1:noRuns){
    Result <- read.csv(paste(FolderTarget, i , "/", j,"/Hormone_Test.csv",sep = "" ))
    ParamResults <- cbind(ParamResults, Result[,2])
  }
  end <- rowMeans(ParamResults)[80]
  if (end > max){
    maxParam = i
    max = end
  }
  if (end <=min){
    minParam = i
    min = end
  }
  AllResults = cbind(AllResults, rowMeans(ParamResults))
  Params <- rbind(Params, i)
  k = k + 1
}  

AllResults = cbind(Result[,1], AllResults)

library(ggplot2)
pdf(file = paste(FolderTarget, "/GraphHvsR.pdf" ,sep = ""))
plot(AllResults[,1], AllResults[,2], type = "l", col = 1, ylim = c(0, 299), xlab = "Ticks", ylab = "Area Mapped", main = paste("Average Area Mapped for 100 Random Movement simulations", sep = ""))
for (l in 1:(k-1)) {
  lines(AllResults[,1], AllResults[,(l+2)], col = l)
}
dev.off()

library(ggplot2)
pdf(file = paste(FolderTarget, "/GraphHvsR_finalVals.pdf" ,sep = ""))
plot(Params[1], AllResults[80,2], type = "p", col = 2, ylim = c(0, 299), xlim = c(0, i), xlab = "D", ylab = "Area Mapped", main = paste("Final Area Mapped for 100 Random Movement simulations", sep = ""))
for (h in 1:(k-1)) {
  points(Params[h+1,1], AllResults[80,(h+2)], col = 2)
}
dev.off()

