filepath<-"/home/jhok500/argos3-examples/FinDat/"
AllDat = NULL
time = NULL
timeTaken = NULL
fault = NULL
diagfault = NULL
ClassPer = NULL
allResults = 0
print("start")
ReDiag <-read.csv(paste(filepath, "ReDiag.csv",sep = ""))
for (i in 1:440){
  for(j in 1:11)
  {
    #print(ReDiag[i,j])
    #print(toString(ReDiag[i,j]))
    if (identical(" Fault: 4",toString(ReDiag[i,j])) == TRUE){
      column = NULL
      print(ReDiag[i,11])
      if (identical("1",toString(ReDiag[i,11])) == TRUE) {
        allResults = allResults + 1
        #print("YIS")
      }
      
      
      #time = toString(AllDat[i,1])
      #time = substr(time, 6,11)
      #time = strtoi(time)
      #fault = toString(AllDat[i,3])
      #fault = substr(fault,9,9)
      #fault = strtoi(fault)
      #diagfault = toString(AllDat[i,j-1])
      #diagfault = strtoi(diagfault)
      #ClassPer = toString(AllDat[i,j-2])
      #ClassPer = as.numeric(ClassPer)
      #timeTaken = toString(AllDat[i,j+1])
      #timeTaken = strtoi(timeTaken)
      #print(time)
      #print(fault)
      #print(diagfault)
      #print(ClassPer)
      #print(timeTaken)
      #column = cbind(time, fault, ClassPer)
      #column = cbind(fault, diagfault)
      #allResults = rbind(allResults, column)
      
    }
    else {
      
    }
  }
}
print(allResults)
#write.csv(allResults, paste(filepath, "SensorResults.csv", sep= ""))
