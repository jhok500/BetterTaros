filepath<-"/home/jhok500/argos3-examples/FinDat/"
AllDat = NULL
time = NULL
timeTaken = NULL
fault = NULL
diagfault = NULL
ClassPer = NULL
allResults = NULL
print("start")
AllDat <-read.csv(paste(filepath, "AllData.csv",sep = ""))
for (i in 1:3245){
  for(j in 1:20)
  {
    #print(AllDat[i,j])
    if (identical(" CLASSIFIED ",toString(AllDat[i,j])) == TRUE){
      column = NULL
      time = toString(AllDat[i,1])
      time = substr(time, 6,11)
      time = strtoi(time)
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
      column = cbind(time)
      allResults = rbind(allResults, column)
      
    }
    else {
      
    }
  }
}

write.csv(allResults, paste(filepath, "ClassTimeResults.csv", sep= ""))

