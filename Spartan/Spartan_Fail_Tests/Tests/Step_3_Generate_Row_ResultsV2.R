filepath<-"/home/jhok500/argos3-examples/Spartan/Spartan_Fail_Data_Struct/"

trials = 110
set = 20

for (i in 1:set){
  rowToExtract = i
    for(j in 1:trials)
    {
      resultFile <-read.csv(paste(filepath,trials,"/",i, "/", j, "/ClassFailPercent.csv",sep=""),header=T)
      colnames(resultFile) <-c("Seed","Memory_Percentage" ,"Failure_Percentage")
      write.csv(resultFile[rowToExtract,],file=paste(filepath,trials,"/",i, "/", j, "/ClassFailPercent_Values.csv",sep=""),row.names=F,quote=F)
   }
}

