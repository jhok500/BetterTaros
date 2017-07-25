num=1
set=20

cd /home/jhok500/argos3-examples/Spartan/Spartan_Fail_Data_Struct
mkdir $num
# Now to run each set:
for ((i=1; i<=trials; i++));
do
   #cd /home/jhok500/argos3-examples/Spartan/Spartan_Fail_Data/$i
   for ((j=1; j<=set; j++));
   do	
       mkdir $num/$j 	
       cp ../Spartan_Fail_Data/$i/ClassFailPercent.csv $num/$j/ClassFailPercent.csv
   done
done
