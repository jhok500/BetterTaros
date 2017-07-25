NUMSAMPLES=62
NUMREPLICATES=100

# Now to run each set:
for ((i=2; i<=NUMSAMPLES; i++));
do
   ((myval = ($i - 2)*10))
   echo $myval			
   cd /home/jsw512/Documents/Hormone/D_Value_Tests_2/Sensitivity/Maze/D
   mv Param_Set_Hormone$i $myval

done
