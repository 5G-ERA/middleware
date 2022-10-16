#!/bin/bash

# Allow bash script to use ROS commands. Change the distro to your robot one.
source /opt/ros/indigo/setup.bash

# Add begining of json file
echo "{
  "\"""Id""\"": null,
  "\"""Name""\"": null,
  ""\""ROSRepo""\"": null,
  "\"""ROSNodes""\"": [" >> rosNodes.json


#Get a list of ros nodes running
NUMROSNODES=$(rosnode list | grep -c "" )
ROSNODE=$(rosnode list | grep "" | sed -n 1p)
echo $ROSNODE
echo "${NUMROSNODES}"
arrVar=() #List of publish topics
subServiceList=() #List of subcriber and servicec topics
subList=() #List of subcriber topics
serviceList=() #List of ros services 

#Get the information of each ros node running.
for ((q=1;q<=NUMROSNODES;q++))
do
  ROSNODE=$(rosnode list | grep "" | sed -n $q\p)
  echo "NUMBER NODE "$q
  echo "ROSNODE INFO "$ROSNODE
  ROSINFOTOPICS=$(rosnode info $ROSNODE)

  for item  in ${ROSINFOTOPICS};do
        if [[ "${item}" == *@("/"|"Publications"|"Subscriptions"|"Services"|"contacting")* ]]; then 
	   arrVar+=("${item}") #  List of all ros nodes.
  	  # echo "${item}" >> pubTopics.json
	fi
  done

  echo "======="
  unset -v 'arrVar[0]'
  TOTAL=${#arrVar[@]}
  echo ${arrVar[@]}
  echo "======"

# Get index of subcription item in list arrVar.
element="Subscriptions:"
index=-1
 
for i in "${!arrVar[@]}";
do
    if [[ "${arrVar[$i]}" = "${element}" ]];
    then
        index=$i
        break
    fi
done
 
if [ $index -gt -1 ];
then
    echo "Index of Element Subscription in Array is : $index"
else
    echo "Element is not in Array."
fi

# Remove all topics but the publisher ones
 
 for (( c=$index; c <= $TOTAL; c++ )) # index is the position in array of subscription word.
do
   echo "unset index $c"
   subServiceList+=("${arrVar[$c]}") 
   unset -v 'arrVar[$c]'
done

unset -v 'arrVar[1]'
echo "=======arrVar====="
echo ${arrVar[@]}
echo "==========initial subServiceList==========="
echo ${subServiceList[@]}
echo "=========================================="

# Get index of contacting in list subServiceList
httpElement="contacting"
indexhttp=-1
for o in "${!subServiceList[@]}";
 do
  if [[ "${subServiceList[$o]}" = "${httpElement}" ]];
  then
     indexhttp=$o
     break
  fi
done
if [ $indexhttp -gt -1 ];
then
    echo "Index of Element contacting in Array is : $indexhttp"
else
    echo "Element is not in Array."
fi

# Get index of Service item in list
serviceElement="Services:"
indexService=-1

for o in "${!subServiceList[@]}";
 do
  if [[ "${subServiceList[$o]}" = "${serviceElement}" ]];
  then
     indexService=$o
     break
  fi
done
if [ $indexService -gt -1 ];
then
    echo "Index of Element service in Array is : $indexService"
else
    echo "Element is not in Array."
fi

echo ===TOTAL LENGTH OF subServiceList =====
TOTALSERCICELIST=${#subServiceList[@]} # Get the total lenght of list: subServiceList
echo $TOTALSERCICELIST
echo ========================================

# Populate with data the lists: subList and serviceList

for (( c=$indexhttp; c <= $TOTALSERCICELIST; c++ )) # index is the position in array of -contacting- word.
do
   echo "unset subServiceList index $c"
   unset -v 'subServiceList[$c]'
done

for (( c=1; c<= $indexService-1; c++ ))  # adding subcriber topics to list subList
do
  echo "adding subscriber topic"
  subList+=("${subServiceList[$c]}")
done

for (( c=$indexService+1; c<= $indexhttp; c++ ))  # adding ros services  to list serviceList
do
  echo "adding ros service "
  serviceList+=("${subServiceList[$c]}")
done

#Save the ros services of the node inside a temporal file. - services do not have type.
echo "==============SAVE TEMP SERVICES=============="
NEWTOTALSERVICE=${#serviceList[@]}
echo "total service list length "+$NEWTOTALSERVICE
SERVICECOUNT=2

 if [[ $NEWTOTALSERVICE == 0 ]];then
    echo "" >> services.txt
 fi

 if [[ $NEWTOTALSERVICE > 0 ]];then

 for (( c=0; c<=NEWTOTALSERVICE-2; c++))
  do
   echo "adding new service to temporal file..."
   echo "SERVICECOUNT "+$SERVICECOUNT
    if [[ $NEWTOTALSERVICE == 1 ]];then
     echo "
          {
          "\"""Name""\"": "\""${serviceList[$c]}"\"",
          "\"""Description""\"": null
          }
         " >> services.txt
    fi

   if [[ $NEWTOTALSERVICE > 1 ]];then
    
    if [[ $((SERVICECOUNT)) == $NEWTOTALSERVICE ]];then
      echo "
           {
          "\"""Name""\"": "\""${serviceList[$c]}"\"",
          "\"""Description""\"": null
           }" >> services.txt
    fi

    if [[ $((SERVICECOUNT)) != $NEWTOTALSERVICE ]];then
      echo "
           {
          "\"""Name""\"": "\""${serviceList[$c]}"\"",
          "\"""Description""\"": null
           }," >>services.txt
    fi
   SERVICECOUNT=$((SERVICECOUNT+1))
  fi
 
done
fi   
echo "=============SAVE TEMP SUBSCRIBERS==============="
#Save the Subcriber topics of the node inside a temporal file.
NEWTOTALSUB=${#subList[@]}
TOTALDIVIDEDSUB=$(($NEWTOTALSUB / 2))
COUNTSUB=0
echo $NEWTOTALSUB + "TOTAL"
echo $TOTALDIVIDEDSUB + "TOTAL DIVIDED"

if [[ $NEWTOTALSUB == 0 ]];then
    echo "" >> subTopics.txt
fi

echo "================SAVE TEMP PUBLISHERS================="
#Save the Publish topics of the node inside a temporal file.
NEWTOTAL=${#arrVar[@]}
TOTALDIVIDED=$(($NEWTOTAL / 2))
COUNT=0
echo $NEWTOTAL + "TOTAL"
echo $TOTALDIVIDED + "TOTAL DIVIDED"

 if [[ $NEWTOTAL == 0 ]];then
    echo "" >> PubTopics.txt.txt
 fi
 
 if [[ $NEWTOTAL > 0 ]];then

 for (( c=2; c<$TOTALDIVIDED+3; c++ ))
  do
    echo "COUNTER "+$COUNT
    if [[ $TOTAL == 2 ]];then 
      echo " 
        {  1 - $COUNT
          "\"""Name""\"": "\""${arrVar[$c]}"\"",
          "\"""Type""\"": "\""${arrVar[$c+1]}"\"",
          "\"""Description""\"": null
        }" >> PubTopics.txt
    fi
    if [[ $TOTAL > 2 ]];then
      echo "result"+$(($c%2))
      if [[ $c == 2 ]];then
         echo ${arrVar[$c]}
         echo ${arrVar[$c+1]}
         echo "
          { 
            "\"""Name""\"": "\""${arrVar[$c]}"\"",
            "\"""Type""\"": "\""${arrVar[$c+1]}"\"",
            "\"""Description""\"": null
          }," >> PubTopics.txt

      fi
      if ! [ $(($c%2)) -eq 0 ] && [ $((COUNT)) != $TOTALDIVIDED ];then #Check if odd number in TOTAL
         echo "Odd number"
         echo ${arrVar[$c+1]}
         echo ${arrVar[$c+2]}
      	 echo " 
          { 
            "\"""Name""\"": "\""${arrVar[$c+1]}"\"",
            "\"""Type""\"": "\""${arrVar[$c+2]}"\"",
            "\"""Description""\"": null
          }," >> PubTopics.txt
      fi

      if ! [ $(($c%2)) -eq 0 ] && [ $((COUNT)) == $TOTALDIVIDED ];then #Check if odd number in TOTAL
         echo "Odd number"
         echo ${arrVar[$c+1]}
         echo ${arrVar[$c+2]}
         echo "
          {  
            "\"""Name""\"": "\""${arrVar[$c+1]}"\"",
            "\"""Type""\"": "\""${arrVar[$c]}"\"",
            "\"""Description""\"": null
          }" >> PubTopics.txt
      fi

      COUNT=$((COUNT+1))      
    fi
  
 done
 fi
 # Save the info of all the node in json format to the final file - rosNodes.json

  #  Check if there is only 1 ros node. 
  value=$(<PubTopics.txt)
  servicesvalues=$(<services.txt)
  subvalues=$(<subTopics.txt)

  if [[ $NUMROSNODES == 1 ]];then
  echo "{
      "\"""Name""\"": "\""$ROSNODE"\"",
      "\"""Publications""\"": [$value],
      "\"""Subscriptions""\"": [$subvalues],
      "\"""Services""\"": [$servicesvalues]
    }" >> rosNodes.json
  fi

  if [[ $NUMROSNODES > 1 ]];then
  echo "{
      "\"""Name""\"": "\""$ROSNODE"\"",
      "\"""Publications""\"": [$value],
      "\"""Subscriptions""\"": [],
      "\"""Services""\"": [$servicesvalues]
    }," >> rosNodes.json
  fi



echo "=========subServiceList================="
echo "*" +  ${subServiceList[@]}
echo "============subList============"
echo "*" + ${subList[@]}
echo "============serviceList============"
echo "*" + ${serviceList[@]}

#== delete content of list

TOTAL=${#arrVar[@]}
TOTALSUBSERVICELIST=${#subServiceList[@]}
TOTALSUBLIST=${#subList[@]}
TOTALSERVICELIST=${#serviceList[@]}

for (( c=0; c<= $TOTAL+1; c++))
 do
  #echo "unset arrVar list index $c"
  unset -v 'arrVar[$c]'
done

for (( c=0; c<= $TOTALSUBSERVICELIST; c++))
 do
  #echo "unset subServiceList list index $c"
  unset -v 'subServiceList[$c]'
done

for (( c=0; c<= $TOTALSUBLIST; c++))
 do
  #echo "unset subServiceList list index $c"
  unset -v 'subList[$c]'
done

for (( c=0; c<= $TOTALSERVICELIST; c++))
 do
  #echo "unset subServiceList list index $c"
  unset -v 'serviceList[$c]'
done

#==Check all list are empty for next iteration
echo "==========CHECK EMPTY LIST==========="
echo "===========subServiceList=========="
echo "*" + ${subServiceList[@]}
echo "============subList============"
echo "*" + ${subList[@]}
echo "============serviceList============"
echo "*" + ${serviceList[@]}
echo "==========arrVar================"
echo "*" + ${arrVar[@]}


#== reset index variable
index=0
# reset counter
COUNT=0
indexhttp=0
indexService=0
SERVICECOUNT=0
TOTALSUBSERVICELIST=0
TOTALSUBLIST=0
TOTALSERVICELIST=0
NEWTOTALSERVICE=0
TOTALSERCICELIST=0

# Remove temporal files for service, sub and pub topics
filePub="PubTopics.txt"
fileService="services.txt"
fileSub="subTopics.txt"
rm "$filePub"
rm "$fileService"
rm "$fileSub"
done



