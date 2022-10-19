#!/bin/bash

# *********************************
# DESCRIPTION: Query ROS and converts all the information to a json file for onboarding into the 5G-ERA Middleware DB.
# DATE: 18/10/2022
# VERSION: 1 
# *********************************

# Allow bash script to use ROS commands. Change the distro to your robot one!
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
echo "NUMBER OF ROS NODES: " "${NUMROSNODES}"
arrVar=() #List of publish topics
subServiceList=() #List of subcriber and servicec topics
subList=() #List of subcriber topics
serviceList=() #List of ros services 

#Get the information of each ros node running.
for ((q=1;q<=NUMROSNODES;q++))
do
  ROSNODE=$(rosnode list | grep "" | sed -n $q\p)
  echo "NODE NUMBER: " $q
  echo "ROSNODE NAME: " $ROSNODE
  ROSINFOTOPICS=$(rosnode info $ROSNODE)

  for item  in ${ROSINFOTOPICS};do
	 
          variable=`echo "${item}" | sed 's/rosNodes.json//g'`
          variable=`echo "${variable}" | sed 's/onboardingRobotFunciona.sh//g'`
          variable=`echo "${variable}" | sed 's/output.txt//g'`
          
          arrVar+=("${variable}") #  List of all ros nodes.
  done

  # Remove some items not neccesary
  unset -v 'arrVar[0]'
  unset -v 'arrVar[1]'
  unset -v 'arrVar[2]'

  echo "===== ALL INFO OF ROSNODE INFO ====="
  TOTAL=${#arrVar[@]}
  echo ${arrVar[@]}
  echo "===================================="

# Get index of subcription item in list arrVar.
element="Subscriptions:"
index=-1
 
for z in "${!arrVar[@]}";
do
    if [[ "${arrVar[$z]}" = "${element}" ]];
    then
        index=$z
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
 
for (( x=$index; x <= $TOTAL+2; x++ )) # index is the position in array of subscription word.
do
   subServiceList+=("${arrVar[$x]}") 
   unset -v 'arrVar[$x]'
done

echo "=======Publisher List lenght ======="
echo ${arrVar[@]}
echo "=======initial subServiceList======="
echo ${subServiceList[@]}
echo "===================================="

# Get index of contacting in list subServiceList
httpElement="contacting"
indexhttp=-1
for n in "${!subServiceList[@]}";
 do
  if [[ "${subServiceList[$n]}" = "${httpElement}" ]];
  then
     indexhttp=$n
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

# Populate with data the lists: subList and serviceList

# Clean subServiceList
for (( a=$indexhttp; a <= $TOTALSERCICELIST; a++ )) # index is the position in array of -contacting- word.
do
   # Remove all unncesary data.
   unset -v 'subServiceList[$a]'
done

for (( b=1; b<= $indexService-1; b++ ))  # adding subcriber topics to list subList
do
    if  [[ ${subServiceList[$b]} != "" ]];then
      subList+=("${subServiceList[$b]}")
    fi
done

for (( c=$indexService+1; c<= $indexhttp; c++ ))  # adding ros services  to list serviceList - for (( c=$indexService+1; c<= $indexhttp; c++ ))
do
  if  [[ ${subServiceList[$c]} != "" ]];then

    serviceList+=("${subServiceList[$c]}") # revisar que los indices no esten vacios, si lo estan, no agregar a la lista.
  fi
done


# REMOVE SUBCRIBER AND PUBLISHER WORDS FROM LISTS arrVar y subList

#Save the ros services of the node inside a temporal file. - services do not have type.
echo "============SAVE TEMP SERVICES========="
NEWTOTALSERVICE=${#serviceList[@]}
echo "Total service list length: " $NEWTOTALSERVICE
echo ${#serviceList[0]}
echo ${#serviceList[1]}
echo ${#serviceList[2]}
echo ${#serviceList[4]}
SERVICECOUNT=1 #SERVICECOUNT=2

 if [[ $NEWTOTALSERVICE == 0 ]];then
    echo "" >> services.txt
 fi

 if [[ $NEWTOTALSERVICE == 1 ]];then
          if  [[ ${subServiceList[$f]} != "" ]];then
            echo "
                  {
                  "\"""Name""\"": "\""${serviceList[$f]}"\"",
                  "\"""Description""\"": null
                  }
                " >> services.txt
          fi
 fi

 if [[ $NEWTOTALSERVICE > 0 ]];then

    if [[ ${serviceList[0]} == "None" ]];then
      echo "" >> services.txt
    fi

    for service  in ${serviceList[@]};do
        if [[ $NEWTOTALSERVICE > 1 ]];then
          
          if [[ $((SERVICECOUNT)) == $NEWTOTALSERVICE ]];then
              if  [[ $service != "" ]];then
                echo "
                    {
                    "\"""Name""\"": "\""$service"\"",
                    "\"""Description""\"": null
                    }" >> services.txt
              fi
          fi

          if [[ $((SERVICECOUNT)) != $NEWTOTALSERVICE ]];then
            if  [[ $service != "" ]];then
              echo "
                  {
                  "\"""Name""\"": "\""$service"\"",
                  "\"""Description""\"": null
                  }," >>services.txt
            fi
          fi
         SERVICECOUNT=$((SERVICECOUNT+1))
        fi

done
fi   
echo "==========SAVE TEMP SUBSCRIBERS========"
#Save the Subcriber topics of the node inside a temporal file.
NEWTOTALSUB=${#subList[@]}
TOTALDIVIDEDSUB=$(($NEWTOTALSUB / 2))
COUNTSUB=0

if [[ $NEWTOTALSUB == 0 ]];then
    echo "" >> subTopics.txt
fi
   
  if [[ $NEWTOTALSUB > 1 ]];then

   for sub  in ${subList[@]};do

        if [ $((COUNTSUB)) != $NEWTOTALSUB ] && ! [ $((COUNTSUB%2)) -eq 0 ];then # must be odd to add an entry.
          if  [[ $sub != "None" ]];then
             if [[ ${subList[$COUNTSUB+1]} == "type]" ]];then
                echo " 
                  { 
                    "\"""Name""\"": "\""${subList[$COUNTSUB]}"\"",
                    "\"""Type""\"": "\""[unknown type]"\"",
                    "\"""Description""\"": null
                  }," >> subTopics.txt
              fi
              if [[ ${subList[$COUNTSUB+1]} != "type]" ]];then
                  echo " 
                      { 
                        "\"""Name""\"": "\""${subList[$COUNTSUB+1]}"\"",
                        "\"""Type""\"": "\""${subList[$COUNTSUB]}"\"",
                        "\"""Description""\"": null
                      }," >> subTopics.txt
                  fi
            fi
        fi

        echo $TOTALDIVIDEDSUB
        if [ $((COUNTSUB)) == $NEWTOTALSUB ] && ! [ $((COUNTSUB%2)) -eq 0 ];then # must be odd to add an entry.
          if  [[ $sub != "None" ]];then
             if [[ ${subList[$COUNTSUB+1]} == "type]" ]];then
                echo " 
                  { 
                    "\"""Name""\"": "\""${subList[$COUNTSUB]}"\"",
                    "\"""Type""\"": "\""[unknown type]"\"",
                    "\"""Description""\"": null
                  }" >> subTopics.txt
              fi
              if [[ ${subList[$COUNTSUB+1]} != "type]" ]];then
                  echo " 
                      { 
                        "\"""Name""\"": "\""${subList[$COUNTSUB+1]}"\"",
                        "\"""Type""\"": "\""${subList[$COUNTSUB]}"\"",
                        "\"""Description""\"": null
                      }" >> subTopics.txt
                  fi
            fi
        fi
        

        COUNTSUB=$((COUNTSUB+1))    
      done  
  fi


echo "================SAVE TEMP PUBLISHERS================="
#Save the Publish topics of the node inside a temporal file.
NEWTOTAL=${#arrVar[@]}
TOTALDIVIDED=$(($NEWTOTAL / 2))
COUNT=0

 if [[ $NEWTOTAL == 0 ]];then
    echo "" >> PubTopics.txt
 fi
 
 if [[ $NEWTOTAL > 0 ]];then

    if [[ ${arrVar[0]} == "None" ]];then
        echo "" >> PubTopics.txt
    fi

    if [[ $TOTAL == 2 ]];then 
          if  [[ ${arrVar[0]} != "" ]];then
            echo " 
              {  1 - $COUNT
                "\"""Name""\"": "\""${arrVar[0]}"\"",
                "\"""Type""\"": "\""${arrVar[1]}"\"",
                "\"""Description""\"": null
              }" >> PubTopics.txt
          fi
        fi

        if [[ $TOTAL > 2 ]];then

          for (( r=0; r<$NEWTOTAL; r++ )) 
            do

              if [ $((COUNT)) != $TOTALDIVIDED ];then 
                  if  [[ ${arrVar[$r]} != "" ]];then
                    echo " 
                      { 
                        "\"""Name""\"": "\""${arrVar[$r+1]}"\"",
                        "\"""Type""\"": "\""${arrVar[$r+2]}"\"",
                        "\"""Description""\"": null
                      }," >> PubTopics.txt
                  fi
                fi

              if [ $((COUNT)) == $TOTALDIVIDED ];then 
                if  [[ ${arrVar[$r]} != "" ]];then
                  echo "
                    {  
                      "\"""Name""\"": "\""${arrVar[$r+1]}"\"",
                      "\"""Type""\"": "\""${arrVar[$r]}"\"",
                      "\"""Description""\"": null
                    }" >> PubTopics.txt
                fi
              fi

              COUNT=$((COUNT+1))   
             done   
        fi
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

  if [[ $NUMROSNODES > 1 ]] && [[ $q != $NUMROSNODES ]];then
  echo "{
      "\"""Name""\"": "\""$ROSNODE"\"",
      "\"""Publications""\"": [$value],
      "\"""Subscriptions""\"": [$subvalues],
      "\"""Services""\"": [$servicesvalues]
    }," >> rosNodes.json
  fi

  if [[ $NUMROSNODES > 1 ]] && [[ $q == $NUMROSNODES ]];then
  echo "{
      "\"""Name""\"": "\""$ROSNODE"\"",
      "\"""Publications""\"": [$value],
      "\"""Subscriptions""\"": [$subvalues],
      "\"""Services""\"": [$servicesvalues]
    }" >> rosNodes.json
  fi

echo "=========subServiceList================="
echo "*" +  ${subServiceList[@]}
echo "============subList============"
echo "*" + ${subList[@]}
echo "============serviceList============"
echo "*" + ${serviceList[@]}

#== delete content of list
echo "DELETE CONTENT OF ALL LISTS - RESET."
TOTAL=${#arrVar[@]}
TOTALSUBSERVICELIST=${#subServiceList[@]}
TOTALSUBLIST=${#subList[@]}
TOTALSERVICELIST=${#serviceList[@]}

for (( c=0; c<= $TOTAL+10; c++))
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
TOTALDIVIDEDSUB=0

# Remove temporal files for service, sub and pub topics
filePub="PubTopics.txt"
fileService="services.txt"
fileSub="subTopics.txt"
rm "$filePub"
rm "$fileService"
rm "$fileSub"
done

  echo " ],
  "\"""Manufacturer""\"": null,
  "\"""ManufacturerUrl"\""": null,
  "\"""RobotModel""\"": null,
  "\"""RobotStatus""\"": null,
  "\"""CurrentTaskID""\"": null,
  "\"""TaskList""\"": null,
  "\"""BatteryStatus""\"": null,
  "\"""MacAddress""\"": null,
  "\"""LocomotionSystem""\"": null,
  "\"""LocomotionTypes""\"": null,
  "\"""Sensors""\"": [],
  "\"""ActuatorModel""\"": [],
  "\"""Manipulators""\"": [],
  "\"""CPU""\"": null,
  "\"""RAM""\"": null,
  "\"""VirtualRam""\"": null,
  "\"""StorageDisk""\"": null,
  "\"""NumberCores""\"": null,
  "\"""ThreadsPerCore""\"": null,
  "\"""Questions""\"": null,
  "\"""Relations""\"": []
} " >> rosNodes.json


