import pandas as pd

# Measures taken by running ping command.
# Requirements:

# pip install pandas
# pip install openpyxl
data = pd.DataFrame(columns = ['IP', 'icmp_seq', 'ttl', 'time'])

fileName = "from_netapp_to_robot_2022-09-12-55.txt"

# Temp lists
ip = []
ttl = []
icmp_seq = []
time = []


def from_txt_to_df(nameFile,data):

    with open(nameFile) as f:
        mylist = f.read().splitlines()
    for i in mylist:
        string_value = i.rstrip('\n')
        print(string_value)
        len_string = len(string_value)
        if ("from" in string_value):
            # IP:
            fromPosition = string_value.index("from")
            icmp_seqPosition = string_value.index("icmp_seq")
            ip_value = string_value[fromPosition+5:icmp_seqPosition-2]
            #print(ip_value)
            ip.append(ip_value)

            # icmp_seq
            ttlPosition = string_value.index("ttl")
            ttl_value = string_value[icmp_seqPosition+9:ttlPosition]
            icmp_seq.append(ttl_value)

            # ttl
            timePosition = string_value.index("time")
            ttl_value = string_value[ttlPosition+4:timePosition]
            ttl.append(ttl_value)

            # time
            time_value = string_value[timePosition+5:len_string-3]
            time.append(time_value)            
        
    for x in range(0,len(time)):
        
        data = data.append({'IP':ip[x], 'icmp_seq':icmp_seq[x], 'ttl':ttl[x], 'time':time[x]}, ignore_index=True)
        data.to_excel("outputMeasurementsPing.xlsx")

      
    f.close()


if __name__ == '__main__':
    from_txt_to_df(fileName,data)

