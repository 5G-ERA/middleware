import pandas as pd

# Measures taken by running the following command over the output of BUT netApp.
# ros2 hz /ai_real_output_obj_det_topic_vpn

# Requirements:

# pip install pandas
# pip install openpyxl
data = pd.DataFrame(columns = ['average_rate', 'min', 'std_dev', 'window'])

fileName = "ros_hz_ai_real_output_obj_det_topic_vpn_2022-09-12-16-25.txt"

# Temp lists
average_rate = []
min_param = []
max_param = []
std_dev = []
window = []

def from_txt_to_df(nameFile,data):

    with open(nameFile) as f:
        mylist = f.read().splitlines()
    for i in mylist:
        string_value = i.rstrip('\n')
        print(string_value)
        len_string = len(string_value)
        if ("average rate" in string_value):
            # Get average value:
            semiColonmPosition = string_value.index(":")
            # +2 Because there is a 2 space gap.
            average_rate_value = string_value[semiColonmPosition+2:len_string]
            # Add the average to the average list.
            average_rate.append(average_rate_value)
        if ("min" in string_value):
            # Get min value
            minPosition = string_value.index("min:")
            maxPosition = string_value.index("max:")
            min_param_value = string_value[minPosition+5:maxPosition-2]
            # Add min value to min list.
            min_param.append(min_param_value)

            #Get max value
            maxPosition = string_value.index("max:")
            std_devPosition = string_value.index("dev:")
            max_value = string_value[maxPosition+5:std_devPosition-6]
            # Add max value to max list.
            max_param.append(max_value)

            # Get std dev value
            windowPosition = string_value.index("window:")
            std_dev_value = string_value[std_devPosition+5:windowPosition-2]
            # Add std_dev value to std_dev list.
            std_dev.append(std_dev_value)
            
            # Get window value
            window_value = string_value[windowPosition+8:len_string]
            window.append(window_value)
        
    for x in range(0,len(window)):
        
        data = data.append({'average_rate':average_rate[x], 'min':min_param[x], 'max':max_param[x], 'std_dev':std_dev[x], 'window':window[x]}, ignore_index=True)
        data.to_excel("outputMeasurements.xlsx")

        print(data.head())
    f.close()


if __name__ == '__main__':
    from_txt_to_df(fileName,data)

