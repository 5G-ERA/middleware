import json

import socketio
# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.




def main():
    file_path = 'connect.json'

    # Open the file in read mode
    with open(file_path, 'r') as json_file:
        # Load the JSON data into a Python dictionary
        data = json.load(json_file)

    # Now you can work with the 'data' dictionary
    host = data['address']
    netapps = data['netapps']

    #namespaces_to_connect = ["/data", "/control", "/results"]
    #io = socketio.Client()
    for netapp in netapps:
        try:
            io = socketio.Client()
            address = 'http://' + netapp + '.' + host
            io.connect(address, wait_timeout=5)
            print(f'{netapp}: Connected')
            pass
        except:
            print(f'{netapp}: Failure')
            pass



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
