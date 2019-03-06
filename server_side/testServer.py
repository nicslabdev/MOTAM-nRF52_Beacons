#! /usr/bin/python3

#########################################################
# Python3 Script that runs cURL and accepts some        #
# parameters in order to test NB-IoT connection server  #
# MOTAM project: https://www.nics.uma.es/projects/motam #
# Created by Manuel Montenegro, Mar 06, 2019.    V. 0.1 #
#########################################################

import subprocess
import argparse

# ==== Global variables ====

# Version of this script
scriptVersion = 0.1

address = None
device = None


# ==== Main execution ====

def main():

    # capture command line arguments
    setUpArgParser()

    print('Device: '+device)

    if device == 'trafficLight':
        cert = "certs/trafficLight.crt"
        key = "certs/trafficLight.key"
    elif device == 'infoPanel':
        cert = "certs/infoPanel.crt"
        key = "certs/infoPanel.key"
    else:
        print("Invalid device.")
        exit()

    proc = subprocess.Popen(['curl', '-k','-v','--cacert', 'certs/ca.crt', '--cert', cert, '--key', key, address], stdout=subprocess.PIPE)
    
    serverResponse = proc.stdout.readline()

    print ("--- SERVER RESPONSE: ---")
    print (str(serverResponse, "utf-8"))


# manage command line interface arguments
def setUpArgParser ( ):

    global address
    global device

    # description of the script shown in command line
    scriptDescription = 'This script runs cURL with and accepts some parameters in order to test NB-IoT connection. E.g: "python3 testServer.py --device trafficLight --address 127.0.0.1"'

    # initiate the arguments parser
    argParser = argparse.ArgumentParser(description = scriptDescription)

    # command line arguments
    argParser.add_argument("-d", "--device", help="Select the device which certificate you are going to use: 'trafficLight' or 'infoPanel'")
    argParser.add_argument("-a", "--address", help="Server IP address")
    argParser.add_argument("-v", "--version", help="Show script version", action="store_true")

    args = argParser.parse_args ()

    if not args.device or not args.address:
        print ('Not enough parameters. You have to introduce device and address. E.g: "python3 testServer.py --device trafficLight --address 127.0.0.1"')
        exit()
    
    if args.version:
        print ("Script version: ", scriptVersion)
        exit()

    address = args.address
    device = args.device

# start script
if __name__ == '__main__':
    main()