from bluetooth import *
from pprint import pprint
import time

def show_devices_bluetooth():
    print("performing inquiry...")
    nearby_devices = discover_devices(lookup_names=True)
    print("found %d devices" % len(nearby_devices))

    for name, addr in nearby_devices:
        print(" %s - %s" % (addr, name))

    service = find_service(address='B8:27:EB:0E:2B:9E')
    pprint(service)

def conenct_pi_bluetooth():
    client_socket = BluetoothSocket(RFCOMM)
    client_socket.connect(("B8:27:EB:0E:2B:9E", 1))
    return client_socket