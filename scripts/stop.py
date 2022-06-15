import socket
from time import sleep
import netifaces as ni
def main():
    allips = [ip['addr'] for ip in ni.ifaddresses('wlp2s0')[ni.AF_INET]]
    print (allips)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((allips[0],0))
    msg = b'stop'

    while True:
        input("Press return to send emergency stop...")
        for i in range(10):
            sock.sendto(msg, ("255.255.255.255", 9111))
            sleep(0.01)

    sock.close()

main()
