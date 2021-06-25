#!/usr/bin/env python    
# -*- coding: utf-8 -*

import socket
import time
import json

def tcp_sim():

    sever_port=('127.0.0.1', 54320)

    try:
        # create an AF_INET, STREAM socket (TCP)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except:
        print('Failed to create a socket. ')
        sys.exit()
    print('Socket Created :)')
    
    sock.connect(sever_port)
    print("start a client")
    time.sleep(0.01)

    s="start"
    s = bytes(s, encoding = "utf8")
    sock.send(s)
    print(s)


    while 1:
        buf = sock.recv(1000)
        #print(buf)
        buf_l = json.loads(buf)
        print(buf_l)
        control_signal=buf_l[0]*1
        # s=str(control_signal)
        s = bytes(str(control_signal), encoding = "utf8")
        sock.send(s)
        print(s)
        #sock.send(control_signal)
        #print(control_signal)
    sock.close()




if __name__ == '__main__':
  
    tcp_sim()
