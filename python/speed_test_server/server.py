#!/usr/bin/env python3

import socket
import time

HOST = '0.0.0.0'  # Standard loopback interface address (localhost)
PORT = 9090  # Port to listen on (non-privileged ports are > 1023)
BUFF_SIZE = 1024

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    while True:
        conn, addr = s.accept()
        start = time.time()
        with conn:
            total_received = 0
            print('Connected by', addr)
            while True:
                data = conn.recv(BUFF_SIZE)
                if not data:
                    break
                total_received += len(data)
                if data[-1] == 13:
                    break
            print('total received: {}'.format(total_received))
            print('time: {}'.format(time.time() - start))


