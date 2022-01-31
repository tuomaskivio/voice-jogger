import threading
import socket


class UDPReceiver (threading.Thread):
    """Handles UDP socket receiving logic"""
    def __init__(self, q, buffersize, ip, port):
        """Initialize configuration"""
        threading.Thread.__init__(self)
        
        self.q = q
        self.close_thread = False
        self.bs = buffersize
        print('Setting up UDP socket')
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.settimeout(1)
        self.udp.bind((ip, port))
      
    def run(self):
        """Thread run function handles receiving datagram and putting in queue"""
        while not self.close_thread:
            try:
                data, _ = self.udp.recvfrom(self.bs)
            except socket.timeout:
                print("UDP socket timed out, retrying...")
                
            if 'data' in locals():
                self.q.put(data)
        
        self.udp.close()