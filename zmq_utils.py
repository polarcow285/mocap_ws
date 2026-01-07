import zmq
import cv2
import base64
import numpy as np
import pickle
import blosc as bl
import threading

# ZMQ Sockets
def create_push_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind('tcp://{}:{}'.format(host, port))
    return socket

def create_pull_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.bind('tcp://{}:{}'.format(host, port))
    return socket

def create_response_socket(host, port):
    content = zmq.Context()
    socket = content.socket(zmq.REP)
    socket.bind('tcp://{}:{}'.format(host, port))
    return socket

def create_request_socket(host, port):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://{}:{}'.format(host, port))
    return socket

class ZMQKeypointSubscriber(threading.Thread):
    def __init__(self, host, port, topic):
        self._host, self._port, self._topic = host, port, topic
        self._init_subscriber()

        # Topic chars to remove
        self.strip_value = bytes("{} ".format(self._topic), 'utf-8')

    def _init_subscriber(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(self._host, self._port))
        self.socket.setsockopt(zmq.SUBSCRIBE, bytes(self._topic, 'utf-8'))

    def recv_keypoints(self, flags=None):
        if flags is None:
            raw_data = self.socket.recv()
            raw_array = raw_data.lstrip(self.strip_value)
            return pickle.loads(raw_array)
        else: # For possible usage of no blocking zmq subscriber
            try:
                raw_data = self.socket.recv(flags)
                raw_array = raw_data.lstrip(self.strip_value)
                return pickle.loads(raw_array)
            except zmq.Again:
                # print('zmq again error')
                return None
    def stop(self):
        print('Closing the subscriber socket in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()
