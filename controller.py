#!/usr/bin/env python
import pyaudio
import socket
import sys
import RPi.GPIO as GPIO
import select

GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(17, GPIO.OUT, initial=GPIO.LOW)

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 4096

audio = pyaudio.PyAudio()

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.bind(('', 4444))
serversocket.listen(5)

def callback(in_data, frame_count, time_info, status):
    if GPIO.input(27) == 1:
        GPIO.output(17,GPIO.HIGH)
        for s in read_list[1:]:
            s.send(in_data)
        return (None, pyaudio.paContinue)
    else:
        GPIO.output(17,GPIO.LOW)
        return (None, pyaudio.paContinue)


# start Recording
stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True,input_device_index=2, frames_per_buffer=CHUNK, stream_callback=callback)
# stream.start_stream()

read_list = [serversocket]
print "recording..."

try:
    while True:
        readable, writable, errored = select.select(read_list, [], [])
        for s in readable:
            if s is serversocket:
                print 'stream'
                (clientsocket, address) = serversocket.accept()
                read_list.append(clientsocket)
                print "Connection from", address
            else:
                print 'else statemetn'
                data = s.recv(1024)
                if not data:
                    print 'remove'
                    read_list.remove(s)
except KeyboardInterrupt:
    pass


print "finished recording"

serversocket.close()
# stop Recording
stream.stop_stream()
stream.close()
audio.terminate()

