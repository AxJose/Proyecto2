# Import standard python modules.

import sys
import time
import random
import serial


# This example uses the MQTTClient instead of the REST client
from Adafruit_IO import MQTTClient

# holds the count for the feed
global run_count

ADAFRUIT_IO_USERNAME = "pas22045"
ADAFRUIT_IO_KEY      = "aio_xrua21htiqihA3oOW8zhZHJYMLli"

dashboard = MQTTClient('pas22045','aio_xrua21htiqihA3oOW8zhZHJYMLli')

# Set to the ID of the feed to subscribe to for updates.
#feedContador = 'contador'
feedEEPROM = 'EEPROM' 
feedMODO = 'MODO'
feedMemoria = 'Memoria'
feedServos = 'Servos'
feedServo1 = 'Servo1'
feedServo2 = 'Servo2'
feedServo3 = 'Servo3'
feedServo4 = 'Servo4'

mensaje = ''

# Define callback functions which will be called when certain events happen.
def connected(client):
    """Connected function will be called when the client is connected to
    Adafruit IO.This is a good place to subscribe to feed changes.  The client
    parameter passed to this function is the Adafruit IO MQTT client so you
    can make calls against it easily.
    """
    # Subscribe to changes on a feed named Counter.
    #print('Subscribing to Feed {0} and {1}'.format(feedLed, feedContador))
    
    client.subscribe(feedMODO)
    client.subscribe(feedMemoria)
    client.subscribe(feedServos)
    client.subscribe(feedServo1)
    client.subscribe(feedServo2)
    client.subscribe(feedServo3)
    client.subscribe(feedServo4)
    print('Waiting for feed data...')

def disconnected(client):
    """Disconnected function will be called when the client disconnects."""
    sys.exit(1)

def message(client, feed_id, payload):
    """Message function will be called when a subscribed feed has a new value.
    The feed_id parameter identifies the feed, and the payload parameter has
    the new value.
    """
    
    if(feed_id == feedMemoria):
        if(payload == '1'):
            arduino.write(bytes('A\n','utf-8'))
        if (payload == '2'):
           arduino.write(bytes('B\n','utf-8'))
        if(payload == '3'):
            arduino.write(bytes('C\n','utf-8'))
        if (payload == '4'):
           arduino.write(bytes('D\n','utf-8'))   
    if(feed_id == feedMODO):
        if(payload == '0'):
            arduino.write(bytes('3\n','utf-8'))
        if (payload == '1'):
           arduino.write(bytes('2\n','utf-8'))
        if(payload == '2'):
            arduino.write(bytes('1\n','utf-8'))
        if (payload == '3'):
           arduino.write(bytes('0\n','utf-8'))
        """if(feed_id == feedServos):
            if (payload == '!'):
                print('!\n')
                arduino.write(bytes('!\n','utf-8'))
            if (payload == '#'):
                print('#\n')
                arduino.write(bytes('#\n','utf-8'))"""               
    if(feed_id == feedServo1):
            print('none/n')
            #arduino.write(bytes('!\n','utf-8'))
            #time.sleep(2)
            #arduino.write(f'Servo1{payload}\n'.encode('utf-8'))
    """if(feed_id == feedServo2):
           arduino.write(bytes('8\n','utf-8'))
    if(feed_id == feedServo3):
           arduino.write(bytes('8\n','utf-8'))
    if(feed_id == feedServo4):
           arduino.write(bytes('8\n','utf-8')) """     
    print('Feed {0} received new value: {1}'.format(feed_id, payload))
    
    time.sleep(3)
    
try:
    client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

    # Setup the callback functions defined above.
    client.on_connect = connected
    client.on_disconnect = disconnected
    client.on_message = message

    # Connect to the Adafruit IO server.
    client.connect()
    client.loop_background()
              
    arduino = serial.Serial(port='COM3', baudrate =9600, timeout = 0.1)
    
    while True:    
        mensaje = arduino.readline().decode('utf-8')
        if(mensaje == 'READ\n'):
            print('Leyendo EEPROM')
            client.publish(feedMODO, 2)
        if(mensaje == 'WRITE\n'):
            print('Escribiendo en la EEPROM')
            client.publish(feedMODO, 3)
        if(mensaje == 'M_1\n'):
            print('Memoria 1')
            client.publish(feedMemoria, 1)
        if(mensaje == 'M_2\n'):
            print('Memoria 2')
            client.publish(feedMemoria, 2)
        if(mensaje == 'M_3\n'):
            print('Memoria 3')
            client.publish(feedMemoria, 3)
        if(mensaje == 'M_4\n'):
            print('Memoria 4')
            client.publish(feedMemoria, 4)             
        if(mensaje == 'MODO_M\n'):
            print('Modo manual')
            client.publish(feedMODO, 0)
        if(mensaje == 'MODO_A\n'):
            print('Modo adafruit')
            client.publish(feedMODO, 1)            
        """if(mensaje == 'Servo1\n'):
            client.publish(feedServo1, 1)
        if(mensaje == 'Servo2\n'):
            print('3')
            client.publish(feedServo2, 3)
        if(mensaje == 'Servo3\n'):
            print('4')
            client.publish(feedServo3, 4)
        if(mensaje == 'Servo4\n'):
            print('5')
            client.publish(feedServo4, 5) """     
        time.sleep(3)
        
        
except KeyboardInterrupt:
    print("Salimos del programa")
    if arduino.is_open:
        arduino.close()
    sys.exit(1)