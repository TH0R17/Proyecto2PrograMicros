import sys
import time
import serial
from Adafruit_IO import MQTTClient

# Configuración de Adafruit IO
ADAFRUIT_IO_USERNAME = "ThorUVG"
ADAFRUIT_IO_KEY = ""

# Feeds a los que se suscribe (recibe)
FEEDS_RECEIVE = ['ServoCodo', 'ServoGarra', 'Modo']

# Feed al que se publica (envía desde Arduino)
FEED_SEND = 'ServoHombro'

# Callback al conectar
def connected(client):
    for feed in FEEDS_RECEIVE:
        print(f'Subscribiendo al feed: {feed}')
        client.subscribe(feed)
    print('Esperando datos...')

# Callback al desconectar
def disconnected(client):
    sys.exit(1)

# Callback al recibir mensaje
def message(client, feed_id, payload):
    print(f'Feed {feed_id} recibió: {payload}')
    
    # Validar que el payload sea numérico (entero o decimal)
    if payload.replace('.', '', 1).isdigit():
        miarduino.write(bytes(payload + '\n', 'utf-8'))  # Enviar solo el número
        print(f'Enviado al Arduino: {payload}')
    else:
        print(f'El payload "{payload}" no es numérico.')

# Inicializar cliente MQTT y puerto serial
client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)
miarduino = serial.Serial(port="COM8", baudrate=9600, timeout=0.1)

# Asignar callbacks
client.on_connect = connected
client.on_disconnect = disconnected
client.on_message = message

# Conectar a Adafruit IO
client.connect()
client.loop_background()

# Bucle principal
while True:
    # Leer del Arduino y publicar si hay datos
    if miarduino.in_waiting > 0:
        try:
            dato = miarduino.readline().decode('latin-1').strip()
            if dato:
                print(f'Dato desde Arduino: {dato}')
                client.publish(FEED_SEND, dato)
        except Exception as e:
            print(f'Error al leer del Arduino: {e}')
    
    time.sleep(1)
