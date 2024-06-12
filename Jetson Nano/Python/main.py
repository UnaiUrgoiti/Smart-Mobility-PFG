import time
import serial
import paho.mqtt.client as mqtt
from confluent_kafka import Consumer, KafkaError
import json
import asyncio
import smbus2
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
from collections import deque

# --------------------------------------------------------------------------------------------------------------
# LWNX library functions
# --------------------------------------------------------------------------------------------------------------
packetParseState = 0
packetPayloadSize = 0
packetSize = 0
packetData = []

def createCrc(data):
    crc = 0
    for i in data:
        code = crc >> 8
        code ^= int(i)
        code ^= code >> 4
        crc = crc << 8
        crc ^= code
        code = code << 5
        crc ^= code
        code = code << 7
        crc ^= code
        crc &= 0xFFFF
    return crc

def buildPacket(command, write, data=[]):
    payloadLength = 1 + len(data)
    flags = (payloadLength << 6) | (write & 0x1)
    packetBytes = [0xAA, flags & 0xFF, (flags >> 8) & 0xFF, command]
    packetBytes.extend(data)
    crc = createCrc(packetBytes)
    packetBytes.append(crc & 0xFF)
    packetBytes.append((crc >> 8) & 0xFF)
    return bytearray(packetBytes)

def parsePacket(byte):
    global packetParseState, packetPayloadSize, packetSize, packetData
    if packetParseState == 0:
        if byte == 0xAA:
            packetParseState = 1
            packetData = [0xAA]
    elif packetParseState == 1:
        packetParseState = 2
        packetData.append(byte)
    elif packetParseState == 2:
        packetParseState = 3
        packetData.append(byte)
        packetPayloadSize = (packetData[1] | (packetData[2] << 8)) >> 6
        packetPayloadSize += 2
        packetSize = 3
        if packetPayloadSize > 1019:
            packetParseState = 0
    elif packetParseState == 3:
        packetData.append(byte)
        packetSize += 1
        packetPayloadSize -= 1
        if packetPayloadSize == 0:
            packetParseState = 0
            crc = packetData[packetSize - 2] | (packetData[packetSize - 1] << 8)
            verifyCrc = createCrc(packetData[0:-2])
            if crc == verifyCrc:
                return True
    return False

def waitForPacket(port, command, timeout=1):
    global packetParseState, packetPayloadSize, packetSize, packetData
    packetParseState = 0
    packetData = []
    packetPayloadSize = 0
    packetSize = 0
    endTime = time.time() + timeout
    while True:
        if time.time() >= endTime:
            return None
        c = port.read(1)
        if len(c) != 0:
            b = ord(c)
            if parsePacket(b):
                if packetData[3] == command:
                    return packetData

def readStr16(packetData):
    str16 = ''
    for i in range(0, 16):
        if packetData[4 + i] == 0:
            break
        else:
            str16 += chr(packetData[4 + i])
    return str16

def readSignalData(packetData):
    signalFront = packetData[4 + 0] << 0
    signalFront |= packetData[4 + 1] << 8
    signalFront |= packetData[4 + 2] << 16
    signalFront |= packetData[4 + 3] << 24
    signalFront /= 1000.0
    signalRear = packetData[8 + 0] << 0
    signalRear |= packetData[8 + 1] << 8
    signalRear |= packetData[8 + 2] << 16
    signalRear |= packetData[8 + 3] << 24
    signalRear /= 1000.0
    currentDistance = packetData[12 + 0] << 0
    currentDistance |= packetData[12 + 1] << 8
    currentDistance |= packetData[12 + 2] << 16
    currentDistance |= packetData[12 + 3] << 24
    currentDistance /= 1000.0
    signalWidth = signalRear - signalFront
    return currentDistance, signalWidth

def executeCommand(port, command, write, data=[], timeout=1):
    packet = buildPacket(command, write, data)
    retries = 4
    while retries > 0:
        retries -= 1
        port.write(packet)
        response = waitForPacket(port, command, timeout)
        if response is not None:
            return response
    raise Exception('LWNX command failed to receive a response.')

def print_product_information(port):
    response = executeCommand(port, 0, 0, timeout=0.1)
    print('Product: ' + readStr16(response))
    response = executeCommand(port, 2, 0, timeout=0.1)
    print('Firmware: {}.{}.{}'.format(response[6], response[5], response[4]))
    response = executeCommand(port, 3, 0, timeout=0.1)
    print('Serial: ' + readStr16(response))

def set_default_distance_output(port, use_last_return=False):
    if use_last_return:
        executeCommand(port, 27, 1, [1, 1, 0, 0])
    else:
        executeCommand(port, 27, 1, [8, 1, 0, 0])

def wait_for_reading(port, timeout=1):
    response = waitForPacket(port, 44, timeout)
    if response is None:
        return -1, 0
    distance = (response[4] << 0 | response[5] << 8) / 100.0
    yaw_angle = response[6] << 0 | response[7] << 8
    if yaw_angle > 32000:
        yaw_angle = yaw_angle - 65535
    yaw_angle /= 100.0
    return distance, yaw_angle

# --------------------------------------------------------------------------------------------------------------
# MQTT Connection with Paho
# --------------------------------------------------------------------------------------------------------------

mqtt_broker = "192.168.0.23"  # IP of the Host
mqtt_port = 1883
topic_input = "lidar/input"

anguloObj = 0
cola = 0

def on_message(client, userdata, msg):
    global anguloObj, cola
    temp = float(msg.payload)
    if temp != anguloObj:
        anguloObj = temp
        cola += 1
        print("Recibido Angulo:", anguloObj, "con cola:", cola)

client = mqtt.Client()
client.on_message = on_message

client.connect(mqtt_broker, mqtt_port, 60)
client.subscribe(topic_input)

print("Esperando mensajes MQTT...")
client.loop_start()

# --------------------------------------------------------------------------------------------------------------
# Kafka Configuration
# --------------------------------------------------------------------------------------------------------------

kafka_bootstrap_servers = '192.168.0.23:9092'  # Change this to your Kafka servers
kafka_topic = 'my-topic'  # Change this to your Kafka topic name

kafka_consumer_conf = {
    'bootstrap.servers': kafka_bootstrap_servers,
    'group.id': 'lidar-group',
    'auto.offset.reset': 'latest'
}

consumer = Consumer(kafka_consumer_conf)
consumer.subscribe([kafka_topic])

print("Esperando mensajes Kafka...")

# --------------------------------------------------------------------------------------------------------------
# Main application.
# --------------------------------------------------------------------------------------------------------------
print('Running LWNX sample.')

serialPortName = '/dev/ttyACM0'
serialPortBaudRate = 921600
port = serial.Serial(serialPortName, serialPortBaudRate, timeout=0.1)

print_product_information(port)

print('Set update rate')
executeCommand(port, 66, 1, [10])
executeCommand(port, 85, 1, [5, 0, 0, 0])
executeCommand(port, 98, 1, [12, 0, 0, 0])
executeCommand(port, 99, 1, [12, 0, 0, 0])

print('Set streaming data')
set_default_distance_output(port)
executeCommand(port, 30, 1, [5, 0, 0, 0])

print('Wait for incoming data')

def calcular_angulo_x(topleftx, bottomrightx, ancho_total):
    centro_x = (topleftx + bottomrightx) / 2
    angulo = ((centro_x / ancho_total) * 20) - 10
    return angulo

objetos_detectados = {}
objetos_lock = threading.Lock()

def calcular_distancia_media():
    with objetos_lock:
        if not objetos_detectados:
            return None
        suma_distancias = sum(obj['distancia'] for obj in objetos_detectados.values() if obj['distancia'] is not None)
    return suma_distancias / len(objetos_detectados)

kafka_queue = asyncio.Queue()

I2C_SLAVE_ADDRESS = 0x28
I2C_OLED_ADDRESS = 0x3C
bus = smbus2.SMBus(1)
serial = i2c(port=1, address=I2C_OLED_ADDRESS)
oled = ssd1306(serial, width=128, height=64)

def enviar_datos_i2c(matriz_datos, rojo, verde, azul):
    data = [
        matriz_datos & 0xFF,
        (matriz_datos >> 8) & 0xFF,
        (matriz_datos >> 16) & 0xFF,
        (matriz_datos >> 24) & 0xFF,
        rojo,
        verde,
        azul
    ]
    bus.write_i2c_block_data(I2C_SLAVE_ADDRESS, 0, data)
    print("Datos enviados al esclavo I2C")
    mostrar_datos_oled(data)

def mostrar_datos_oled(data):
    image = Image.new('1', (oled.width, oled.height), "black")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    draw.text((0, 0), 'Datos enviados:', font=font, fill=255)
    draw.text((0, 10), 'Direccion: 0x{:02X}'.format(I2C_SLAVE_ADDRESS), font=font, fill=255)
    draw.text((0, 20), 'Matriz: 0x{:08X}'.format(
        data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)), font=font, fill=255)
    draw.text((0, 30), 'Rojo: 0x{:02X}'.format(data[4]), font=font, fill=255)
    draw.text((0, 40), 'Verde: 0x{:02X}'.format(data[5]), font=font, fill=255)
    draw.text((0, 50), 'Azul: 0x{:02X}'.format(data[6]), font=font, fill=255)
    oled.display(image)

async def procesar_mensajes_kafka():
    while True:
        msg = await asyncio.to_thread(consumer.poll, timeout=1.0)
        if msg is None:
            continue
        if msg.error():
            if msg.error().code() == KafkaError._PARTITION_EOF:
                continue
            else:
                print(msg.error())
            break
        await kafka_queue.put(msg)
        print("Mensaje Kafka procesado y puesto en la cola")

  async def procesar_datos_lidar():
      while True:
          distance, yaw_angle = wait_for_reading(port)
          if distance != -1 and distance <= 30:
              client.publish("lidar/output/distancia", distance, 0)
              client.publish("lidar/output/angulo", yaw_angle, 0)
              with objetos_lock:
                  for obj_id, obj_data in objetos_detectados.items():
                      if abs(obj_data['angulo_x'] - yaw_angle) <= 0.5:
                          obj_data['distancia'] = distance
              distancia_media = calcular_distancia_media()
              if distancia_media and abs(anguloObj - yaw_angle) <= 0.5 and cola >= 1:
                  if distancia_media <= 20:
                      client.publish("lidar/output/distanciaMedia", distancia_media, 0)
                      cola -= 1
              print(f"Distancia: {distance}, Ángulo: {yaw_angle}, Distancia media: {distancia_media}, Cola: {cola}")
          await asyncio.sleep(0.1)

async def actualizar_objetos():
    while True:
        msg = await kafka_queue.get()
        print("Mensaje Kafka recibido")
        try:
            message = json.loads(msg.value().decode('utf-8'))
            bbox = message['object']['bbox']
            topleftx = bbox['topleftx']
            toplefty = bbox['toplefty']
            bottomrightx = bbox['bottomrightx']
            bottomrighty = bbox['bottomrighty']
            print(f"Bounding Box: Top Left ({topleftx}, {toplefty}), Bottom Right ({bottomrightx}, {bottomrighty})")
            ancho_total = 1280
            angulo_x = calcular_angulo_x(topleftx, bottomrightx, ancho_total)
            print(f"Ángulo X calculado: {angulo_x}")
            object_id = message['object']['id']
            with objetos_lock:
                if object_id in objetos_detectados:
                    objetos_detectados[object_id]['angulo_x'] = angulo_x
                else:
                    objetos_detectados[object_id] = {'angulo_x': angulo_x, 'distancia': None}
        except (KeyError, json.JSONDecodeError) as e:
            print(f"Error procesando el mensaje Kafka: {e}")

async def actualizar_matriz_oled():
    while True:
        with objetos_lock:
            matriz_datos = sum(1 << calcular_segmento(obj['angulo_x']) for obj in objetos_detectados.values() if obj['distancia'] is not None)
        enviar_datos_i2c(matriz_datos, 0, 0xFF, 0xFF)
        print(f"Matriz OLED actualizada: {matriz_datos}")
        await asyncio.sleep(1)

async def main():
    await asyncio.gather(
        procesar_mensajes_kafka(),
        procesar_datos_lidar(),
        actualizar_objetos(),
        actualizar_matriz_oled()
    )

asyncio.run(main())
