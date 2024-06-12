import serial
import time
import paho.mqtt.client as mqtt
import json
import re

def imprimir_info():
    """
    Prints information for visualization purposes.
    """
    print("Parallax GPS module Test Application")
    print("--------------------------------------------------------------------------------")
    print("By Unai Urgoiti for the TFG smart-mobility 2023/2024")
    print("--------------------------------------------------------------------------------")
    print("App Starting Now")
    print("--------------------------------------------------------------------------------")


def recibir_info():
    """
    Reads NMEA sentences from the GPS module and processes them.
    """
    sentencias_nmea = []  # List for storing NMEA sentences
    while True:
        mensaje_bytes = ser.read()  # Read a byte
        if mensaje_bytes == b'$':  # If start byte
            sentencia = b'$'
            while True:
                byte = ser.read()  # Read next byte
                if byte == b'\r':  # If end byte
                    sentencia += byte
                    byte = ser.read()  # Read next byte
                    if byte == b'\n':  # If skip byte
                        sentencia += byte
                        sentencias_nmea.append(sentencia)  # Add to list
                        if sentencia.startswith(b'$GPRMC'):  # If control sentence
                            for s in sentencias_nmea:
                                datos = decode_nmea0183(s.decode('ascii').strip())  # Decode all sentences
                                if datos:
                                    imprimir_datos(datos)
                            sentencias_nmea = []  # Clear list
                        break
                else:
                    sentencia += byte  # Add byte to sentence

def decode_nmea0183(message):
    """
    Decodes an NMEA0183 message into a dictionary of GPS data.
    """
    parts = message.split(',')  # Split message into parts

    if len(parts) >= 6 and parts[0][0] == '$':  # Verify message
        message_type = parts[0][1:]

        if message_type == 'GPGGA':
            # Example: $GPGGA,170834,4124.8963,N,08151.6838,W,1,05,1.5,280.2,M,-34.0,M,,,*75
            time_utc = parts[1]  # Time in UTC format
            latitude = convertir_coordenadas_grados(parts[2], parts[3])  # Latitude in degrees
            longitude = convertir_coordenadas_grados(parts[4], parts[5])  # Longitude in degrees
            altitude = parts[9]  # Altitude in meters

            return {
                'message_type': message_type,
                'time_utc': time_utc,
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude
            }
        elif message_type == 'GPRMC':
            # Example: $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
            time_utc = parts[1]  # Time in UTC format
            latitude = convertir_coordenadas_grados(parts[3], parts[4])  # Latitude in degrees
            longitude = convertir_coordenadas_grados(parts[5], parts[6])  # Longitude in degrees
            speed = parts[7]  # Speed in knots
            track_angle = parts[8]  # Track angle

            return {
                'message_type': message_type,
                'time_utc': time_utc,
                'latitude': latitude,
                'longitude': longitude,
                'speed': speed,
                'track_angle': track_angle
            }

    return None

def convertir_coordenadas_grados(coordenada, direccion):
    """
    Converts GPS coordinates from NMEA format to degrees.
    """
    partes = coordenada.split('.')  # Split coordinate into parts
    grados = int(partes[0][:-2])  # First two digits are degrees

    minutos_decimal = float(f"{partes[0][-2:]}.{partes[1]}")  # The rest are decimals

    minutos = minutos_decimal / 60  # Convert minutes to degrees

    coordenadas = grados + minutos

    if direccion in ['S', 'W']:  # Adjust for southern or western hemisphere
        coordenadas *= -1

    return coordenadas

def imprimir_datos(datos):
    """
    Prints and sends GPS data for debugging and verification.
    """
    print("UTC Hour:", convertir_hora(datos['time_utc']))
    print("Latitude:", datos['latitude'])
    print("Longitude:", datos['longitude'])
    if 'altitude' in datos:
        print("Altitude:", datos['altitude'])
    if 'speed' in datos:
        print("Speed:", datos['speed'])
    if 'track_angle' in datos:
        print("Track angle:", datos['track_angle'])
    print("--------------------------------------------------------------------------------")
    enviar_datos(datos)

def enviar_datos(datos):
    """
    Sends GPS data via MQTT.
    """
    datos_json = json.dumps(datos)  # Convert data to JSON
    client.publish("gps/datos", datos_json)  # Publish data

def convertir_hora(hora_utc):
    """
    Converts UTC time from HHMMSS format to HH:MM:SS format.
    """
    hora = hora_utc[:2]  # First two digits are hours
    minutos = hora_utc[2:4]  # Next two digits are minutes
    segundos = hora_utc[4:6]  # Last two digits are seconds

    return f"{hora}:{minutos}:{segundos}"

#------------------------------------------------------------------
# MQTT Connection Setup with Paho
#------------------------------------------------------------------

mqtt_broker = "192.168.23"  # MQTT broker address
mqtt_port = 1883  # MQTT broker port
client = mqtt.Client()
client.connect(mqtt_broker, mqtt_port, 60)
client.loop_start()

#------------------------------------------------------------------
# Main function
#------------------------------------------------------------------

ser = serial.Serial("/dev/ttyTHS1", baudrate=4800, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.5)
imprimir_info()

while True:
    recibir_info()
