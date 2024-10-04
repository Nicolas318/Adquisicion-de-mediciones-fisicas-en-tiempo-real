import socket
import struct
import csv
import threading
import logging
import time
import os
import io
from flask import Flask, request, jsonify, render_template, abort, send_file

# Configuración de la aplicación Flask
app = Flask(__name__)

# Función para configurar el socket UDP
def setup_socket():
    global server_socket, UDP_IP, UDP_PORT, BUFFER_SIZE, ARDUINO_IP, ARDUINO_PORT

    # Configuración de las variables globales relacionadas con la conexión UDP
    UDP_IP = ''          # Dirección IP local para escuchar en todas las interfaces disponibles
    UDP_PORT = 5005      # Puerto UDP donde escuchará el servidor
    BUFFER_SIZE = 560    # Tamaño del buffer para recibir datos (20 muestras de 7 floats)

    # Dirección IP y puerto del Arduino (se inicializan vacíos)
    ARDUINO_IP = ''
    ARDUINO_PORT = 0

    # Creación de un socket UDP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Configuración para permitir broadcast en el socket
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # Intento de obtener la dirección IP del servidor local
    try:
        UDP_IP = socket.gethostbyname(socket.gethostname())
    except socket.gaierror:
        print("Error: No se puede obtener la dirección IP.")
        exit()

    # Vinculación del socket al puerto especificado
    server_socket.bind(("", UDP_PORT))

    # Bucle principal para esperar la conexión y recibir datos
    while True:
        # Recibir datos y dirección del cliente (Arduino)
        data, addr = server_socket.recvfrom(BUFFER_SIZE)
        message = data.decode()

        # Si el mensaje recibido es "HELLO", se establece la dirección IP y puerto del Arduino
        if message == "HELLO":
            ARDUINO_IP = addr[0]
            ARDUINO_PORT = addr[1]
            response = "HELLO"
            # Enviar respuesta "HELLO" de vuelta al Arduino
            server_socket.sendto(response.encode(), addr)
            break  # Salir del bucle una vez establecida la conexión

    # Mensaje indicando que el servidor UDP está iniciado y esperando datos
    print("Servidor UDP iniciado. Esperando datos...")

# Función para mostrar resultados parseados y guardarlos en CSV
def show_results(data):
    fmt = '<140f'  # Suponiendo 20 muestras de 7 floats cada una (formato little-endian)
    samples = struct.unpack(fmt, data)  # Desempaquetar los datos binarios según el formato especificado

    for i in range(20):
        sample_data = samples[i*7:(i+1)*7]  # Extraer datos para cada muestra (7 floats)
        accelerometer_x, accelerometer_y, accelerometer_z, gyro_x, gyro_y, gyro_z, timestamp = sample_data
        
        # Escribir los datos en el archivo CSV 'data.csv'
        with open('data.csv', 'a', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([timestamp, accelerometer_x, accelerometer_y, accelerometer_z, gyro_x, gyro_y, gyro_z])
            csvfile.close()

# Función para enviar mensaje de inicio al Arduino
def send_start_message(start_number):
    if ARDUINO_IP != '':
        try:
            print("Eliminamos el archivo.csv /send_start_message()")
            os.remove('data.csv')
            print(f"File {'data.csv'} deleted successfully.")
        except FileNotFoundError:
            print(f"File {'data.csv'} not found.")
        except PermissionError:
            print(f"Permission denied: Unable to delete {'data.csv'}.")
        except Exception as e:
            print(f"Error occurred while deleting file {'data.csv'}: {e}")
        start_message = f"START;{start_number}".encode()  # Codifica el mensaje de inicio con el número especificado

        server_socket.sendto(start_message, (ARDUINO_IP, ARDUINO_PORT))


# Función para enviar mensaje de parada al Arduino
def send_stop_message():
    if ARDUINO_IP != '':
        stop_message = b"STOP"  # Define el mensaje de parada como una secuencia de bytes
        server_socket.sendto(stop_message, (ARDUINO_IP, ARDUINO_PORT))


def test_sensors():
    if ARDUINO_IP != '':
        # Enviar mensaje de inicio con el número 10
        start_message = f"START;10".encode()
        server_socket.sendto(start_message, (ARDUINO_IP, ARDUINO_PORT))

        end_time = time.time() + 10  # Establecer el tiempo de finalización (10 segundos desde ahora)
        interval = 1 / 50  # Intervalo de tiempo entre cada muestra (aproximadamente 1/25 segundos)
        data_points = []  # Lista para almacenar los datos de las muestras

        while time.time() < end_time:
            # Recibir datos del socket
            data, addr = server_socket.recvfrom(BUFFER_SIZE)

            # Desempaquetar los datos recibidos (suponiendo que son 140 floats en formato little-endian)
            fmt = '<140f'
            samples = struct.unpack(fmt, data)

            # Tomar solo la primera muestra de 7 floats para simplificar
            sample_data = samples[:7]

            # Agregar los datos de la muestra a la lista de puntos de datos
            data_points.append(sample_data)

            # Esperar el intervalo antes de la próxima muestra
            time.sleep(interval)

        return data_points  # Devolver la lista de puntos de datos recolectados durante la prueba    

# Función para iniciar el servidor UDP
def start_udp_server():
    setup_socket()  # Configurar el socket UDP
    
    while True:
        data, addr = server_socket.recvfrom(BUFFER_SIZE)  # Esperar a que lleguen datos
        # Procesar los datos recibidos
        show_results(data)


# Rutas de Flask
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/command')
def command():
    cmd = request.args.get('cmd')  # Obtener el valor del parámetro 'cmd' de la solicitud
    time = request.args.get('time', default=10, type=int)  # Obtener el valor de 'time' como entero, valor por defecto 10 si no está presente

    if cmd == 'start':
        send_start_message(0)  # Llamar a la función send_start_message() con el número 0
    elif cmd == 'start_temp':
        send_start_message(time)  # Llamar a send_start_message() con el valor de 'time'
    elif cmd == 'stop':
        send_stop_message()  # Llamar a la función send_stop_message() para detener la operación

    return jsonify({'message': 'Command received: ' + cmd})  # Devolver una respuesta JSON indicando el comando recibido


@app.route('/test_sensors')
def get_sensor_data():
    data_points = test_sensors()  # Llamar a la función test_sensors() para obtener los datos del sensor
    return jsonify(data_points)  # Devolver los datos del sensor como respuesta JSON

@app.route('/check_data')
def check_sensor_data():
    data = []

    # Verificar si el archivo existe
    if not os.path.exists('data.csv'):
        abort(404, description="Archivo 'data.csv' no encontrado")

    # Leer data.csv y guardar en data
    with open('data.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            data.append([float(value) for value in row])

    return jsonify(data)

@app.route('/download_csv')
def download_csv():
    # Leer el archivo CSV existente
    filename = request.args.get('filename')
    input_file = 'data.csv'
    processed_data = []

    with open(input_file, mode='r', newline='') as file:
        reader = csv.reader(file)
        headers = next(reader)  # Leer las cabeceras
        for row in reader:
            # Aquí puedes procesar cada fila según sea necesario
            processed_data.append(row)

    # Crear el archivo CSV en memoria con los datos procesados
    si = io.StringIO()
    cw = csv.writer(si)
    cw.writerow(headers)  # Escribir las cabeceras
    cw.writerows(processed_data)  # Escribir los datos procesados
    output = io.BytesIO()
    output.write(si.getvalue().encode('utf-8'))
    output.seek(0)

    # Eliminamos el archivo.csv
    try:
        print("Eliminamos el archivo.csv /download()")
        os.remove(input_file)
        print(f"File {input_file} deleted successfully.")
    except FileNotFoundError:
        print(f"File {input_file} not found.")
    except PermissionError:
        print(f"Permission denied: Unable to delete {input_file}.")
    except Exception as e:
        print(f"Error occurred while deleting file {input_file}: {e}")

    # Enviar el archivo al cliente
    return send_file(output, mimetype='text/csv', as_attachment=True, download_name=f'{filename}.csv')


# Función principal
def main():
    # Configurar el nivel de registro de Flask para reducir la cantidad de información en la consola
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)  # Cambiar a ERROR para evitar los mensajes de acceso y advertencias

    # Crear un hilo para ejecutar el servidor Flask
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000))
    flask_thread.start()

    # Llamar a la función para iniciar el servidor UDP
    start_udp_server()


if __name__ == '__main__':
    main()
