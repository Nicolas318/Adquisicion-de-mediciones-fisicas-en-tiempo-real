#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include <WiFiUdp.h>


// Credenciales de Red
const char* ssid = "LAPTOP-NICO";
const char* password = "84|4Z98a";

//Mensajes de Protocolo de Comunicación
const char* startMessage = "START";
const char* stopMessage = "STOP";
const char* handshakeMessage = "HELLO";


// La dirección IP y el puerto del servidor Python
IPAddress serverIP;       
unsigned int serverPort;  

// Inicializar la instancia de la librería UDP
WiFiUDP Udp;

// Variables Globales Estáticas
#define DATOSPORMUESTRA 7
#define BYTESPORMUESTRA 28
#define MUESTRASPORENVIO 20
#define PACKETSIZE (MUESTRASPORENVIO * BYTESPORMUESTRA)
#define UPDPORT 12345


// Contadores
int contadorMuestras = 0;
int startNumber = 0;
long tiempoInicioCaptura = 0;
byte* byteArrayEnvio;

/**
 * @brief Rellena el buffer con los datos del acelerómetro y giroscopio.
 * 
 * @param buffer Array de floats que será llenado con los datos del sensor.
 */
void accelerometerPlusgyroscope(float* buffer) {
  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    buffer[0] = x;
    buffer[1] = y;
    buffer[2] = z;
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    buffer[3] = x;
    buffer[4] = y;
    buffer[5] = z;
  }
  buffer[6] = millis() - tiempoInicioCaptura;
}

/**
 * @brief Convierte un array de floats a un array de bytes.
 * 
 * @param floatToTransform Array de floats a convertir.
 * @param byteArray Array de bytes que almacenará los datos convertidos.
 */
void floatToByte(float* floatToTransform, byte* byteArray) {
  byte* byteArray1 = (byte*)&floatToTransform[0];
  byte* byteArray2 = (byte*)&floatToTransform[1];
  byte* byteArray3 = (byte*)&floatToTransform[2];
  byte* byteArray4 = (byte*)&floatToTransform[3];
  byte* byteArray5 = (byte*)&floatToTransform[4];
  byte* byteArray6 = (byte*)&floatToTransform[5];
  byte* byteArray7 = (byte*)&floatToTransform[6];
  for (int i = 0; i < 4; i++) {
    byteArray[i] = byteArray1[i];
  }
  for (int i = 0; i < 4; i++) {
    byteArray[4 + i] = byteArray2[i];
  }
  for (int i = 0; i < 4; i++) {
    byteArray[8 + i] = byteArray3[i];
  }
  for (int i = 0; i < 4; i++) {
    byteArray[12 + i] = byteArray4[i];
  }
  for (int i = 0; i < 4; i++) {
    byteArray[16 + i] = byteArray5[i];
  }
  for (int i = 0; i < 4; i++) {
    byteArray[20 + i] = byteArray6[i];
  }
  for (int i = 0; i < 4; i++) {
    byteArray[24 + i] = byteArray7[i];
  }
}

/**
 * @brief Copia los valores de un array de bytes a otro, considerando el número de muestras acumuladas.
 * 
 * @param array1 Array de destino donde se copiarán los valores.
 * @param array2 Array de origen desde donde se copiarán los valores.
 * @param numeroMuestrasAcumuladas Número de muestras ya acumuladas en array1.
 */
void copyArrays(byte* array1, byte* array2, int numeroMuestrasAcumuladas) {
  for (int i = 0; i < BYTESPORMUESTRA; i++) {
    array1[i + BYTESPORMUESTRA * numeroMuestrasAcumuladas] = array2[i];
  }
}
/**
 * @brief Espera recibir un mensaje de inicio ("START") desde el servidor.
 * 
 * La función se ejecuta en un bucle hasta que se recibe el mensaje "START".
 * La función también extrae el número de segundos que estará activa 
 * la adquisición de datos
 */

void waitForStartMessage() {
  while (true) {
    // Comprobar si hay datos disponibles
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      // Asignar un buffer para almacenar el paquete entrante
      char packetBuffer[255];
      // Leer el paquete en el buffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;  // Terminar la cadena con un carácter nulo 
        // Dividir el mensaje recibido por el carácter ';'
        char* token = strtok(packetBuffer, ";");
        // Comprobar si la primera parte del mensaje es el mensaje de inicio
        if (strcmp(token, startMessage) == 0) {
          // Mensaje de inicio recibido
          // Extraer el número de inicio de la segunda parte del mensaje
          token = strtok(NULL, ";");
          if (token != NULL) {
            // Convertir el número de inicio a entero
            startNumber = atoi(token);
          }
          // Guardar el tiempo de inicio
          tiempoInicioCaptura = millis();
          return;  // Salir del bucle
        }
      }
    }
    delay(100);  // Esperar antes de volver a comprobar
  }
}

/**
 * @brief Espera recibir un mensaje de parada ("STOP") desde el servidor.
 * 
 * Si se recibe el mensaje "STOP", se vuelve a llamar a waitForStartMessage() para reiniciar la espera del mensaje de inicio.
 */
void waitForStopMessage() {
  // Comprobar si hay datos disponibles
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // Asignar un buffer para almacenar el paquete entrante
    char packetBuffer[255];
    // Leer el paquete en el buffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;  // Terminar la cadena con un carácter nulo 
      // Comprobar si el mensaje recibido coincide con el mensaje de parada
      if (strcmp(packetBuffer, stopMessage) == 0) {
        // Mensaje de parada recibido
        waitForStartMessage();  // Llamar a la función para esperar el mensaje de inicio
      }
    }
  }
}

/**
 * @brief Espera recibir un mensaje de saludo ("HELLO") desde el servidor.
 * 
 * La función se ejecuta en un bucle hasta que se recibe el mensaje "HELLO".
 * También extrae la IP y el puerto del servidor
 */
void waitForHandshake(){
  while(1){
    // Comprobar si hay datos disponibles
    int packetSize = Udp.parsePacket();
    if(packetSize){
      // Asignar un buffer para almacenar el paquete entrante
      char packetBuffer[255];
      // Leer el paquete en el buffer
      int len = Udp.read(packetBuffer, 255);
      if(len > 0){
        packetBuffer[len] = 0;  // Terminar la cadena con un carácter nulo
        // Comprobar si el mensaje recibido coincide con el mensaje de saludo
        if(strcmp(packetBuffer, handshakeMessage) == 0){
          serverIP = Udp.remoteIP();  // Guardar la IP del servidor
          serverPort = Udp.remotePort();  // Guardar el puerto del servidor
          Serial.println(serverIP);
          Serial.println(serverPort);
          return;  // Salir del bucle
        }
      }
    }
    delay(100);  // Esperar antes de volver a comprobar
  }
}

/**
 * @brief Envía un mensaje de difusión ("HELLO") a la red y espera una respuesta.
 * 
 * La función envía un mensaje de difusión a la dirección IP de broadcast y luego llama a waitForHandshake() para esperar la respuesta.
 */
void sendBroadcastMessage(){
  // Calcular la dirección IP de broadcast
  IPAddress broadcastIP = ~WiFi.subnetMask() | WiFi.gatewayIP();
  // Iniciar el paquete UDP a la dirección IP de broadcast
  Udp.beginPacket(broadcastIP, 5005);
  // Escribir el mensaje de saludo en el paquete
  Udp.write(handshakeMessage);
  // Terminar y enviar el paquete
  Udp.endPacket();
  // Esperar la respuesta del servidor
  waitForHandshake();
}

/**
 * @brief Envía los datos de las muestras acumuladas al servidor a través de UDP.
 * 
 * La función combina los datos de las muestras acumuladas en un array de bytes y los envía al servidor. Luego, limpia el buffer de datos y espera un mensaje de parada ("STOP") del servidor para reiniciar la espera del mensaje de inicio ("START").
 * 
 * @param byteArrayEnvio Array de bytes que contiene los datos de las muestras acumuladas que se van a enviar.
 * @param byteArray Array de bytes que contiene los datos de la muestra actual que se va a agregar al buffer.
 * @param numeroMuestrasAcumuladas Número de muestras acumuladas en el buffer de envío.
 */
void sendSamples(byte* byteArrayEnvio, byte* byteArray, int numeroMuestrasAcumuladas){
  // Copiar los datos de la muestra actual al buffer de envío
  copyArrays(byteArrayEnvio, byteArray, contadorMuestras);
  
  // Iniciar el paquete UDP a la dirección IP del servidor y puerto
  Udp.beginPacket(serverIP, serverPort);
  
  // Escribir los datos del buffer en el paquete
  Udp.write(byteArrayEnvio, PACKETSIZE);
  
  // Terminar y enviar el paquete
  Udp.endPacket();
  
  // Reiniciar el contador de muestras acumuladas
  contadorMuestras = 0;
  
  // Liberar la memoria del buffer de envío
  free(byteArrayEnvio);
  
  // Esperar un mensaje de parada del servidor antes de continuar
  waitForStopMessage();
}


void setup() {

    Serial.begin(9600);
  // Conectar a la red WiFi
  WiFi.begin(ssid, password);

  // Esperar a que se establezca la conexión WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);  // Esperar antes de intentar de nuevo
    WiFi.begin(ssid, password);  // Intentar conectar de nuevo
  }
  
  // Iniciar UDP
  Udp.begin(UPDPORT);  // El puerto no importa para el envío

  // Iniciar el sensor IMU
  if (!IMU.begin()) {
    // Si el IMU no se inicializa correctamente, entrar en un bucle infinito
    while (1);
  }
  
  // Enviar un mensaje de difusión y esperar una respuesta de saludo
  sendBroadcastMessage();
  
  // Esperar a recibir el mensaje de inicio ("START") desde el servidor
  waitForStartMessage();
}

void loop() {
  // Comprobar si el tiempo de ejecución no ha superado el tiempo especificado por startNumber
  if (startNumber == 0 || startNumber * 1000 > (millis() - tiempoInicioCaptura)) {
    // Convertimos los datos de los sensores en un array de bytes
    float* sensorData = (float*)(malloc(DATOSPORMUESTRA * sizeof(float)));
    byte* byteArray = (byte*)(malloc(BYTESPORMUESTRA * sizeof(byte)));
    accelerometerPlusgyroscope(sensorData);
    floatToByte(sensorData, byteArray);
    free(sensorData);

    if (contadorMuestras == 0) {
      // Si no hay muestras acumuladas, reservar memoria para el envío
      byteArrayEnvio = (byte*)(malloc(PACKETSIZE * sizeof(byte)));
      copyArrays(byteArrayEnvio, byteArray, contadorMuestras);
      contadorMuestras++;
    } else {
      if (contadorMuestras < MUESTRASPORENVIO - 1) {
        // Acumular muestras hasta alcanzar MUESTRASPORENVIO
        copyArrays(byteArrayEnvio, byteArray, contadorMuestras);
        contadorMuestras++;
      } else {
        // Al alcanzar el número de muestras, enviar los datos por UDP
        sendSamples(byteArrayEnvio, byteArray, contadorMuestras);
      }
    }
    free(byteArray);
  } else {
    // Si el tiempo ha superado startNumber * 1000 ms, esperar el mensaje de inicio nuevamente
    waitForStartMessage();
  }
}

