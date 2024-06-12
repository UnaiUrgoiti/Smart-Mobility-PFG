#include <Arduino.h>
#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x28
#define SDA_PIN 0 // Para la versión final, usa 12
#define SCL_PIN 1 // Para la versión final, usa 13
#define LED_R_PIN 19
#define LED_G_PIN 20
#define LED_B_PIN 21
#define GP25_PIN 25 // LED interno
#define BUZZER_PIN 17 // Buzzer

const int columnPins[] = {2, 3, 4, 12, 13};  // Pines para las columnas, final {0, 1, 2, 3, 4}
const int rowPins[] = {5, 6, 7, 10, 11};     // Pines para las filas
const int switchPins[] = {28, 27, 26, 22};   // Pines para los switches

// Variables para almacenar los datos recibidos
uint32_t matrizData = 0;
uint8_t redValue = 0;
uint8_t greenValue = 0;
uint8_t blueValue = 0;
uint8_t switchState = 0;

// Prototipos de funciones
void apagar_matriz();
void encender_matriz();
void led(int uno, int dos, int tres, int cuatro, int cinco);
void actualizar_matriz();
void buzzer_on();
void buzzer_off();
void actualizar_matriz_desde_datos();
void actualizar_led_rgb();
void debug_print_data();  // Función añadida para depuración
uint8_t leer_switches();  // Función para leer los switches
void play_melody();  // Función para reproducir la melodía

// Notas musicales en frecuencias (Hz)
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

// Melodía de "Daisy Bell (Bicycle Built for Two)"
int melody[] = {
  NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4,
  NOTE_D4, NOTE_F4, NOTE_A4, NOTE_D4, NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_D4, NOTE_C4
};

// Duraciones de las notas (4 = negra, 8 = corchea, etc.)
int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2
};

// Apaga todas las columnas y desactiva todas las filas
void apagar_matriz() {
    for (int i = 0; i < 5; i++) {
        digitalWrite(columnPins[i], LOW);
    }
    for (int i = 0; i < 5; i++) {
        digitalWrite(rowPins[i], HIGH);
    }
}

// Enciende todas las columnas y activa todas las filas
void encender_matriz() {
    for (int i = 0; i < 5; i++) {
        digitalWrite(columnPins[i], HIGH);
    }
    for (int i = 0; i < 5; i++) {
        digitalWrite(rowPins[i], LOW);
    }
}

// Enciende los LEDs en una fila según los valores pasados
void led(int uno, int dos, int tres, int cuatro, int cinco) {
    digitalWrite(columnPins[0], uno);
    digitalWrite(columnPins[1], dos);
    digitalWrite(columnPins[2], tres);
    digitalWrite(columnPins[3], cuatro);
    digitalWrite(columnPins[4], cinco);
}

// Actualiza la matriz LED encendiendo los LEDs según la fila activa
void actualizar_matriz() {
    apagar_matriz();
    digitalWrite(rowPins[0], LOW);
    led(1, 1, 1, 1, 1);
    delay(2);

    apagar_matriz();
    digitalWrite(rowPins[1], LOW);
    led(1, 0, 0, 0, 0);
    delay(2);

    apagar_matriz();
    digitalWrite(rowPins[2], LOW);
    led(1, 1, 1, 0, 0);
    delay(2);

    apagar_matriz();
    digitalWrite(rowPins[3], LOW);
    led(1, 0, 0, 0, 0);
    delay(2);

    apagar_matriz();
    digitalWrite(rowPins[4], LOW);
    led(1, 1, 1, 1, 1);
    delay(2);
}

// Enciende el buzzer
void buzzer_on() {
    digitalWrite(BUZZER_PIN, HIGH);
}

// Apaga el buzzer
void buzzer_off() {
    digitalWrite(BUZZER_PIN, LOW);
}

// Reproduce la melodía
void play_melody() {
    for (int thisNote = 0; thisNote < sizeof(melody) / sizeof(int); thisNote++) {
        int noteDuration = 1000 / noteDurations[thisNote];
        tone(BUZZER_PIN, melody[thisNote], noteDuration);
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        noTone(BUZZER_PIN);
    }
}

// Actualiza la matriz LED desde los datos recibidos por I2C
void actualizar_matriz_desde_datos() {
    apagar_matriz();
    for (int row = 0; row < 5; row++) {
        digitalWrite(rowPins[row], LOW);
        for (int col = 0; col < 5; col++) {
            int bitIndex = row * 5 + col;
            int bitValue = (matrizData >> bitIndex) & 1;
            digitalWrite(columnPins[col], bitValue);
        }
        delay(2);
        apagar_matriz();
    }
}

// Actualiza los valores del LED RGB
void actualizar_led_rgb() {
    analogWrite(LED_R_PIN, redValue);
    analogWrite(LED_G_PIN, greenValue);
    analogWrite(LED_B_PIN, blueValue);
}

// Función para imprimir datos de depuración
void debug_print_data() {
    Serial.print("Matriz Data: ");
    Serial.println(matrizData, BIN);
    Serial.print("Red: ");
    Serial.println(redValue);
    Serial.print("Green: ");
    Serial.println(greenValue);
    Serial.print("Blue: ");
    Serial.println(blueValue);
}

// Función para leer los switches y devolver un valor de 4 bits
uint8_t leer_switches() {
    uint8_t switches = 0;
    for (int i = 0; i < 4; i++) {
        if (digitalRead(switchPins[i]) == LOW) {  // Suponiendo que el switch está activo en LOW
            switches |= (1 << i);
        }
    }
    return switches;
}

// Esta función se llama cuando el maestro envía datos al esclavo.
void receiveEvent(int howMany) {
    Serial.print("Número de bytes recibidos: ");
    Serial.println(howMany);
    digitalWrite(GP25_PIN, HIGH);  // Enciende el pin GP25
    if (howMany == 8 && switchState == 0b0000) {  // Verificar que se reciben 8 bytes y que este en modo default
        matrizData = Wire.read() | (Wire.read() << 8) | (Wire.read() << 16) | (Wire.read() << 24);  // Leer los 4 bytes de datos de matriz
        redValue = Wire.read();  // Leer el valor del color rojo
        greenValue = Wire.read();  // Leer el valor del color verde
        blueValue = Wire.read();  // Leer el valor del color azul

        buzzer_on();  // Encender el buzzer
        delay(100);   // Mantener el buzzer encendido por 100 ms
        buzzer_off(); // Apagar el buzzer

        actualizar_matriz_desde_datos();  // Actualizar la matriz LED
        actualizar_led_rgb();  // Actualizar los valores del LED RGB inmediatamente

        debug_print_data();  // Imprimir los datos recibidos para depuración
    } else {
        Serial.print("Número de bytes recibidos inesperado: ");
        Serial.println(howMany);
        while (Wire.available()) {
            Wire.read();  // Descartar los bytes si no se reciben exactamente 8
        }
    delay(10);
    digitalWrite(GP25_PIN, LOW);  // Apaga el pin GP25
    }
}

// Esta función se llama cuando el maestro solicita datos del esclavo.
void requestEvent() {
    Wire.write("Hello from Pico");
}

void setup() {
    // Inicia la comunicación serie
    Serial.begin(115200);
    Serial.println("I2C Slave Initialized");

    // Configura los pines de columnas y filas como salidas
    for (int i = 0; i < 5; i++) {
        pinMode(columnPins[i], OUTPUT);
        pinMode(rowPins[i], OUTPUT);
    }

    // Configura los pines de los switches como entradas
    for (int i = 0; i < 4; i++) {
        pinMode(switchPins[i], INPUT_PULLUP); // Usa resistencias pull-up internas
    }

    // Leer el estado de los switches y mostrarlo en la consola serie
    switchState = leer_switches();
    Serial.print("Estado de los switches (4 bits): ");
    Serial.println(switchState, BIN);

    // Configura los pines del LED RGB como salidas
    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    digitalWrite(LED_R_PIN, LOW);
    digitalWrite(LED_G_PIN, LOW);
    digitalWrite(LED_B_PIN, LOW);

    // Configura el pin GP25 como salida
    pinMode(GP25_PIN, OUTPUT);
    digitalWrite(GP25_PIN, LOW);

    // Configura el pin del buzzer como salida
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Configura los pines I2C con resistencias pull-up internas
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);

    // Configura los pines I2C en la biblioteca Wire
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);

    // Inicia la comunicación I2C
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

void loop() {
    switch (switchState) {
        case 0b0000:
            // Acción por defecto: actualizar matriz y RGB con datos recibidos
            actualizar_matriz_desde_datos();
            actualizar_led_rgb();
            delay(100);
            Serial.println("Switch State: 0000 - Default action");
            break;
        case 0b0001:
            // Acción para 0001: apagar todo
            Serial.println("Switch State: 0001 ");
            apagar_matriz();
            redValue = 0; greenValue = 0; blueValue = 0;
            actualizar_led_rgb();
            buzzer_off();
            delay(1000);
            break;
        case 0b0010:
            // Acción para 0010: encender todo
            Serial.println("Switch State: 0010 ");
            encender_matriz();
            redValue = 255; greenValue = 255; blueValue = 255;
            actualizar_led_rgb();
            delay(1000);
            break;
        case 0b0100:
            // Acción para 0100: encender RGB en Magenta y accion.
            Serial.println("Switch State: 0100 ");
            redValue = 255; greenValue = 0; blueValue = 255;
            actualizar_led_rgb();
            delay(1000);
            break;
        case 0b1000:
            // Acción para 1000: encender RGB en Azul (Blue) y accion especial.
            Serial.println("Switch State: 1000 ");
            redValue = 0; greenValue = 0; blueValue = 255;
            actualizar_led_rgb();
            play_melody();
            delay(1000);
            break;
    }
}
