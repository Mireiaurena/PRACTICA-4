# PRACTICA-4
# Práctica 4: Sistemas Operativos en Tiempo Real

## Objetivo
El objetivo de esta práctica es comprender el funcionamiento de un sistema operativo en tiempo real (RTOS). Para ello, realizaremos ejercicios prácticos en los que crearemos y ejecutaremos múltiples tareas en un microcontrolador ESP32 utilizando FreeRTOS, observando cómo se distribuye el tiempo de la CPU entre ellas.

## Materiales
- ESP32-S3
- Arduino IDE

---

## Primera Parte: Creación de Tareas en FreeRTOS

### Código
```cpp
    #include <Arduino.h>

    void anotherTask( void * parameter );
    void setup()
    {
        Serial.begin(112500);
        /* we create a new task here */
        xTaskCreate(
        anotherTask, /* Task function. */
        "another Task", /* name of task. */
        10000, /* Stack size of task */
        NULL, /* parameter of the task */
        1, /* priority of the task */
        NULL); /* Task handle to keep track of created task */
    }
    /* the forever loop() function is invoked by Arduino ESP32 loopTask */
    void loop()
    {
        Serial.println("this is ESP32 Task");
        delay(1000);
    }
    /* this function will be invoked when additionalTask was created */
    void anotherTask( void * parameter )
    {
        /* loop forever */
        for(;;)
        {
            Serial.println("this is another Task");
            delay(1000);
        }
        /* delete a task when finish,
        this will never happen because this is infinity loop */
        vTaskDelete( NULL );
    }
```

### Descripción y Funcionamiento
El código implementa un sistema operativo en tiempo real donde se crean dos tareas concurrentes:

- **Tarea principal:** Se ejecuta en la función `loop()`, imprimiendo repetidamente "this is ESP32 Task" en el puerto serie cada segundo.
- **Tarea secundaria:** Creada en `setup()` mediante `xTaskCreate()`. Su función es imprimir "this is another Task" cada segundo en un bucle infinito.

Ambas tareas se ejecutan simultáneamente gracias a FreeRTOS, dividiendo el tiempo de CPU entre ellas.

### Salida esperada en el puerto serie:
```
this is another Task
this is ESP32 Task
this is another Task
this is ESP32 Task
```

### Diagrama de flujo
```mermaid
graph TD;
    A(Inicio) --> B[Serial.begin(112500)];
    B --> C[Crear otra tarea];
    C --> D[Imprimir 'this is another Task'];
    D --> E[Delay de 1000ms];
    E --> D;
    A --> F[Imprimir 'this is ESP32 Task'];
    F --> G[Delay de 1000ms];
    G --> F;
```

---

## Segunda Parte: Uso de Semáforos en FreeRTOS

### Código
```cpp
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

const int ledPin = 11;
SemaphoreHandle_t semaphore;

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    semaphore = xSemaphoreCreateBinary();
    
    xTaskCreate(encenderLED, "Encender LED", 1000, NULL, 1, NULL);
    xTaskCreate(apagarLED, "Apagar LED", 1000, NULL, 1, NULL);
}

void loop() {
}

void encenderLED(void *parameter) {
    for (;;) {
        digitalWrite(ledPin, HIGH);
        Serial.println("LED HIGH");
        delay(1000);
        xSemaphoreGive(semaphore);
    }
}

void apagarLED(void *parameter) {
    for (;;) {
        digitalWrite(ledPin, LOW);
        Serial.println("LED LOW");
        delay(1000);
        xSemaphoreGive(semaphore);
    }
}
```

### Descripción y Funcionamiento
Este código utiliza un semáforo para alternar entre dos tareas:

- **Tarea encenderLED():** Enciende un LED, imprime "LED HIGH" y espera un segundo.
- **Tarea apagarLED():** Apaga el LED, imprime "LED LOW" y espera un segundo.

Las tareas se alternan cada segundo gracias al uso del semáforo.

### Salida esperada en el puerto serie:
```
LED HIGH
LED LOW
LED HIGH
LED LOW
```

---

## Conclusión
Esta práctica nos ha permitido experimentar con FreeRTOS y comprender cómo se pueden ejecutar múltiples tareas en un microcontrolador ESP32. En la primera parte, vimos cómo se pueden crear tareas concurrentes y, en la segunda, exploramos la sincronización entre ellas utilizando un semáforo. Esto demuestra el potencial de los sistemas operativos en tiempo real para gestionar la ejecución eficiente de procesos.

