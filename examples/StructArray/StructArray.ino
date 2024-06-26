/*
 * Example of a basic FreeRTOS queue
 * https://www.freertos.org/Embedded-RTOS-Queues.html
 */

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

// Include queue support
#include <queue.h>

// Define a Structure Array
struct Arduino {
    int pin[2];
    int ReadValue[2];
};

//Function Declaration
void Blink (void *pvParameters);
void POT (void *pvParameters);
void TaskSerial (void *pvParameters);

/*
 * Declaring a global variable of type QueueHandle_t
 *
 */
QueueHandle_t structArrayQueue;

void setup()
{
    Serial.begin (115200);

    // Wait for a serial port connection to be established before continuing.
    // Don't want to miss any debug messages.
    while ( !Serial ) {
        delay (10);    // for nrf52840 with native usb
    }

    Serial.println ("STARTING THE APPLICATION.");
    // Configure pin 4 as an input and enable the internal pull-up resistor.
    pinMode (4, INPUT_PULLUP);
    /**
     * Create a queue.
     * https://www.freertos.org/a00116.html
     */
    structArrayQueue = xQueueCreate (10, //Queue length
                                     sizeof (struct Arduino) ); //Queue item size

    if (structArrayQueue != NULL) {
        xTaskCreate (TaskBlink, // Task function
                     "Blink",// Task name
                     128,// Stack size
                     NULL,
                     0,// Priority
                     NULL);
// Create other task that publish data in the queue if it was created.
        xTaskCreate (POT, // Task function
                     "AnalogRead",// Task name
                     128,  // Stack size
                     NULL,
                     2,// Priority
                     NULL);
        // Create task that consumes the queue if it was created.
        xTaskCreate (TaskSerial, // Task function
                     "PrintSerial",// A name just for humans
                     128,// This stack size can be checked & adjusted by reading the Stack Highwater
                     NULL,
                     1, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                     NULL);
    }
}

void loop()
{
    static bool firstTime = true;
    static int previousDigitalReadValue = -1;

    if ( firstTime ) {
        Serial.println ("Starting loop....");
        delay (1000);
        firstTime = false;
    }

    int digitalReadValue = digitalRead (4);

    if (digitalReadValue != previousDigitalReadValue) {
        if (digitalReadValue == HIGH) {
            Serial.println ("HIGH");
        }

        else {
            Serial.println ("LOW");
        }

        previousDigitalReadValue = digitalReadValue;
    }

    delay (1000);
}

/*
 * Blink task.
 * See Blink_AnalogRead example.
 */
void TaskBlink (void *pvParameters)
{
    (void) pvParameters;
    Serial.print ("Starting task ");
    Serial.println (pcTaskGetName (NULL) ); // Get task name
    pinMode (LED_BUILTIN, OUTPUT);
    digitalWrite (LED_BUILTIN, LOW);

    for (;;) {
        digitalWrite (LED_BUILTIN, HIGH);
        vTaskDelay (250 / (1 + portTICK_PERIOD_MS) );
        digitalWrite (LED_BUILTIN, LOW);
        vTaskDelay (250 / (1 + portTICK_PERIOD_MS) );
    }
}

/**
 * Analog read task for Pin A0 and A1
 * Reads an analog input on pin 0 and pin 1
 * Send the readed value through the queue.
 * See Blink_AnalogRead example.
 */
void POT (void *pvParameters)
{
    (void) pvParameters;
    Serial.print ("Starting task ");
    Serial.println (pcTaskGetName (NULL) ); // Get task name
    pinMode (A0, INPUT);
    pinMode (A1, INPUT);

    for (;;) {
        // Read the input on analog pin 0:
        struct Arduino currentVariable;
        currentVariable.pin[0] = 0;
        currentVariable.pin[1] = 1;
        currentVariable.ReadValue[0] = analogRead (A0);
        currentVariable.ReadValue[1] = analogRead (A1);
        /**
          * Post an item on a queue.
          * https://www.freertos.org/a00117.html
          */
        xQueueSend (structArrayQueue, &currentVariable, portMAX_DELAY);
        // One tick delay (15ms) in between reads for stability
        vTaskDelay (1);
    }
}

/**
 * Serial task.
 * Prints the received items from the queue to the serial monitor.
 */
void TaskSerial (void *pvParameters)
{
    (void) pvParameters;
    Serial.print ("Starting task ");
    Serial.println (pcTaskGetName (NULL) ); // Get task name

    for (;;) {
        struct Arduino currentVariable;

        /**
          * Read an item from a queue.
          * https://www.freertos.org/a00118.html
          */
        if (xQueueReceive (structArrayQueue, &currentVariable, portMAX_DELAY) == pdPASS ) {
            for (int i = 0; i < 2; i++) {
                Serial.print ("PIN:");
                Serial.println (currentVariable.pin[i]);
                Serial.print ("value:");
                Serial.println (currentVariable.ReadValue[i]);
            }
        }

        vTaskDelay (500 / (1 + portTICK_PERIOD_MS) );
    }
}
