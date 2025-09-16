/*
this is the code for an ESP32-c3 super mini reading from an MPU6050 via I2C

                 ┌───────────── ESP32-C3 ─────────────┐
MISO    GPIO5/A5[ 5 ] o       |         |             o [ 5V]  5V
MOSI    GPIO6   [ 6 ] o       | USB C   |             o [  G]  GND
SS      GPIO7   [ 7 ] o       |_________|             o [3.3]  3V3
SDA     GPIO8   [ 8 ] o                               o [  4]  GPIO4/A4
SCL     GPIO9   [ 9 ] o                               o [  3]  GPIO3/A3
        GPIO10  [ 10 ] o        TOP                   o [  2]  GPIO2/A2 
RX      GPIO20  [ 20 ] o                              o [  1]  GPIO1/A1
TX      GPIO21  [ 21 ] o                              o [  0]  GPIO0/A0


*/


#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


MPU6050 mpu;

const float pitch_offset = 0.0;
const float roll_offset = 0.0;

const int N = 10;  // number of values to average
float pitchBuffer[N] = {0};
float rollBuffer[N] = {0};
float pitchAvg;
float rollAvg;
int bufferIndex = 0;
bool bufferFilled = false;

// bluetooth stuff
BLECharacteristic *attitudeCharacteristic;
bool deviceConnected = false;


// UUIDs (randomly generated or use your own)
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define PITCH_CHARACTERISTIC_UUID "abcd1234-1234-1234-1234-1234567890ab"
#define ROLL_CHARACTERISTIC_UUID "abcd2234-1234-1234-1234-1234567890ab"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.printf("Client connected\n");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected, restarting advertising...");
        pServer->getAdvertising()->start();
    }
};

float roundToTenth(float val) {
    return round(val * 10.0) / 10.0;
}

// Function to compute averaged pitch and roll
void computePitchRoll(
    int16_t ax, int16_t ay, int16_t az,       // raw accelerometer inputs
    float pitch_offset, float roll_offset,    // offsets
    float* pitchBuffer, float* rollBuffer,    // circular buffers
    int N,                                    // buffer size
    int& bufferIndex, bool& bufferFilled,     // buffer state
    float& pitchAvg, float& rollAvg           // outputs
) {
    // Convert raw accelerometer to angles
    float axf = ax / 16384.0f;
    float ayf = ay / 16384.0f;
    float azf = az / 16384.0f;

    float roll = atan2(-axf, sqrt(ayf * ayf + azf * azf)) * 180.0f / M_PI;
    float pitch  = atan2(ayf, azf) * 180.0f / M_PI;

    // Store in circular buffer
    pitchBuffer[bufferIndex] = pitch;
    rollBuffer[bufferIndex]  = roll;
    bufferIndex = (bufferIndex + 1) % N;
    if (bufferIndex == 0) bufferFilled = true;

    // Compute average
    int count = bufferFilled ? N : bufferIndex;
    float pitchSum = 0.0f;
    float rollSum = 0.0f;
    for (int i = 0; i < count; i++) {
        pitchSum += pitchBuffer[i];
        rollSum  += rollBuffer[i];
    }

    pitchAvg = roundToTenth(pitchSum / count) + pitch_offset;
    rollAvg  = roundToTenth(rollSum / count) + roll_offset;
}

void sendBleData(float pitch, float roll, float temp){
    
    char buf[64];
    snprintf(buf, sizeof(buf), "Pitch: %.1f, Roll: %.1f, Temp: %.1f", pitch, roll, temp);

    Serial.println(buf);
    
    char ble_buf[32];
    snprintf(ble_buf, sizeof(ble_buf), "%f,%f,%f", pitch, roll, temp);

    if (deviceConnected) {
        attitudeCharacteristic->setValue(ble_buf);
        attitudeCharacteristic->notify();  // send update

        esp_power_level_t adv_pwr = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
        Serial.printf("Advertising TX power: %d dBm\n", (int)adv_pwr);
    }
    return;
}


void setup() {
    Serial.begin(460800);
    Wire.begin(8, 9); // ESP32-C3 SuperMini: SDA=GPIO8, SCL=GPIO9
    Wire.setClock(400000);

    Serial.println("Initializing Izinga...");

    mpu.initialize();

    if (mpu.testConnection()) {
        Serial.println("MPU6050 connected!");
    } else {
        Serial.println("MPU6050 connection failed!");
    }

    // init bluetooth
    Serial.println("Initializing Bluetooth...");

    BLEDevice::init("Izinga");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Pitch
    attitudeCharacteristic = pService->createCharacteristic(
        PITCH_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );
    attitudeCharacteristic->addDescriptor(new BLE2902());  // required for notifications
    // BLEDescriptor *attitudeDesc = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    // attitudeDesc->setValue("Pitch / Roll");
    // attitudeCharacteristic->addDescriptor(attitudeDesc);

    pService->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();

    Serial.println("BLE IMU service started, waiting for client...");
}

void loop() {
    // read the accelerometer and gyro values from the MPU
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // calc the pitch and roll values
    computePitchRoll(
        ax, ay, az,
        pitch_offset, roll_offset,        // offsets
        pitchBuffer, rollBuffer,
        N,
        bufferIndex, bufferFilled,
        pitchAvg, rollAvg
    );

    // get the termperature
    int16_t rawTemp = mpu.getTemperature();
    float tempC = (rawTemp / 340.0) + 36.53;

    // send the data
    sendBleData(pitchAvg, rollAvg, tempC);

    delay(200);
}