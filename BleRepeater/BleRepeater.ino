#include <esp_task_wdt.h>

#include "NimBLEAdvertising.h"
#include "NimBLEBeacon.h"
#include "NimBLEDevice.h"
#include "NimBLEEddystoneURL.h"

class BleSensorData
{
public:
    int16_t Temperature;
    uint16_t BatteryMv;
    uint8_t BatteryLevel;
    int Rssi;
    uint16_t Humid;
    std::string AddressStr;
    byte AddressData[6];
    uint8_t Count;

    void SetAddressData(const NimBLEAddress &addr)
    {
        const uint8_t *addrNative = addr.getNative();

        memcpy(&AddressData, addrNative, 6);
    }
};

const int BLE_INTERVAL = 150;      // listen interval
const int BLE_WINDOW_LISTEN = 140; // listen window size

// messages to repeat
std::vector<BleSensorData *> bleScanResults;

// last message number. drop messages with repeating number to not duplicate advertising,
// repeater translates messages multiple times by itself
std::map<uint64_t, uint8_t> lastThermometerPacket;

// bleScanResults is accessed simultaneously from bleScan while advertising from loop
SemaphoreHandle_t bleScanResultsMutex;

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
public:
    AdvertisedDeviceCallbacks() {}

    void onResult(BLEAdvertisedDevice *advertisedDevice)
    {
        NimBLEAddress addr = advertisedDevice->getAddress();
        std::string addrStr = addr.toString();
        uint64_t addr64 = addr;

        // Xiaomi thermometer
        if (addrStr.rfind("a4:c1:38", 0) == 0)
        {
            uint8_t *payLoad = advertisedDevice->getPayload();
            uint8_t payLoadLength = advertisedDevice->getAdvLength();
            NimBLEUUID uuid = advertisedDevice->getServiceDataUUID();

            static BLEUUID atc1441UUID = (uint16_t)0x181A; // UUID 0x181A - atc1441 format
            static BLEUUID xiaomiUUID = (uint16_t)0xFE95;  // UUID 0xFE95 - 0x0D: Xiaomi - temperature x0.1C, humidity x0.1%
            /*
            -atc1441 format:
            UUID 0x181A - size 16: atc1441 format
            UUID 0x181A - size 19: Custom format (all data little-endian):
            -Xiaomi Battery Charge
            UUID 0xFE95 - 0x0A: Xiaomi - battery charge level 0..100%, battery voltage in mV
            Xiaomi Temperature & Humidity
            UUID 0xFE95 - 0x0D: Xiaomi - temperature x0.1C, humidity x0.1%
            Xiaomi Encrypted (bindkey enable)
            UUID 0xFE95 - 0x0A, 0x04, 0x06 Xiaomi - battery charge level 0..100%, temperature x0.1C, humidity x0.1% (All data are averaged over a period of 16 measurements)
            */

            // Serial.printf("%i bytes, UUID: %s", payLoadLength, uuid.toString().c_str());

            if (payLoadLength == 17)
            {
                /*
                uint16_t temp = (uint16_t)((uint16_t)payLoad[10] << 8 | payLoad[11]);
                uint8_t humid = payLoad[12];
                uint16_t battMv = (payLoad[14] << 8) | payLoad[15];

                Serial.printf("[BLE] %i°C, %i%%, %imV\r\n", temp, humid, battMv);
                */
            }
            else if (uuid.equals(atc1441UUID) && payLoadLength == 19)
            {
                uint8_t packetCnt = payLoad[17];

                bool found = false;

                for (auto it = lastThermometerPacket.begin(); it != lastThermometerPacket.end(); ++it)
                {
                    if (it->first == addr64 && it->second == packetCnt)
                    {
                        found = true;
                        break;
                    }
                }

                if (found)
                    return;

                lastThermometerPacket[addr64] = packetCnt;

                int16_t temp = (payLoad[11] << 8) | payLoad[10];
                uint16_t humid = payLoad[13] << 8 | payLoad[12];
                uint16_t battMv = (payLoad[15] << 8) | payLoad[14];
                uint8_t battLevel = payLoad[16];

                int rssi = advertisedDevice->getRSSI();

                BleSensorData *pData = new BleSensorData();
                (*pData).Temperature = temp;
                (*pData).BatteryLevel = battLevel;
                (*pData).BatteryMv = battMv;
                (*pData).Humid = humid;
                (*pData).Rssi = rssi;
                (*pData).AddressStr = addrStr;
                (*pData).SetAddressData(addr);
                (*pData).Count = packetCnt;

                if (xSemaphoreTake(bleScanResultsMutex, portTICK_PERIOD_MS * 10) == pdTRUE)
                {
                    bleScanResults.push_back(pData);

                    xSemaphoreGive(bleScanResultsMutex);
                }
                else
                {
                    Serial.printf("[BLE] Added: %s (%i dbi) [Unable to take semaphore]\n", pData->AddressStr.c_str(), pData->Rssi);
                    bleScanResults.push_back(pData);
                }
            }
        }
    }
};

void setupAdvData(BleSensorData *pData, NimBLEAdvertising *pAdvertising)
{
    char beacon_data[15];
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();

    // ATC format
    beacon_data[0] = pData->AddressData[2];
    beacon_data[1] = pData->AddressData[1];
    beacon_data[2] = pData->AddressData[0];

    beacon_data[3] = 1;
    beacon_data[4] = 1;

    beacon_data[5] = 1;
    beacon_data[6] = pData->Temperature & 0xFF;

    beacon_data[7] = pData->Temperature >> 8;
    beacon_data[8] = pData->Humid & 0xFF;

    beacon_data[9] = pData->Humid >> 8;

    beacon_data[10] = pData->BatteryMv & 0xFF;

    beacon_data[11] = pData->BatteryMv >> 8;
    beacon_data[12] = pData->BatteryLevel;
    beacon_data[13] = pData->Count;
    beacon_data[14] = 1;

    const uint16_t uuid = 0x181A;
    oAdvertisementData.setServiceData(BLEUUID(uuid), std::string(beacon_data, 15));

    pAdvertising->setAdvertisementData(oAdvertisementData);
}

AdvertisedDeviceCallbacks advCallbacks;

void runScan()
{
    NimBLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(&advCallbacks);
    pBLEScan->setActiveScan(false);
    pBLEScan->setInterval(BLE_INTERVAL); // WIFI connection fails in AP mode if interval==window, because of single antena for WIFI and BLE
    pBLEScan->setWindow(BLE_WINDOW_LISTEN);
    pBLEScan->setDuplicateFilter(false);
    pBLEScan->setMaxResults(0);

    pBLEScan->start(0, nullptr);
}

void runAdvertising(BleSensorData *pData)
{
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

    setupAdvData(pData, pAdvertising);

    pAdvertising->setMaxInterval(400); // 400x0.625 = 250 ms
    pAdvertising->setMinInterval(200); // 200x0.625 = 125 ms
    pAdvertising->start();
    Serial.printf("Advertizing %s [%i] for 1s...", pData->AddressStr.c_str(), pData->Count);
    delay(1000);
    pAdvertising->stop();
    delay(100);

    pAdvertising->setMaxInterval(80); // 80x0.625 = 50 ms
    pAdvertising->setMinInterval(80);
    pAdvertising->start();
    Serial.print(" +0.5s...");
    delay(500);
    pAdvertising->stop();
    Serial.println(" done");
}

void printHeapInfo()
{
    static unsigned int lastPrint = 0;
    if (lastPrint < millis())
    {
        lastPrint = millis() + 30000;

        float freeMem = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL) / 1024.0f;
        float maxMem = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL) / 1024.0f;
        float minMem = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL) / 1024.0f;

        Serial.printf("MEM: free %.2f; max %.2f; min %.2f;\n", freeMem, maxMem, minMem);
    }
}

void setup()
{
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);

    delay(2000);

    Serial.begin(9600);

// workaround for hanging serial.print on native USB disconnected
#if ARDUINO_USB_MODE
#if ARDUINO_USB_CDC_ON_BOOT // Serial used for USB CDC
    Serial.setTxTimeoutMs(0);
#endif
#endif

    bleScanResultsMutex = xSemaphoreCreateMutex();
    if (bleScanResultsMutex == NULL)
    {
        Serial.println("[ERR] Cant create mutex");
        Serial.flush();

        while (true)
            ;
    }

    Serial.println("Starting BLE Repeater...");

    BLEDevice::init("");

    runScan();
}

void loop()
{
    esp_task_wdt_reset();

    printHeapInfo();

    BleSensorData *pData = nullptr;

    if (xSemaphoreTake(bleScanResultsMutex, portTICK_PERIOD_MS * 10) == pdTRUE)
    {
        if (bleScanResults.size() != 0)
        {
            pData = bleScanResults.back();
            bleScanResults.pop_back();
        }

        xSemaphoreGive(bleScanResultsMutex);
    }
    else
    {
        Serial.println("Failed to obtain mutex");
    }

    if (pData == nullptr)
    {
        delay(500);
        return;
    }

    // need to deinit BLE to change base address
    BLEDevice::getScan()->stop();
    BLEDevice::deinit(true);

    // -1 for last byte because ESP has two interfaces (active)
    uint8_t new_mac[8] = {pData->AddressData[5], pData->AddressData[4], pData->AddressData[3], pData->AddressData[2], pData->AddressData[1], (uint8_t)(pData->AddressData[0] - 1)};
    esp_err_t e = esp_base_mac_addr_set(new_mac);
    if (e != ESP_OK)
    {
        Serial.println("Failed to set BLE address");
        while (true)
            delay(1000);
    }

    BLEDevice::init("");

    runScan();

    runAdvertising(pData);

    delete pData;
}
