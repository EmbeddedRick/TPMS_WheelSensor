#include "Arduino.h"
#include "ArduinoJson.h"
#include "NimBLEDevice.h"
#include "EEPROM.h"
#include <PubSubClient.h>
#include <WiFi.h>
#include <time.h>
// C libaries
#include <string.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* strtol */

// PINES - LLANTAS
#define LLANTA1                 0
#define LLANTA2                 1
#define LLANTA3                 7
#define LLANTA4                 4
#define LLANTA5                 5
#define LLANTA6                 16
#define LLANTA7                 17
#define LLANTA8                 21
#define LLANTA9                 22
#define LLANTA10                23
#define LLANTA11                33
#define LLANTA12                32
#define LLANTA13                27
#define LLANTA14                26
#define LLANTA15                25
#define LLANTA16                19
#define LLANTA17                18
#define LLANTA18                15

// Se declara el numero max de targets y/o Beacons
#define num_max_targets 18  

uint8_t Pin[18]  = {LLANTA1, LLANTA2, LLANTA3, LLANTA4, LLANTA5, 
                    LLANTA6, LLANTA7, LLANTA8, LLANTA9, LLANTA10, 
                    LLANTA11, LLANTA12, LLANTA13, LLANTA14, LLANTA15, 
                    LLANTA16, LLANTA17, LLANTA18};//array of pins

// Estructura de caracteristicas
struct TPMS_param{
  String MAC_identifier;  
  float temperature; 
  float pressure;
  int battery;  
}deviceTargets[num_max_targets];


NimBLEScan* pBLEScan;
static NimBLEServer* pServer;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID0 "beb5483e-36e1-4688-b7f5-ea07361b26a8" // Advertisement MAC Addresses
#define CHARACTERISTIC_UUID1 "68796cce-9412-46a4-a813-aeb936663d3c" // Advertisement Placa Bus
#define CHARACTERISTIC_UUID2 "9d764990-cce1-4565-b511-66bb397934d7" // Advertisement Cant. Sensores
#define CHARACTERISTIC_UUID3 "7b354e36-e86d-49e7-88e9-889cc0702e65" // Advertisement Presion Estandar
#define CHARACTERISTIC_UUID4 "0b142b46-9690-4a1e-9729-5bac0fcc1774" // Advertisement Presion Porcentaje


// Direccion en la memoria EEPROM
#define EEPROM_SIZE 255    // Son 18 targets * 17 bytes de MAC address + 19 = 325
const int Posc_Init_Cnt_Sensores = 0;
const int Posc_Init_Set_Press= 1;
const int Posc_Init_Percent_Press = 2;
const int Posc_Init_Placa = 3;
const int Posc_Init_Address = 9;

uint8_t cnt_sensores = 1;
uint8_t beacon_selection = 0;
uint8_t set_press = 0;
float percent_press = 0;
String placa = "";

#define   WIFI_TIMEOUT_MS 20000
//const char* ssid       = "www.icm.com";
//const char* password   = "Dakar*2024";
const char* ssid       = "WIN007";
const char* password   = "Risof0823";
const char *mqtt_server = "154.53.50.241"; //Servidor mqtt del sistema
WiFiClient espClient; //Cliente del esp
PubSubClient client(espClient);

// WATCHDOG
unsigned long currentMillis;
unsigned long previousButtonMillis;
const int intervalButton = 1000; 


// Prototypes
void loadIdentifiers(void);
void loadPlaca(void);
void loadCntSensores(void);
void loadPressureStd(void);
void loadPressurePrcnt(void);

String ordenarCadena(String rawData){
    uint8_t i = 0;
    String a = "";
    String b = "";
    while(i<rawData.length()){
        a=rawData.substring(i,2);
        b= a + b;
        i = i+2;
    }
    //cout << b;
    return b;
}

void mqtt_subscribe() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("prueba"); // Sujeto a cambios
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//Funcion para armar Json para enviar
void mqtt_publish(struct TPMS_param* miObjeto, uint8_t ID){
  // Crear un objeto JSON y asignar valores desde la estructura
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["id"] = ID;
  jsonDoc["temperature"] = miObjeto->temperature;
  jsonDoc["pressure"] = miObjeto->pressure;
  jsonDoc["batteryLevel"] = miObjeto->battery;

  // Convertir el objeto JSON en una cadena de caracteres (string)
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  Serial.println(jsonString);
  client.publish("prueba", jsonString.c_str());
}

void mqtt_packets(void * parameters){
  for(;;){
    if(WiFi.status()==WL_CONNECTED){
      //Serial.println("WiFi still connected");
      // ESCRIBIR CODIGO DE ENVIO A LA NUBE
      client.loop();
      for(int posc_sensor = 0; posc_sensor < cnt_sensores; posc_sensor++){
        mqtt_publish(&deviceTargets[posc_sensor], posc_sensor+1);
      }
      vTaskDelay(10000/portTICK_PERIOD_MS);
      continue;
    }
    Serial.println("WiFi connecting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    unsigned long startAttempTime = millis();
    // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
    while((WiFi.status() != WL_CONNECTED) && (millis() - startAttempTime < WIFI_TIMEOUT_MS)) {}
    if(WiFi.status()!=WL_CONNECTED){
      Serial.println("WiFi [FAILED]");
      vTaskDelay(20000/portTICK_PERIOD_MS);
      continue;
    }
    client.setServer(mqtt_server, 8083);
    mqtt_subscribe();
  }
}

class ServerCallbacks: public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) {
      Serial.println("Client connected");
      digitalWrite(8, HIGH);
  };
  void onDisconnect(BLEServer* pServer) {
    Serial.println("Client disconnected");
      digitalWrite(8, LOW);
    }
};


class CharacteristicUUID0: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
      pCharacteristic->setValue(deviceTargets[beacon_selection-1].MAC_identifier);
    };
    void onWrite(NimBLECharacteristic* pCharacteristic) {
      String pChar0_value_str = pCharacteristic->getValue();
      Serial.println(pChar0_value_str);
      Serial.println(pChar0_value_str.length());
      if(pChar0_value_str.length()<=2){
        // El identificador MAC del indice seleccionado en la app se muestra
        beacon_selection = pChar0_value_str.toInt();
        String address_id = deviceTargets[beacon_selection-1].MAC_identifier;
        pCharacteristic->setValue(address_id);
        // Notificamos el cambio
        pCharacteristic->notify();
      }else{
        // Se escribe el identificador MAC del beacon en la memoria EEPROM
        for(uint8_t i = 0; i<6; i++){
          EEPROM.write(Posc_Init_Address + (beacon_selection-1)*6 + i, pChar0_value_str[i]);
        }
        EEPROM.commit();
        // Se actualiza el inedx-1 del beacons whitelist
        deviceTargets[beacon_selection-1].MAC_identifier = pChar0_value_str.c_str();
      }
    };
};

class CharacteristicUUID1: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
      // readPlaca de la EEPROM
      pCharacteristic->setValue(placa);
    };

    void onWrite(NimBLECharacteristic* pCharacteristic) {
      String pChar1_value_str = pCharacteristic->getValue().c_str();
      pCharacteristic->setValue(pChar1_value_str); placa = pChar1_value_str;
      EEPROM.writeString(Posc_Init_Placa, placa);
      EEPROM.commit();
      Serial.println(pChar1_value_str);
    };
};

class CharacteristicUUID2: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
      pCharacteristic->setValue((String)cnt_sensores);
    };

    void onWrite(NimBLECharacteristic* pCharacteristic) {
      String pChar2_value_str = pCharacteristic->getValue().c_str();
      pCharacteristic->setValue(pChar2_value_str); cnt_sensores = pChar2_value_str.toInt();
      EEPROM.write(Posc_Init_Cnt_Sensores, cnt_sensores);
      EEPROM.commit();
      Serial.println(pChar2_value_str);
    };
};

class CharacteristicUUID3: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
      pCharacteristic->setValue((String)set_press);
    };

    void onWrite(NimBLECharacteristic* pCharacteristic) {
      String pChar3_value_str = pCharacteristic->getValue().c_str();
      pCharacteristic->setValue(pChar3_value_str); 
      set_press = pChar3_value_str.toInt();
      EEPROM.write(Posc_Init_Set_Press, set_press);
      EEPROM.commit();
      Serial.println(set_press);
    };
};

class CharacteristicUUID4: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
      pCharacteristic->setValue((String)percent_press);
    };

    void onWrite(NimBLECharacteristic* pCharacteristic) {
      String pChar4_value_str = pCharacteristic->getValue().c_str();
      pCharacteristic->setValue(pChar4_value_str); 
      percent_press = pChar4_value_str.toInt();
      EEPROM.write(Posc_Init_Percent_Press, percent_press);
      EEPROM.commit();
      Serial.println(percent_press);
    };
};

class AdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {

    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
      std::string res;
      if(advertisedDevice->haveName()){
        String temp_name = advertisedDevice->getName().c_str();
        if(temp_name.indexOf("TPMS")!=-1){
          Serial.printf("Advertised Device: %s \n", temp_name);
          uint8_t undelineIndex = temp_name.indexOf('_');
          String macID = temp_name.substring(undelineIndex + 1);
          Serial.printf("Its MAC identifier: %s \n", macID);
          for(int posc_sensor = 0; posc_sensor<cnt_sensores; posc_sensor++){
            String lowercase_MAC = deviceTargets[posc_sensor].MAC_identifier;
            lowercase_MAC.toLowerCase();
            if(strcmp(macID.c_str(),lowercase_MAC.c_str())==0){
              char *pHex = NimBLEUtils::buildHexData(nullptr, (uint8_t *)advertisedDevice->getManufacturerData().data(), advertisedDevice->getManufacturerData().length());
              res += pHex;
              String Value_Hex = res.c_str();
              Value_Hex = Value_Hex.substring(4, Value_Hex.length());
              Serial.println(Value_Hex);
              free(pHex);
              Serial.println("----------------------------------------------------------------");
              Serial.println("A obtener la data...");

              //String Address_Temp = Value_Hex.substring(0, 12);

              char Pressure_Char[13] = "";
              for (int i = 3; i >= 0; i--) {
                strcat(Pressure_Char, Value_Hex.substring(12 + i * 2, 12 + i * 2 + 2).c_str());
              }

              char Temperature_char[13] = "";
              for (int i = 3; i >= 0; i--) {
                strcat(Temperature_char, Value_Hex.substring(20 + i * 2, 20 + i * 2 + 2).c_str());
              }

              char Batt_char[3] = "";
              strcat(Batt_char, Value_Hex.substring(28, 30).c_str());

              deviceTargets[posc_sensor].pressure = (float)strtol(Pressure_Char, 0, 16) / 100000;
              deviceTargets[posc_sensor].temperature = (float)strtol(Temperature_char, 0, 16) / 100;
              deviceTargets[posc_sensor].battery = (int)strtol(Batt_char, 0, 16);

              /*
              String addressTemp = Value_Hex.substring(0, 12);
              String pressure = Value_Hex.substring(12, 20);
              String temperature = Value_Hex.substring(20, 28);
              String battery = Value_Hex.substring(28, 30);

              //Serial.println(addressTemp);
              Serial.println(pressure);
              Serial.println(temperature);
              Serial.println(battery);
              */
            }
          }
        }
      }
    }
};

/* Define callback instances globally to use for multiple Charateristics */
static CharacteristicUUID0 identifierChr;
static CharacteristicUUID1 placaChr;
static CharacteristicUUID2 sensorChr;
static CharacteristicUUID3 pressureStdChr;
static CharacteristicUUID4 pressurePrcntChr;

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 3; i++) { //for each pin
    pinMode (Pin[i], OUTPUT);       // set as output
  }
  
  // Inicializamos la memoria EEPROM
  Serial.println("\nTesting EEPROM Library\n");
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  loadCntSensores();
  loadIdentifiers();
  loadPlaca();
  loadPressureStd();
  loadPressurePrcnt();

  Serial.println("Starting NimBLE Server");
  //NimBLEDevice::setScanDuplicateCacheSize(200);
  NimBLEDevice::init("BLE-Central");

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create a BLE Characteristic0: MAC IDENTIFIER
  NimBLEService* pVehicleService = pServer->createService(SERVICE_UUID);
  NimBLECharacteristic* pBeaconCharacteristic = pVehicleService->createCharacteristic(
                CHARACTERISTIC_UUID0,
                NIMBLE_PROPERTY::READ |
                NIMBLE_PROPERTY::WRITE |
                NIMBLE_PROPERTY::NOTIFY
                );
  pBeaconCharacteristic->setValue(deviceTargets[0].MAC_identifier);
  pBeaconCharacteristic->setCallbacks(&identifierChr);

  // Create a BLE Characteristic1: Numero de Placa
  NimBLECharacteristic* pPlacaCharacteristic = pVehicleService->createCharacteristic(
                CHARACTERISTIC_UUID1,
                NIMBLE_PROPERTY::READ |
                NIMBLE_PROPERTY::WRITE |
                NIMBLE_PROPERTY::INDICATE
                );
  pPlacaCharacteristic->setValue(placa);
  pPlacaCharacteristic->setCallbacks(&placaChr);

  // Create a BLE Characteristic2: Cant. Sensores
  NimBLECharacteristic* pSensorsCharacteristic = pVehicleService->createCharacteristic(
                CHARACTERISTIC_UUID2,
                NIMBLE_PROPERTY::READ |
                NIMBLE_PROPERTY::WRITE |
                NIMBLE_PROPERTY::INDICATE
                );
  pSensorsCharacteristic->setValue(cnt_sensores);
  pSensorsCharacteristic->setCallbacks(&sensorChr);

  // Create a BLE Characteristic3: Presión Estandar
  NimBLECharacteristic* pStandarCharacteristic = pVehicleService->createCharacteristic(
                CHARACTERISTIC_UUID3,
                NIMBLE_PROPERTY::READ |
                NIMBLE_PROPERTY::WRITE |
                NIMBLE_PROPERTY::INDICATE
                );
  pStandarCharacteristic->setValue(set_press);
  pStandarCharacteristic->setCallbacks(&pressureStdChr);

  // Create a BLE Characteristic4: Presión Porcentaje
  NimBLECharacteristic* pPercentCharacteristic = pVehicleService->createCharacteristic(
                CHARACTERISTIC_UUID4,
                NIMBLE_PROPERTY::READ |
                NIMBLE_PROPERTY::WRITE |
                NIMBLE_PROPERTY::INDICATE
                );
  pPercentCharacteristic->setValue(percent_press);
  pPercentCharacteristic->setCallbacks(&pressurePrcntChr);

  /** Start the services when finished creating all Characteristics and Descriptors */
  pVehicleService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  /** Add the services to the advertisment data **/
  pAdvertising->addServiceUUID(pVehicleService->getUUID());
  /** If your device is battery powered you may consider setting scan response
   *  to false as it will extend battery life at the expense of less data sent.
   */
  //pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("Advertising Started");

  Serial.println("Scanning...");
  pBLEScan = NimBLEDevice::getScan(); //create new scan
  // Set the callback for when devices are discovered, no duplicates.
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); // Set active scanning, this will get more data from the advertiser.
  pBLEScan->setInterval(97); // How often the scan occurs / switches channels; in milliseconds,
  pBLEScan->setWindow(37);  // How long to scan during the interval; in milliseconds.
  pBLEScan->setMaxResults(0); // do not store the scan results, use callback only.
  
  xTaskCreatePinnedToCore(
    mqtt_packets,
    "SEND TO BROKER",
    5000,
    NULL,1,NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  );
}

void loop() {
  // If an error occurs that stops the scan, it will be restarted here.
  if(pBLEScan->isScanning() == false) {
    //Serial.println("I'm scanning");
      // Start scan with: duration = 0 seconds(forever), no scan end callback, not a continuation of a previous scan.
      pBLEScan->start(5, false);
  }
  currentMillis = millis();
  if(currentMillis - previousButtonMillis > intervalButton){
    float minPressure = set_press-percent_press/100;
    float maxPressure = set_press+percent_press/100;
    for(int Pos = 0; Pos<cnt_sensores; Pos++){
      //float beacon_pressure = deviceTargets[Pos].pressure;
      if(deviceTargets[Pos].pressure < maxPressure && deviceTargets[Pos].pressure > minPressure){
        Serial.println("Valor dentro del rango");
        digitalWrite(Pin[Pos], LOW);
      }else if(deviceTargets[Pos].pressure > maxPressure){
        Serial.println("Valor excede el maximo");
        digitalWrite(Pin[Pos], !digitalRead(Pin[Pos]));
      }else if(deviceTargets[Pos].pressure < minPressure){
        Serial.println("Valor esta por debajo del minimo");
        digitalWrite(Pin[Pos], HIGH);
      }
    }
    previousButtonMillis = currentMillis;
  }
}

void loadIdentifiers(void){
  for (int Posc_Sensor = 0; Posc_Sensor < cnt_sensores; Posc_Sensor++) {
    String temp_identifier = "";
    for (int Posc = 0; Posc < 6; Posc++) {
      int char_read = EEPROM.read(Posc_Init_Address + (Posc_Sensor)*6 + Posc);
      Serial.println((char)char_read);
      if ((char_read >= 48 && char_read <= 57) || (char_read >= 97 && char_read <= 102) || (char_read >= 65 && char_read <= 70)) {
        temp_identifier += char(char_read);
      } else {
        temp_identifier = "FFFFFF";
        //break;
      }
    }
    deviceTargets[Posc_Sensor].MAC_identifier = temp_identifier;
    Serial.println("\nSensor " + String(Posc_Sensor + 1) + " " + deviceTargets[Posc_Sensor].MAC_identifier);
  }
}
void loadPlaca(void) {
  for (int Posc = 0; Posc <= 5; Posc++) {
    int num_read = EEPROM.read(Posc_Init_Placa + Posc);
    if (num_read <= 122 && num_read >= 48) {
      placa += char(num_read);
      /*
      if (Posc == 2) {
        placa += "-";
      }
      */
    } else {
      placa = "FFFFFF";
      break;
    }
  }
  Serial.println("\nPlaca: ");
  Serial.println(placa);
}
void loadCntSensores(void) {
  if (EEPROM.read(Posc_Init_Cnt_Sensores) <= num_max_targets) {
    cnt_sensores = EEPROM.read(Posc_Init_Cnt_Sensores);
  } else {
    cnt_sensores = 0;
  }
  Serial.println("\nCantidad de sensores: ");
  Serial.println(cnt_sensores);
}
void loadPressureStd(void){
  set_press = EEPROM.read(Posc_Init_Set_Press);
  Serial.println("\nPresion de llantas seteada a: ");
  Serial.println(set_press);
}
void loadPressurePrcnt(void){
  percent_press = EEPROM.read(Posc_Init_Percent_Press);
  Serial.println("\nVariacion en porcentaje: ");
  Serial.println(percent_press);
}