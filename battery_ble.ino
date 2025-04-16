#include <ArduinoBLE.h>

// Your specific BMS MAC address
const char* BLE_ADDR = " ";

// BLE UUIDs used by Vestwoods BMS
const char* SVC_UUID = " ";
const char* RD_UUID = " ";
const char* WR_UUID = " ";

// Module name as required for NMEA format
const char* MODULE_NAME = " ";

// Command to send
const uint8_t POLL_CMD[] = { 0x7a, 0, 5, 0, 0, 1, 12, 229, 0xa7 };

BLEDevice peripheral;
BLECharacteristic rxChar;
BLECharacteristic txChar;
bool isConnected = false;
int errors = 0;
const int MAX_ERRORS = 5;

// Battery data structure
struct BatteryData {
  float totalVoltage;
  float current;
  float cellVoltages[4];
  float maxCellVoltage;
  float minCellVoltage;
  int maxCellNum;
  int minCellNum;
  float soc;
  int cycles;
  float surplusCapacity;
  float fullCapacity;
  float pcbTemp;
  float ambientTemp;
  float minTemp;
  float maxTemp;
};

BatteryData batteryData;

void setup() {
  Serial.begin(38400); // Using NMEA-0183HS baud rate
  while (!Serial) delay(10);
  
  Serial.println("Starting Vestwoods BMS with NMEA output");
  Serial.print("Target Device: ");
  Serial.println(BLE_ADDR);
  
  if (!BLE.begin()) {
    Serial.println("Failed to start BLE!");
    while (1) delay(10);
  }

  Serial.println("Scanning for BMS...");
  BLE.scan();
}

uint16_t get16(const uint8_t* buf, int pos) {
  return ((uint16_t)buf[pos] << 8) | buf[pos + 1];
}

// calculate NMEA checksum
uint8_t calculateNMEAChecksum(const char* sentence) {
  uint8_t checksum = 0;
  // start after $ and continue until * or end
  for (int i = 1; sentence[i] != '*' && sentence[i] != 0; i++) {
    checksum ^= sentence[i];
  }
  return checksum;
}

// format and send BD1 message
void sendBD1Message() {
  char message[83];
  snprintf(message, sizeof(message), 
    "$ERBD1,%s,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
    MODULE_NAME,
    batteryData.totalVoltage,
    batteryData.current,
    batteryData.cellVoltages[0],
    batteryData.cellVoltages[1],
    batteryData.cellVoltages[2],
    batteryData.cellVoltages[3],
    batteryData.maxCellVoltage,
    batteryData.minCellVoltage
  );
  
  uint8_t checksum = calculateNMEAChecksum(message);
  char finalMessage[86];
  snprintf(finalMessage, sizeof(finalMessage), "%s*%02X\r\n", message, checksum);
  Serial.print(finalMessage);
}

// format and send BD2 message
void sendBD2Message() {
  char message[83];
  snprintf(message, sizeof(message),
    "$ERBD2,%s,%.1f,%d,%.1f,%.1f,%d,%d,%d,%d",
    MODULE_NAME,
    batteryData.soc,
    batteryData.cycles,
    batteryData.surplusCapacity,
    batteryData.fullCapacity,
    (int)batteryData.pcbTemp,
    (int)batteryData.ambientTemp,
    (int)batteryData.minTemp,
    (int)batteryData.maxTemp
  );
  
  uint8_t checksum = calculateNMEAChecksum(message);
  char finalMessage[86];
  snprintf(finalMessage, sizeof(finalMessage), "%s*%02X\r\n", message, checksum);
  Serial.print(finalMessage);
}

void processResponse(uint8_t* buffer, int length) {
  Serial.print("Raw response (");
  Serial.print(length);
  Serial.println(" bytes):");
  for(int i = 0; i < length; i++) {
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
    if((i + 1) % 16 == 0) Serial.println();
  }
  Serial.println();

  if (buffer[0] != 0x7A || buffer[length-1] != 0xA7) {
    Serial.println("Invalid frame markers, skipping");
    return;
  }

  // get message length (byte 2)
  int msgLen = buffer[2];
  Serial.print("Message length from packet: ");
  Serial.println(msgLen);

  // number of cells (byte 7)
  int numCells = buffer[7];
  Serial.print("Number of cells: ");
  Serial.println(numCells);

  // cell voltages (starting at byte 8)
  for (int i = 0; i < 4; i++) {
    uint16_t cellmV = get16(buffer, 8 + (i * 2));
    batteryData.cellVoltages[i] = (cellmV & 0x7FFF) / 1000.0f;
    Serial.print("Cell ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(batteryData.cellVoltages[i], 3);
    Serial.println("V");
  }

  // max cell info (byte 16 for number, 17-18 for voltage)
  batteryData.maxCellNum = buffer[16];
  uint16_t maxmV = get16(buffer, 17);
  batteryData.maxCellVoltage = (maxmV & 0x7FFF) / 1000.0f;
  Serial.print("Max cell (");
  Serial.print(batteryData.maxCellNum);
  Serial.print("): ");
  Serial.print(batteryData.maxCellVoltage, 3);
  Serial.println("V");

  // min cell info (byte 19 for number, 20-21 for voltage)
  batteryData.minCellNum = buffer[19];
  uint16_t minmV = get16(buffer, 20);
  batteryData.minCellVoltage = (minmV & 0x7FFF) / 1000.0f;
  Serial.print("Min cell (");
  Serial.print(batteryData.minCellNum);
  Serial.print("): ");
  Serial.print(batteryData.minCellVoltage, 3);
  Serial.println("V");

  // current (bytes 22-23)
  uint16_t currentRaw = get16(buffer, 22);
  batteryData.current = ((float)currentRaw / 100.0f) - 300.0f;
  Serial.print("Current: ");
  Serial.print(batteryData.current, 2);
  Serial.println("A");

  // SOC (bytes 24-25)
  uint16_t socRaw = get16(buffer, 24);
  batteryData.soc = socRaw / 100.0f;
  Serial.print("SOC: ");
  Serial.print(batteryData.soc, 1);
  Serial.println("%");

  // tot voltage calculation - sum of cell voltages
  batteryData.totalVoltage = 0;
  for(int i = 0; i < 4; i++) {
    batteryData.totalVoltage += batteryData.cellVoltages[i];
  }
  Serial.print("Total voltage (sum of cells): ");
  Serial.print(batteryData.totalVoltage, 2);
  Serial.println("V");

  // Extra data for BD2
  // From byte trace:
  // 27 10 27 10 shows up at bytes 24-27, looks like SOC
  // 02 00 at bytes 33-34 might be cycle count
  // Temps look wrong, let's adjust their offsets

  // cycle count
  batteryData.cycles = get16(buffer, 33);
  Serial.print("Cycles: ");
  Serial.println(batteryData.cycles);

  // capacity - these offsets look okay
  uint16_t surplusRaw = get16(buffer, 34);
  batteryData.surplusCapacity = surplusRaw / 100.0f;
  uint16_t fullRaw = get16(buffer, 36);
  batteryData.fullCapacity = fullRaw / 100.0f;
  Serial.print("Surplus capacity: ");
  Serial.print(batteryData.surplusCapacity, 1);
  Serial.println("Ah");
  Serial.print("Full capacity: ");
  Serial.print(batteryData.fullCapacity, 1);
  Serial.println("Ah");

  // temperature processing
  Serial.println("\nTemperature bytes in sequence:");
  for(int i = 0; i < 8; i++) {
    Serial.print(buffer[24+i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // try method 1: Direct byte swapped reading
  uint16_t temp1 = (buffer[24] << 8) | buffer[25];  // 0x2710
  uint16_t temp2 = (buffer[26] << 8) | buffer[27];  // 0x2710
  Serial.print("Method 1 - Temps as uint16: ");
  Serial.print(temp1);
  Serial.print(", ");
  Serial.println(temp2);
  
  // try method 2: Divide by 10 (fixed point)
  float tempA = (float)temp1 / 10.0;
  float tempB = (float)temp2 / 10.0;
  Serial.print("Method 2 - Temps divided by 10: ");
  Serial.print(tempA);
  Serial.print(", ");
  Serial.println(tempB);
  
  // try method 3: Simpler byte reading
  int tempC = ((int)buffer[24]) - 40;  // Try different offset
  int tempD = ((int)buffer[26]) - 40;
  Serial.print("Method 3 - Single byte with offset 40: ");
  Serial.print(tempC);
  Serial.print(", ");
  Serial.println(tempD);

  // use whichever method looks closest to 18°C
  int envTemp = tempC;
  int pcbTemp = tempD;
  
  // max/min temps - keep the working method
  int maxTempNumber = buffer[28];
  int maxTemp = ((int8_t)buffer[29]) - 50;
  int minTempNumber = buffer[30];
  int minTemp = ((int8_t)buffer[31]) - 50;

  // set temperatures
  batteryData.pcbTemp = envTemp;
  batteryData.ambientTemp = pcbTemp;
  batteryData.minTemp = minTemp;
  batteryData.maxTemp = maxTemp;
  
  Serial.print("Environmental Temp: ");
  Serial.print(envTemp);
  Serial.println("°C");
  Serial.print("PCB Temp: ");
  Serial.print(pcbTemp);
  Serial.println("°C");
  Serial.print("Max Temp (sensor ");
  Serial.print(maxTempNumber + 1);
  Serial.print("): ");
  Serial.print(maxTemp);
  Serial.println("°C");
  Serial.print("Min Temp (sensor ");
  Serial.print(minTempNumber + 1);
  Serial.print("): ");
  Serial.print(minTemp);
  Serial.println("°C");
  
  Serial.print("PCB Temp: ");
  Serial.print(batteryData.pcbTemp);
  Serial.println("°C");
  Serial.print("Ambient Temp: ");
  Serial.print(batteryData.ambientTemp);
  Serial.println("°C");
  Serial.print("Min Temp: ");
  Serial.print(batteryData.minTemp);
  Serial.println("°C");
  Serial.print("Max Temp: ");
  Serial.print(batteryData.maxTemp);
  Serial.println("°C");

  sendBD1Message();
  sendBD2Message();

  Serial.println("------------------------");
}

void loop() {
  if (!isConnected) {
    BLEDevice device = BLE.available();

    if (device) {
      Serial.print("Found device: ");
      Serial.print(device.address());
      Serial.print(" '");
      Serial.print(device.localName());
      Serial.println("'");
      
      if (device.address() == String(BLE_ADDR)) {
        Serial.println("Found BMS!");
        BLE.stopScan();
        peripheral = device;
        
        if (!peripheral.connect()) {
          Serial.println("Connection failed!");
          peripheral = BLEDevice();
          BLE.scan();
          return;
        }
        
        Serial.println("Connected to BMS");
        
        if (!peripheral.discoverService(SVC_UUID)) {
          Serial.println("Service discovery failed!");
          peripheral.disconnect();
          peripheral = BLEDevice();
          BLE.scan();
          return;
        }
        
        Serial.println("Service found");
        txChar = peripheral.characteristic(WR_UUID);
        rxChar = peripheral.characteristic(RD_UUID);
        
        if (!rxChar || !txChar) {
          Serial.println("Failed to get characteristics!");
          peripheral.disconnect();
          peripheral = BLEDevice();
          BLE.scan();
          return;
        }
        
        Serial.println("Got characteristics");
        
        if (!rxChar.canSubscribe()) {
          Serial.println("Can't subscribe to RX characteristic!");
          peripheral.disconnect();
          peripheral = BLEDevice();
          BLE.scan();
          return;
        }
        
        if (!rxChar.subscribe()) {
          Serial.println("Failed to subscribe!");
          peripheral.disconnect();
          peripheral = BLEDevice();
          BLE.scan();
          return;
        }

        isConnected = true;
        errors = 0;
        Serial.println("Ready to communicate with BMS");
      }
    }
  } else {
    Serial.println("\nSending command:");
    for(int i = 0; i < sizeof(POLL_CMD); i++) {
      if (POLL_CMD[i] < 0x10) Serial.print("0");
      Serial.print(POLL_CMD[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    if (txChar.writeValue(POLL_CMD, sizeof(POLL_CMD))) {
      Serial.println("Command sent successfully");
      delay(100);
      
      if (rxChar.valueUpdated()) {
        uint8_t buffer[256];
        int length = rxChar.readValue(buffer, sizeof(buffer));
        
        if (length > 0) {
          processResponse(buffer, length);
        } else {
          Serial.println("Received zero length response");
          errors++;
        }
      } else {
        Serial.println("No response received");
        errors++;
      }
    } else {
      Serial.println("Failed to send command");
      errors++;
    }
    
    if (errors > MAX_ERRORS) {
      Serial.println("Too many errors - attempting reconnect");
      isConnected = false;
      peripheral.disconnect();
      BLE.scan();
    }
    
    delay(1000);
  }
}
