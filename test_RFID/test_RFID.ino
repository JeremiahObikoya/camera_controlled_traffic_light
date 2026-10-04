#include <SPI.h>
#include <MFRC522.h>

// --- Shared SPI Pins ---
#define SPI_SCK   18
#define SPI_MOSI  23
#define SPI_MISO  19

// --- SHARED RFID Reset ---
// Wire the RST pin of ALL THREE readers to pin 22
#define SHARED_RFID_RST 22 

// --- RFID Readers per Lane ---
// Lane 1
#define RFID1_SS   4
MFRC522 mfrc522_1(RFID1_SS, SHARED_RFID_RST);

// Lane 2 (Moved SS to 21 to free up 16)
#define RFID2_SS   21 
MFRC522 mfrc522_2(RFID2_SS, SHARED_RFID_RST);

// Lane 3 
#define RFID3_SS   2
MFRC522 mfrc522_3(RFID3_SS, SHARED_RFID_RST);

// Array of pointers for easy iteration
MFRC522* rfidReaders[3] = { &mfrc522_1, &mfrc522_2, &mfrc522_3 };
const int laneForReader[3] = {1, 2, 3}; // lane number for each reader

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial monitor to connect

  Serial.println("\n=================================");
  Serial.println("  ESP32 3x RFID Card ID Scanner");
  Serial.println("=================================");

  // Initialize SPI bus with custom pins
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

  // Initialize each reader and print firmware versions
  for (int i = 0; i < 3; i++) {
    rfidReaders[i]->PCD_Init();
    delay(50);
    
    Serial.print("Reader Lane ");
    Serial.print(laneForReader[i]);
    Serial.print(" (SS Pin ");
    if (i == 0) Serial.print(RFID1_SS);
    else if (i == 1) Serial.print(RFID2_SS);
    else Serial.print(RFID3_SS);
    Serial.print("): ");
    
    // Check firmware version to verify SPI communication
    byte v = rfidReaders[i]->PCD_ReadRegister(MFRC522::VersionReg);
    if (v == 0x91 || v == 0x92) {
      Serial.print("ONLINE (Version: 0x");
      Serial.print(v, HEX);
      Serial.println(")");
    } else if (v == 0x00 || v == 0xFF) {
      Serial.print("CHECK WIRING! (Version: 0x");
      Serial.print(v, HEX);
      Serial.println(")");
    } else {
      Serial.print("UNKNOWN (Version: 0x");
      Serial.print(v, HEX);
      Serial.println(")");
    }
  }

  Serial.println("\nReady! Tap any RFID card/tag on Reader 1, 2, or 3...");
  Serial.println("---------------------------------------------------------");
}

void loop() {
  for (int i = 0; i < 3; i++) {
    MFRC522* reader = rfidReaders[i];

    // Look for new cards and read serial
    if (reader->PICC_IsNewCardPresent() && reader->PICC_ReadCardSerial()) {
      Serial.println();
      Serial.println("==========================================");
      Serial.print(" CARD DETECTED on Lane ");
      Serial.print(laneForReader[i]);
      Serial.print(" (SS Pin ");
      if (i == 0) Serial.print(RFID1_SS);
      else if (i == 1) Serial.print(RFID2_SS);
      else Serial.print(RFID3_SS);
      Serial.println(")");
      Serial.println("------------------------------------------");
      
      // Build Card HEX String & 32-bit Decimal Value
      String hexID = "";
      unsigned long decID = 0;
      
      for (byte b = 0; b < reader->uid.size; b++) {
        if (reader->uid.uidByte[b] < 0x10) hexID += "0";
        hexID += String(reader->uid.uidByte[b], HEX);
        if (b < reader->uid.size - 1) hexID += " ";
        
        decID = (decID << 8) | reader->uid.uidByte[b];
      }
      hexID.toUpperCase();

      // 1. HEX ID (e.g., 55 66 77 88)
      Serial.print(" Card ID (HEX)  : ");
      Serial.println(hexID);

      // 2. Decimal ID (e.g., 1432778632)
      Serial.print(" Card ID (DEC)  : ");
      Serial.println(decID);

      // 3. C-Array format for direct use in code
      Serial.print(" C-Array Code   : {");
      for (byte b = 0; b < reader->uid.size; b++) {
        Serial.print("0x");
        if (reader->uid.uidByte[b] < 0x10) Serial.print("0");
        Serial.print(reader->uid.uidByte[b], HEX);
        if (b < reader->uid.size - 1) Serial.print(", ");
      }
      Serial.println("}");

      // 4. Card Type
      MFRC522::PICC_Type piccType = reader->PICC_GetType(reader->uid.sak);
      Serial.print(" Card Type      : ");
      Serial.println(reader->PICC_GetTypeName(piccType));
      Serial.println("==========================================");

      // Halt card and stop crypto
      reader->PICC_HaltA();
      reader->PCD_StopCrypto1();
      
      delay(300); // Prevent duplicate rapid reads
    }
  }

  delay(20);
}

