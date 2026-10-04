// ESP32-CAM LED Flash pin (GPIO 4)
#define FLASH_LED_PIN 4

void setup() {
  Serial.begin(115200);
  
  // Configure the flash LED pin as output
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  
  Serial.println("ESP32-CAM LED Blink Start");
}

void loop() {
  // Turn LED ON
  digitalWrite(FLASH_LED_PIN, HIGH);
  Serial.println("LED ON");
  delay(500);

  // Turn LED OFF
  digitalWrite(FLASH_LED_PIN, LOW);
  Serial.println("LED OFF");
  delay(500);
}
