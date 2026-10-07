// --- Traffic Light Pins (unchanged) ---
#define LANE1_RED   32
#define LANE1_YELLOW 33
#define LANE1_GREEN  25
#define LANE2_RED    26
#define LANE2_YELLOW 27
#define LANE2_GREEN  14
#define LANE3_RED    12
#define LANE3_YELLOW 15
#define LANE3_GREEN  13

void setup() {
  // Set all pins as OUTPUT
  pinMode(LANE1_RED, OUTPUT);
  pinMode(LANE1_YELLOW, OUTPUT);
  pinMode(LANE1_GREEN, OUTPUT);
  pinMode(LANE2_RED, OUTPUT);
  pinMode(LANE2_YELLOW, OUTPUT);
  pinMode(LANE2_GREEN, OUTPUT);
  pinMode(LANE3_RED, OUTPUT);
  pinMode(LANE3_YELLOW, OUTPUT);
  pinMode(LANE3_GREEN, OUTPUT);
}

void loop() {
  // Set all pins HIGH
  digitalWrite(LANE1_RED, HIGH);
  digitalWrite(LANE1_YELLOW, HIGH);
  digitalWrite(LANE1_GREEN, HIGH);
  digitalWrite(LANE2_RED, HIGH);
  digitalWrite(LANE2_YELLOW, HIGH);
  digitalWrite(LANE2_GREEN, HIGH);
  digitalWrite(LANE3_RED, HIGH);
  digitalWrite(LANE3_YELLOW, HIGH);
  digitalWrite(LANE3_GREEN, HIGH);
  
  delay(1000); // Wait 1 second
  
  // Set all pins LOW
  digitalWrite(LANE1_RED, LOW);
  digitalWrite(LANE1_YELLOW, LOW);
  digitalWrite(LANE1_GREEN, LOW);
  digitalWrite(LANE2_RED, LOW);
  digitalWrite(LANE2_YELLOW, LOW);
  digitalWrite(LANE2_GREEN, LOW);
  digitalWrite(LANE3_RED, LOW);
  digitalWrite(LANE3_YELLOW, LOW);
  digitalWrite(LANE3_GREEN, LOW);
  
  delay(1000); // Wait 1 second
}