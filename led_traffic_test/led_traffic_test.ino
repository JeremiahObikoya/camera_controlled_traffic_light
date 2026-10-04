// --- Traffic Light Pins ---
#define LANE1_RED    32
#define LANE1_YELLOW 33
#define LANE1_GREEN  25
#define LANE2_RED    26
#define LANE2_YELLOW 27
#define LANE2_GREEN  14
#define LANE3_RED    12
#define LANE3_YELLOW 15
#define LANE3_GREEN  13

// --- Push Button ---
#define BUTTON_PIN   5

const int totalLanes = 3;
int activeLane = 1; // currently green lane

// Debounce for button
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// --- Function Prototypes ---
void setLights(int lane, bool red, bool yellow, bool green);
void switchToLane(int newActiveLane);
void performRedToGreenTransition(int targetLane);
void performGreenToRedTransition(int currentLane);

// --- Setup ---
void setup() {
  Serial.begin(115200);

  // Init LED pins
  pinMode(LANE1_RED, OUTPUT);
  pinMode(LANE1_YELLOW, OUTPUT);
  pinMode(LANE1_GREEN, OUTPUT);
  pinMode(LANE2_RED, OUTPUT);
  pinMode(LANE2_YELLOW, OUTPUT);
  pinMode(LANE2_GREEN, OUTPUT);
  pinMode(LANE3_RED, OUTPUT);
  pinMode(LANE3_YELLOW, OUTPUT);
  pinMode(LANE3_GREEN, OUTPUT);

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Traffic Light Test initialized.");

  // Start with all red
  setLights(1, true, false, false);
  setLights(2, true, false, false);
  setLights(3, true, false, false);
  activeLane = 1;
  switchToLane(1); // give green to lane 1

  randomSeed(analogRead(0));
}

// --- Main Loop ---
void loop() {
  // Button: random lane change
  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      int newLane = random(1, totalLanes + 1);
      if (newLane != activeLane) {
        Serial.print("Button Pressed: Switching to Lane ");
        Serial.println(newLane);
        switchToLane(newLane);
      } else {
        Serial.println("Same lane chosen, no change.");
      }
      lastDebounceTime = millis();
    }
  }

  delay(50);
}

// --- Traffic Light Functions ---
void setLights(int lane, bool red, bool yellow, bool green) {
  int r, y, g;
  switch(lane) {
    case 1: r = LANE1_RED; y = LANE1_YELLOW; g = LANE1_GREEN; break;
    case 2: r = LANE2_RED; y = LANE2_YELLOW; g = LANE2_GREEN; break;
    case 3: r = LANE3_RED; y = LANE3_YELLOW; g = LANE3_GREEN; break;
    default: return;
  }
  digitalWrite(r, red ? HIGH : LOW);
  digitalWrite(y, yellow ? HIGH : LOW);
  digitalWrite(g, green ? HIGH : LOW);
}

void performRedToGreenTransition(int targetLane) {
  Serial.print("Lane "); Serial.print(targetLane); Serial.println(": Red -> Green");
  setLights(targetLane, true, false, false);
  delay(1000); // red for 1 sec (safety)
  setLights(targetLane, false, false, true);
  Serial.print("Lane "); Serial.print(targetLane); Serial.println(" is now GREEN.");
}

void performGreenToRedTransition(int currentLane) {
  Serial.print("Lane "); Serial.print(currentLane); Serial.println(": Green -> Yellow -> Red");
  setLights(currentLane, false, true, false);
  delay(3000); // yellow for 3 seconds
  setLights(currentLane, true, false, false);
  Serial.print("Lane "); Serial.print(currentLane); Serial.println(" is now RED.");
}

void switchToLane(int newActiveLane) {
  if (newActiveLane < 1 || newActiveLane > totalLanes) return;
  // First, turn off current green (if any)
  if (activeLane >= 1 && activeLane <= totalLanes) {
    performGreenToRedTransition(activeLane);
  }
  delay(100);
  // Then turn on new lane green
  performRedToGreenTransition(newActiveLane);
  activeLane = newActiveLane;
  Serial.print("Now active lane: ");
  Serial.println(activeLane);
}
