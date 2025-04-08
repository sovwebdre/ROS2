#define ENCODER_A 32  // Encoder Phase A pin
#define ENCODER_B 33  // Encoder Phase B pin

volatile long encoder_count = 0; // Variable to store encoder count

void IRAM_ATTR readEncoder() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoder_count++;  // Forward rotation
  } else {
    encoder_count--;  // Reverse rotation
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, CHANGE);
}

void loop() {
  Serial.print("Encoder Count: ");
  Serial.println(encoder_count);
  delay(100);  // Print every 100ms
}
