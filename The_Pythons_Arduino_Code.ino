// The Pythons' Arduino Code
// Latest_Algorithm  - By: Ahmad Zaki

// These are the pins required to operate the motor driver
#define in_1 7
#define in_2 6
#define ena 5

void setup() {
  // Defining the baud rate to match that of the Openmv
  Serial.begin(19200);

  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(ena, OUTPUT);

}

void loop() {

  if (Serial.available()) { // If something was recieved by the arduino
    // Read the most recent character
    char byteRead = Serial.read();
    Serial.println(byteRead);
    if (byteRead == 'F') { // Fast speed is needed
      digitalWrite(in_1, LOW);
      digitalWrite(in_2, HIGH);
      analogWrite(ena, 220);
    }
    else if (byteRead == 'S') { // Slow speed is needed
      digitalWrite(in_1, LOW);
      digitalWrite(in_2, HIGH);
      analogWrite(ena, 180);
    }

  }

}
