#define encodPinA 2
#define encodPinB 4
#define IN1 6
#define IN2 5
#define IN3 7
#define IN4 8
#define ENA 9
#define ENB 10

int xung = 0;

void setup() {
  pinMode(encodPinA, INPUT_PULLUP);
  pinMode(encodPinB, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);

  attachInterrupt(digitalPinToInterrupt(encodPinA), DemXung, FALLING);
  Serial.begin(9600);
}

void loop() {
    Serial.print("So xung hien tai: ");
    Serial.print(xung);
    Serial.println("...");

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void DemXung() {
  // Đếm số xung từ bộ mã hóa
  if (digitalRead(encodPinB) == 0) {
    xung++;
  } else {
    xung--;
  }
}
