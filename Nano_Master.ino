
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

int state = 0;

void setup() {
  if (!bmp.begin()) {

  }
  sensor_t sensor;
  lcd.init();                      // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(12, OUTPUT); // Green of white
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT); //blue
  pinMode(7, OUTPUT); // green
  pinMode(6, OUTPUT); // yellow
  pinMode(5, OUTPUT); // red
  Serial.begin(115200);
  Serial.print("Begin Test");

  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  if (Serial.available()) {
    delay(100);
    state = Serial.read();
    Serial.println(state);
  }

  if (state == 1) {
    digitalWrite(7, HIGH);
    delay(500);
    digitalWrite(7, LOW);
  }
  else if (state == 2 ) {
    digitalWrite(6, HIGH);
    delay(500);
    digitalWrite(6, LOW);
  }
  else if (state == 3) {
    digitalWrite(5, HIGH);
    delay(500);
    digitalWrite(5, LOW);
  }
  else if (state == 4) {
    digitalWrite(8, HIGH);
    delay(500);
    digitalWrite(8, LOW);
  }
  lcd.setCursor(0, 0);
  lcd.print("Temp in Robot:");

  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);

  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure) {
    lcd.setCursor(0, 1);

    float temperature;
    bmp.getTemperature(&temperature);

    lcd.print(temperature);
    lcd.print(" C");
  }
  delay(200);
}
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);
}
