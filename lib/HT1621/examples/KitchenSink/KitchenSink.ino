#include <HT1621.h>
#define LCD_CS_PIN 6
#define LCD_WR_PIN 7
#define LCD_DATA_PIN 8
#define DELAY_SINK 1000

HT1621 lcd;
void setup() {
  lcd.begin(LCD_CS_PIN, LCD_WR_PIN, LCD_DATA_PIN);
  lcd.backlight();
  lcd.clear();
  lcd.setBatteryLevel(3);
}

void loop(){
  lcd.clear();
  lcd.print("AaBbCc");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("DdEeFf");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("GgHhIi");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("JjKkLl");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("MmNnOo");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("PpQqRr");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("SsTtUu");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("VvWwXx"); 
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("YyZz  ");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("123456");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("7890", true); // left padded
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("  _-|*"); // Spaces
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("HELLO");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("HT1621");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("bye");
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print("123*", true); // Degrees
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print((long)123456);
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print(1234.10, 2);
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print(999.123234);
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print(1250.0, 3); 
  delay(DELAY_SINK);
  lcd.clear();
  lcd.print(111.1111, 3);
  delay(DELAY_SINK);
}