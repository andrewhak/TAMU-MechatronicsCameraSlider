/*     Stepper Motor using a Rotary Encoder
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
 //#include <LiquidCrystal.h> // includes the LiquidCrystal Library 
// LiquidCrystal lcd(1, 2, 4, 5, 6, 7); // Creates an LC object. Parameters: (rs, enable, d4, d5, d6, d7) 

// defines pins numbers
// #define stepPin 8 
// #define dirPin  9
 #define outputA 8  
 #define outputB 7
 #define outputC 6
 int buttcount= 0;
 int counter = 0;
 int rot0 = 0;
 int rot1 = 0;
 int rot2 = 0;
 int rot3 = 0; 
 int aState;
 int aLastState;  
 unsigned long lastButtonPress = 0;
 int dist1 = 0;
 int dist2 = 0;
 int angle1 = 0;
 int angle2 = 0;
 int timer = 0;
 
void setup() {
  // Sets the two pins as Outputs
 // pinMode(stepPin,OUTPUT); 
 // pinMode(dirPin,OUTPUT);
  pinMode (outputA,INPUT);
  pinMode (outputB,INPUT);
  pinMode (outputC,INPUT_PULLUP);
  Serial.begin(9600);
  aLastState = digitalRead(outputA);
  lcd.begin(16,2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display } 

}

void loop() {
  while (buttcount == 0){
    aState = digitalRead(outputA);
    if (aState != aLastState){     
       if (digitalRead(outputB) != aState) { 
         counter ++;
         rot0 ++;
         //rotateCW();  
       }
       else {
         counter--;
         rot0 --;
         //rotateCCW(); 
       }
       if (counter >=30 ) {
        counter =0;
       }
       if (rot0 < 0){
        rot0 = 1;
       }
       if (rot0 > 1){
        rot0 = 0;
       }
       if (rot0 == 0){
        lcd.clear();
        lcd.print("Free Motion");
       }
       if (rot0 == 1){
        lcd.clear();
        lcd.print("Object Track");
       }
       lcd.setCursor(0,0);
       /*
       Serial.print("Position: ");
       Serial.print(counter);
       Serial.println("deg");
       */ 
     }
    aLastState = aState;
    // Read the button state
    int btnState = digitalRead(outputC); 
    //If we detect LOW signal, button is pressed  
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {  
        lcd.setCursor(0,0);
        buttcount ++;
      }
  
      // Remember last button press event
      lastButtonPress = millis();
    }
  
    // Put in a slight delay to help debounce the reading
    delay(1);
  }
  while (buttcount == 1){
    aState = digitalRead(outputA);
    if (aState != aLastState){     
       if (digitalRead(outputB) != aState) { 
         counter ++;
         rot1 ++;
         //rotateCW();  
       }
       else {
         counter--;
         rot1 --;
         //rotateCCW(); 
       }
       if (counter >=30 ) {
        counter =0;
       }
       if (rot1 < 0 & rot0 == 0){
        rot1 = 3;
       }
       if (rot1 > 3 & rot0 == 0){
        rot1 = 0;
       }
       if (rot1 == 0 & rot0 == 0){
        lcd.clear();
        lcd.print("Translation");
       }
       if (rot1 == 1 & rot0 == 0){
        lcd.clear();
        lcd.print("Rotation");
       }
       if (rot1 == 2 & rot0 == 0){
        lcd.clear();
        lcd.print("Run");
       }
       if (rot1 == 3 & rot0 == 0){
        lcd.clear();
        lcd.print("Go Back");
       }
       lcd.setCursor(0,0);
       /*
       Serial.print("Position: ");
       Serial.print(counter);
       Serial.println("deg");
       */ 
     }
    aLastState = aState;
    // Read the button state
    int btnState = digitalRead(outputC);
  
    //If we detect LOW signal, button is pressed  
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {
        lcd.setCursor(0,0);
        if (rot1 == 3){
          buttcount--;
        }
        else if (rot1 ==2){
          Serial.print(dist1);
          Serial.println();
          Serial.print(dist2);
          Serial.println();
          Serial.print(angle1);
          Serial.println();
          Serial.print(angle2); 
        }
        else {
          buttcount++;
        }
      }
  
      // Remember last button press event
      lastButtonPress = millis();
    }
  
    // Put in a slight delay to help debounce the reading
    delay(1);
  }
  while (buttcount == 2 & rot1 == 0 & rot0 == 0){//Translation Menu
    aState = digitalRead(outputA);
    if (aState != aLastState){     
       if (digitalRead(outputB) != aState) { 
         counter ++;
         rot2 ++;
         //rotateCW();  
       }
       else {
         counter--;
         rot2 --;
         //rotateCCW(); 
       }
       if (counter >=30 ) {
        counter =0;
       }
       if (rot2 < 0){
        rot2 = 2; 
       }
       if (rot2 > 2){
        rot2 = 0;
       }
       if (rot2 == 0){
        lcd.clear();
        lcd.print("Distance 1");
       }
       if (rot2 == 1){
        lcd.clear();
        lcd.print("Distance 2");
       }
       if (rot2 == 2){
        lcd.clear();
        lcd.print("GO BACK");
       }
       lcd.setCursor(0,0);
    }   
    aLastState = aState;
    // Read the button state
    int btnState = digitalRead(outputC);
  
    //If we detect LOW signal, button is pressed  
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {
        lcd.setCursor(0,0);
        if (rot2 == 2){
          buttcount--;
        }
        else {
          buttcount++;
        }
      }
      // Remember last button press event
      lastButtonPress = millis();
    }
  
    // Put in a slight delay to help debounce the reading
    delay(1);
  }
  while (buttcount == 2 & rot1 == 1 & rot0 == 0){//Rotation Menu
    aState = digitalRead(outputA);
    if (aState != aLastState){     
       if (digitalRead(outputB) != aState) { 
         counter ++;
         rot2 ++;
         //rotateCW();  
       }
       else {
         counter--;
         rot2 --;
         //rotateCCW(); 
       }
       if (counter >=30 ) {
        counter =0;
       }
       if (rot2 < 0){
        rot2 = 2; 
       }
       if (rot2 > 2){
        rot2 = 0;
       }
       if (rot2 == 0){
        lcd.clear();
        lcd.print("Angle 1");
       }
       if (rot2 == 1){
        lcd.clear();
        lcd.print("Angle 2");
       }
       if (rot2 == 2){
        lcd.clear();
        lcd.print("GO BACK");
       }
       lcd.setCursor(0,0);
    }   
    aLastState = aState;
    // Read the button state
    int btnState = digitalRead(outputC);
  
    //If we detect LOW signal, button is pressed  
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {
        lcd.setCursor(0,0);
        if (rot2 == 2){
          buttcount--;
        }
        else {
          buttcount++;
        }
      }
      // Remember last button press event
      lastButtonPress = millis();
    }
  
    // Put in a slight delay to help debounce the reading
    delay(1);
  }
  while (buttcount == 3 & rot2 ==0 & rot1 == 0 & rot0 == 0){//Translation Distance1 Menu
    aState = digitalRead(outputA);
    if (aState != aLastState){     
       if (digitalRead(outputB) != aState) { 
         counter ++;
         rot3 ++;
         //rotateCW();  
       }
       else {
         counter--;
         rot3 --;
         //rotateCCW(); 
       }
       if (counter >=30 ) {
        counter =0;
       }
       if (rot3 < 0){
        rot3 = 0;
       }
       dist1 = 1*rot3;
       lcd.clear();
       lcd.print(dist1);
     }
    lcd.setCursor(0,0);
    aLastState = aState;
    // Read the button state
    int btnState = digitalRead(outputC);
    //If we detect LOW signal, button is pressed  
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {
        lcd.setCursor(0,0);
        buttcount--;
      }
  
      // Remember last button press event
      lastButtonPress = millis();
    }
  
    // Put in a slight delay to help debounce the reading
    delay(1);
  }
  while (buttcount == 3 & rot2 ==1 & rot1 == 0 & rot0 == 0){//Translation Distance2 Menu
    aState = digitalRead(outputA);
    if (aState != aLastState){     
       if (digitalRead(outputB) != aState) { 
         counter ++;
         rot3 ++;
         //rotateCW();  
       }
       else {
         counter--;
         rot3 --;
         //rotateCCW(); 
       }
       if (counter >=30 ) {
        counter =0;
       }
       if (rot3 < 0){
        rot3 = 0;
       }
       dist2 = 1*rot3;
       lcd.clear();
       lcd.print(dist2);
     }
    lcd.setCursor(0,0);
    aLastState = aState;
    // Read the button state
    int btnState = digitalRead(outputC);
  
    //If we detect LOW signal, button is pressed  
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {
        lcd.setCursor(0,0);
        buttcount--;
        rot3 = 0;
      }
  
      // Remember last button press event
      lastButtonPress = millis();
    }
  
    // Put in a slight delay to help debounce the reading
    delay(1);
  }
while (buttcount == 3 & rot2 ==0 & rot1 == 1 & rot0 == 0){//Translation Angle1 Menu
    aState = digitalRead(outputA);
    if (aState != aLastState){     
       if (digitalRead(outputB) != aState) { 
         counter ++;
         rot3 ++;
         //rotateCW();  
       }
       else {
         counter--;
         rot3 --;
         //rotateCCW(); 
       }
       if (counter >=30 ) {
        counter =0;
       }
       angle1 = 1*rot3;
       lcd.clear();
       lcd.print(angle1);
     }
    lcd.setCursor(0,0);
    aLastState = aState;
    // Read the button state
    int btnState = digitalRead(outputC);
    //If we detect LOW signal, button is pressed  
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {
        lcd.setCursor(0,0);
        buttcount--;
      }
  
      // Remember last button press event
      lastButtonPress = millis();
    }
  
    // Put in a slight delay to help debounce the reading
    delay(1);
  }
  while (buttcount == 3 & rot2 ==1 & rot1 == 1 & rot0 == 0){//Translation Angle2 Menu
    aState = digitalRead(outputA);
    if (aState != aLastState){     
       if (digitalRead(outputB) != aState) { 
         counter ++;
         rot3 ++;
         //rotateCW();  
       }
       else {
         counter--;
         rot3 --;
         //rotateCCW(); 
       }
       if (counter >=30 ) {
        counter =0;
       }
       angle2 = 1*rot3;
       lcd.clear();
       lcd.print(angle2);
     }
    lcd.setCursor(0,0);
    aLastState = aState;
    // Read the button state
    int btnState = digitalRead(outputC);
  
    //If we detect LOW signal, button is pressed  
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {
        lcd.setCursor(0,0);
        buttcount--;
        rot3 = 0;
      }
  
      // Remember last button press event
      lastButtonPress = millis();
    }
  
    // Put in a slight delay to help debounce the reading
    delay(1);
  }
}
/*
void rotateCW() {
  digitalWrite(dirPin,LOW);
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(2000); 
}
void rotateCCW() {
  digitalWrite(dirPin,HIGH);
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(2000);   
}
*/
