

#include <Arduino.h>
#include <Keypad.h>


#define PIN_BRIGHTNESS A3
#define PIN_TEMPERATURE A4
#define PIN_COLOR A5

#define POT_TOLERANCE 5


#define PIN_RED 10
#define PIN_GREEN 9
#define PIN_BLUE 11

#define AFTERGLOW 2000


const byte rows = 5; // five rows
const byte cols = 7; // three columns
char keys[rows][cols] = {
  {1,4,7,10,13,16,     19},
  {2,5,8,11,14,17,     20},
  {3,6,9,12,15,18,     21},

  {22,23,24,25,26,27,  34},
  {28,29,30,31,32,33,  35},
};
byte rowPins[rows] = {
                      16,// I  I  I  I  I  I    I
                      15,// 0  0  0  0  0  0    0
                      14,// S  S  S  S  S  S    S
                      
                      13,// 1  2  3  4  5  6    P1
                      12};//7  8  9  10 11 12   P2
byte colPins[cols] =       {8, 7, 6, 5, 4, 3,   2}; 
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, rows, cols );



// var to store raw potentiometer values of brightness, temperature and color pots
uint16_t pot_b;
uint16_t pot_t;
uint16_t pot_c;

// vars to store the scaled potentiometer value of brightness, temperature and color pots
uint8_t pots_b;
uint8_t pots_t;
uint8_t pots_c;

uint8_t lasts_b;
uint8_t lasts_t;
uint8_t lasts_c;

// variables for color
uint16_t hue;

uint8_t r;
uint8_t g;
uint8_t b;


// variables for the color LED
bool led_active = false;
long led_start_time;


// varialbes used for the "set" and "power" command
uint16_t set_pot_val;
char set_cmd = 'b';

char controll_cmd = 'p';
//uint8_t out_devices[7] = {2, 0, 5, 3, 1, 6, 4};
uint8_t out_device = 0;



void hue2rgb(int16_t hue, uint8_t * outred, uint8_t * outgreen, uint8_t * outblue){
  int8_t red = 0;
  int8_t green = 0;
  int8_t blue = 0;

  uint8_t h_section = hue / 60;
  switch (h_section)
  {
  case 0:
    red = 59;
    green = hue;
    blue = 0;
    break;
  
  case 1:
    red = abs(hue - 119);
    green = 59;
    blue = 0;
    break;

  case 2:
    red = 0;
    green = 59;
    blue = hue - 120;
    break;
  
  case 3:
    red = 0;
    green = abs(hue - 239);
    blue = 59;
    break;

  case 4:
    red = hue - 240;
    green = 0;
    blue = 59;
    break;

  case 5:
    red = 59;
    green = 0;
    blue = abs(hue - 359);
    break;
  }
  *outred = map(red, 0, 59, 0, 255);
  *outgreen = map(green, 0, 59, 0, 255);
  *outblue = map(blue, 0, 59, 0, 255);
}


void setup() {
  Serial.begin(115200);

  // define Pinmodes
  pinMode(PIN_BRIGHTNESS, INPUT);
  pinMode(PIN_TEMPERATURE, INPUT);
  pinMode(PIN_COLOR, INPUT);
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);
  
}


void loop() {
  
  // we read the keypad and set a variable in case a key is pressed
  uint8_t key = keypad.getKey();
  if (key != NO_KEY){
    Serial.println(key);    // printing it for debugging
    if (key < 22) {         // if the keynr is smaller than 22 it is a direct controll key
    Serial.print("modulo 3: ");
    Serial.println(key % 3);
    Serial.print("minus 1 divide 3: ");
    Serial.println((key - 1) / 3);
    
    // by dividing by 3 we get the row if the count starts at zero so we have to subtract one first
    out_device = (key - 1) / 3;
    
    switch (key%3)
    {
    case 1:     // 1 corresponds to the first row
      Serial.print("+++p;"); Serial.print(out_device); Serial.println(";1;;;");
      break;
    
    case 2:     // 2 corresponds to the second row
      Serial.print("+++p;"); Serial.print(out_device); Serial.println(";0;;;");
      break;
    
    case 0:
      Serial.print("+++s;"); Serial.print(out_device); Serial.print(";"); Serial.print(set_cmd); Serial.print(";"); Serial.print(set_pot_val); Serial.println(";;");
      break;
      
    default:
      break;
    }


    }else if (key < 34){    // if the keynr is bigger than 22 but smaller than 34 it is a scene key
      Serial.print("+++z;"); Serial.print(key - 22); Serial.println(";;;;");     // sending the scene command to the esp
    }else{                  // otherwise it is a color preset key
      Serial.println("color preset");
    }
  }



  // readin the potentiometer values and scaling them down to 8 bits
  pot_b = analogRead(A3);
  pots_b = map(pot_b, 0, 1023, 0, 255);

  pot_t = analogRead(A4);
  pots_t = map(pot_t, 0, 1023, 0, 255);

  pot_c = analogRead(A5);
  pots_c = map(pot_c, 0, 1023, 0, 255);

  // print the results to the Serial Monitor for debugging:
  /*
  Serial.print(pots_b);
  Serial.print("\t");
  Serial.print(pots_t);
  Serial.print("\t");
  Serial.print(pots_c);
  // set min and max value for the serial plotter
  Serial.println("\t0\t255");
  */


  // this processing for setting the led color is only nessesary when the led_active flag is set
  if (led_active){
    hue = map(pots_c, 0, 255, 0, 359);
    hue2rgb(hue, &r, &g, &b);
    analogWrite(PIN_RED, r);
    analogWrite(PIN_GREEN, g);
    analogWrite(PIN_BLUE, b);
  }

  // next we check if enough time has passed since the color pot was moved. In that case the LED would be turned of
  if (millis() - led_start_time >= AFTERGLOW && led_active){
    analogWrite(PIN_RED, 0);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE, 0);
    led_active = false;
  }


  // now we check if the potentiometers have been moved by any significant amount that is defined by the POT_TOLERANCE
  if (pots_b > lasts_b + POT_TOLERANCE || pots_b < lasts_b - POT_TOLERANCE){
    lasts_b = pots_b;
    set_pot_val = map(pots_b, 0, 255, 0, 1023);
    set_cmd = 'b';
  }else if (pots_t > lasts_t + POT_TOLERANCE || pots_t < lasts_t - POT_TOLERANCE){
    lasts_t = pots_t;
    set_pot_val = map(pots_t, 0, 255, 0, 1023);
    set_cmd = 't';
  }else if (pots_c > lasts_c + POT_TOLERANCE || pots_c < lasts_c - POT_TOLERANCE){
    lasts_c = pots_c;
    set_pot_val = map(pots_c, 0, 255, 0, 1023);
    set_cmd = 'c';

    // when the color pot is moved by a significan amount the led_active flag is set and a timer will begin
    // by saving the time it turned on in a variable. This is needed for the color LED to be active a certain 
    // amount of time while the color pot is not moved anymore
    led_active = true;
    led_start_time = millis();
  }

}