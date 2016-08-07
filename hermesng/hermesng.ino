// LED imports.
#include <FastLED.h>

// Accel imports.
#include <Wire.h>
#include <Adafruit_LSM303_Old.h>

// Our custom data type.
#include "AccelReading.h"

/* Run parameters: */
#define MAX_BRIGHTNESS 0.75 // Max LED brightness.
#define MIN_BRIGHTNESS 0.3

/* Rain parameters */
#define RAIN_BRIGHTNESS 0.5

/* Neopixel parameters: */
#define NUM_LEDS 40
#define DATA_PIN 6

/* Animation parameters: */
// ~15 ms minimum crawl speed for normal mode,
// ~2 ms minimum for superfast hack mode.
#define CRAWL_SPEED_MS 5
// General sensitivity of the animation.
// Raising this raises the vector magnitude needed to reach max (purple),
// and thus lowers sensitivity.
// Eg: 800 = more sensitive, 1600 = less sensitive
#define HERMES_SENSITIVITY 1600.0

// Emulate two strips by starting the crawl in the
// middle of the strip and crawling both ways.
#define ENABLE_SPLIT_STRIP 1
// Center LED, aka LED #0.
#define SPLIT_STRIP_CENTER 6

/* Sleeping parameters: */
#define SLEEP_BRIGHTNESS 0.30
#define SLEEP_CYCLE_MS 5000 // 5 second breathing cycle. Default: 5000
#define SLEEP_WAIT_TIME_MS 4000 // No movement for 5 seconds triggers breathing. Default: 5000
#define SLEEP_SENSITIVITY 100

/* Debug parameters: */
#define WAIT_FOR_KEYBOARD 0 // Use keyboard to pause/resume program.
#define PRINT_LOOP_TIME 0
#define PRINT_ACCEL_DATA 1
#define PRINT_SLEEP_TIME 0
#define PRINT_SLEEP_SENS 0

/* Advanced: */
#define ONBOARD_LED_PIN 7 // Pin D7 has an LED connected on FLORA.
#define ONBOARD_LED_NEOPIX 8 // Pin D8 has an LED connected on FLORA.

/* Button parameters: */
#define BUTTON_PIN 9
#define BUTTON_A_PIN 9
#define BUTTON_B_PIN 10

Adafruit_LSM303_Old lsm; // Bridge to accelerometer hardware.
AccelReading accelBuffer[10]; // Buffer for storing the last 10 readings.
int bufferPosition; // Current read position of the buffer.
double calibration; // Baseline for accelerometer data.

// For breathing, track the time of the last significant movement.
unsigned long lastSignificantMovementTime;

CRGB onboard[1];
CRGB leds[NUM_LEDS];

// Whether to show the blinking lead
uint8_t gHue = 0;
uint8_t chase = 0;

void setup() {
  // Initialize the strips
  FastLED.addLeds<NEOPIXEL, ONBOARD_LED_NEOPIX>(onboard, 1);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // Initialize the button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUTTON_PIN, HIGH);

  // Let the user know we are ready to start
  onboard[0] = CRGB::Blue;
  FastLED.show();

  // Initialize the accelerometer
  accelSetup();
  accelCalibrate();

  // Turn off the calibration pixel
  onboard[0] = CRGB::Black;
  FastLED.show();
}

/* Number of total animations: */
#define A_ANIMATIONS 4
#define B_ANIMATIONS 2

int fadeout = 0;
int fadein = 0;
int a_animation = 1;
int b_animation = 1;
//uint8_t cycle = 0;
unsigned long now;

void loop() {
  EVERY_N_MILLISECONDS( 20 ) {
    gHue++;  // slowly cycle the "base color" through the rainbow
  }
  EVERY_N_MILLISECONDS( 100 ) {
    chase++;
    if (chase == 3) {
      chase = 0;
    }
  }

  accelPoll();
  updateLEDs();
  FastLED.show();
  buttons();
}

void updateLEDs() {
  // Largest vector needed to hit max color (1.0).
  double upperBound = HERMES_SENSITIVITY;
  double magnitude = getMagnitude(getCurrentReading());

  double normalizedVector = abs(calibration - magnitude);
  double scale = normalizedVector / upperBound;
  CRGB pixelColor = pixelColorForScale(scale);

  if (!sleep(magnitude)) {
    switch (a_animation) {
      case 1:
        crawlColor(pixelColor);
        break;
      case 2:
        theaterChase(leds, NUM_LEDS, true);
        break;
      case 3:
        theaterChase(leds, NUM_LEDS, false);
        break;
      case 4:
        fill_rainbow(leds, NUM_LEDS, gHue, 5);
        break;
      case 100:
        fadeToBlackBy(leds, NUM_LEDS, 1);
        break;
    }
  } else {
    switch (b_animation) {
      case 1:
        breathe();
        break;
      case 2:
        fadeOut(0, 0, 0, 20);
        break;
      case 100:
        fadeToBlackBy(leds, NUM_LEDS, 1);
        break;
      default:
        breathe();
        break;
    }
  }
}

///////////
// accel //
///////////


// Initialization.

void accelSetup() {
  lsm.begin();
  bufferPosition = 0;

  // Initialize the full buffer to zero.
  for (int i = 0; i < bufferSize(); i++) {
    accelBuffer[i].x = 0;
    accelBuffer[i].y = 0;
    accelBuffer[i].z = 0;
  }
}

void accelCalibrate() {
  calibration = 0;

  unsigned long now = millis();
  unsigned long last_change = now;
  int blinker_time = 250;
  bool onboard_blink = true;

  while (1) {
    // TODO(): Add another animation than a blinking led for calibration
    now = millis();
    if (now - last_change > blinker_time) {
      onboard_blink = !onboard_blink;
      last_change = millis();
      if (onboard_blink) {
        onboard[0] = CRGB::Red;
      } else {
        onboard[0] = CRGB::Black;
      }
    }
    FastLED.show();

    // Fill the buffer.
    if (!fillBuffer()) {
      delay(10);
      continue;
    }

    // Check to see if we're done.
    bool pass = true;
    double avg = 0;
    for (int i = 0; i < bufferSize(); i++) {
      double m = getMagnitude(accelBuffer[i]);
      pass = pass && (abs(m - calibration) < 10);
      avg += m;
    }

    if (pass) {
      break;
    } else {
      avg /= bufferSize();
      calibration = avg;
    }
  }
  // Clear the onboard pixel, in case it was left on
  onboard[0] = CRGB::Black;
  FastLED.show();
}

void accelPoll() {
  // Read new accelerometer data. If there is no new data, return immediately.
  if (!fillBuffer()) {
    return;
  }
}

// Gets the vector for the given reading.
double getVector(AccelReading reading) {
  double normalizedVector = abs(calibration - getMagnitude(reading));
  return normalizedVector;
}

///////////////////////////////////////////////////////////////////
// This may or may not fill the next buffer position. If the accelerometer hasn't
// processed a new reading since the last buffer, this function immediately exits,
// returning false.
// Otherwise, if the accelerometer has read new data, this function advances the
// buffer position, fills the buffer with accelerometer data, and returns true.
bool fillBuffer() {
  // Read from the hardware.
  lsm.read();

  AccelReading newReading;
  newReading.x = lsm.accelData.x;
  newReading.y = lsm.accelData.y;
  newReading.z = lsm.accelData.z;

  // The accelerometer hasn't processed a new reading since the last buffer.
  // Do nothing and return false.
  if (equalReadings(getCurrentReading(), newReading)) {
    return false;
  }

  // The accelerometer has read new data.
  // Advance the buffer.
  if (++bufferPosition >= bufferSize()) {
    bufferPosition = 0;
  }

  AccelReading *mutableCurrentReading = &accelBuffer[bufferPosition];

  mutableCurrentReading->x = newReading.x;
  mutableCurrentReading->y = newReading.y;
  mutableCurrentReading->z = newReading.z;

  return true;
}

///////////////////////////////////////////////////////////////////
// Gets the average difference between the latest buffer and previous buffer.
int getDelta() {
  AccelReading previousReading = getPreviousReading();
  AccelReading currentReading  = getCurrentReading();

  int deltaX = abs(abs(currentReading.x) - abs(previousReading.x));
  int deltaY = abs(abs(currentReading.y) - abs(previousReading.y));
  int deltaZ = abs(abs(currentReading.z) - abs(previousReading.z));

  return (deltaX + deltaY + deltaZ) / 3;
}

// Gets the vector magnitude for the given reading.
// http://en.wikipedia.org/wiki/Euclidean_vector#Length
double getMagnitude(AccelReading reading) {
  double x = reading.x;
  double y = reading.y;
  double z = reading.z;

  double vector = x * x + y * y + z * z;

  return sqrt(vector);
}

// Returns the number of items held by the buffer.
int bufferSize() {
  return sizeof(accelBuffer) / sizeof(accelBuffer[0]);
}

AccelReading getCurrentReading() {
  return accelBuffer[bufferPosition];
}

// Gets the previous buffer reading.
AccelReading getPreviousReading() {
  int previous = bufferPosition - 1;
  if (previous < 0) previous = bufferSize() - 1;
  return accelBuffer[previous];
}

// Returns true if two readings are equal.
bool equalReadings(AccelReading a, AccelReading b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}



/////////////
// buttons //
/////////////

void buttons() {
  int b = checkButton();
  if (b == 1) {
    a_animation++;
    if (a_animation > A_ANIMATIONS)
      a_animation = 1;
    blinky(a_animation, 0);
  }

  if (b == 2) {
    b_animation++;
    if (b_animation > B_ANIMATIONS)
      b_animation = 1;
    blinky(b_animation, 160);
  }
  if (b == 3 || b == 4) {
    a_animation = 100;
    b_animation = 100;
  }
}


/*
  MULTI-CLICK: One Button, Multiple Events

  Oct 12, 2009
  Run checkButton() to retrieve a button event:
  Click
  Double-Click
  Hold
  Long Hold
*/

// Button timing variables
int debounce = 20; // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 250; // max ms between clicks for a double click event
int holdTime = 2000; // ms hold period: how long to wait for press+hold event
int longHoldTime = 5000; // ms long hold period: how long to wait for press+hold event

// Other button variables
boolean buttonVal = HIGH; // value read from button
boolean buttonLast = HIGH; // buffered value of the button's previous state
boolean DCwaiting = false; // whether we're waiting for a double click (down)
boolean DConUp = false; // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true; // whether it's OK to do a single click
long downTime = -1; // time the button was pressed down
long upTime = -1; // time the button was released
boolean ignoreUp = false; // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false; // when held, whether to wait for the up event
boolean holdEventPast = false; // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already

int checkButton()
{
  int event = 0;
  // Read the state of the button
  buttonVal = digitalRead(BUTTON_PIN);
  // Button pressed down
  if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce) {
    downTime = millis();
    ignoreUp = false;
    waitForUp = false;
    singleOK = true;
    holdEventPast = false;
    longHoldEventPast = false;
    if ((millis() - upTime) < DCgap && DConUp == false && DCwaiting == true) DConUp = true;
    else DConUp = false;
    DCwaiting = false;
  }
  // Button released
  else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce) {
    if (not ignoreUp) {
      upTime = millis();
      if (DConUp == false) DCwaiting = true;
      else {
        event = 2;
        DConUp = false;
        DCwaiting = false;
        singleOK = false;
      }
    }
  }
  // Test for normal click event: DCgap expired
  if ( buttonVal == HIGH && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true) {
    event = 1;
    DCwaiting = false;
  }
  // Test for hold
  if (buttonVal == LOW && (millis() - downTime) >= holdTime) {
    // Trigger "normal" hold
    if (not holdEventPast) {
      event = 3;
      waitForUp = true;
      ignoreUp = true;
      DConUp = false;
      DCwaiting = false;
      //downTime = millis();
      holdEventPast = true;
    }
    // Trigger "long" hold
    if ((millis() - downTime) >= longHoldTime) {
      if (not longHoldEventPast) {
        event = 4;
        longHoldEventPast = true;
      }
    }
  }
  buttonLast = buttonVal;
  return event;
}



/////////////////////
// Color functions //
/////////////////////

CRGB lightArray[NUM_LEDS];

void fadeToColor(CRGB c, uint8_t wait) {
  boolean done = true;
  for (int i = 0; i < NUM_LEDS; i++) {
    done = true;
    if (leds[i] != c) {
      if (leds[i] > c) {
        leds[i] -= 1;
        done = false;
      } else {
        leds[i] += 1;
        done = false;
      }
    }
    delay(wait);
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  // TODO(): c is 32-bit color for Neopixel lib. We need to convert it to 0-255 values.
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(c, 255, MAX_BRIGHTNESS * 255);
    FastLED.show();
    delay(wait);
  }
}

void blinky(uint8_t c, uint8_t gHue) {
  fadeToBlackBy(leds, NUM_LEDS, 255);
  for (int i = 1; i <= c; i++) {
    for (int j = 0; j < i ; j++) {
      leds[j] = CHSV(gHue, 255, 192);
    }
    FastLED.show();
    delay(500);
  }
  fadeToBlackBy(leds, NUM_LEDS, 255);
  FastLED.show();
}

unsigned long lastCrawl;
// "Crawls" the given color along the strip.
// This always sets LED[0] to the given color.
// After CRAWL_SPEED_MS milliseconds,
// we set LED[n + 1] = LED[n] for each LED.
void crawlColor(CRGB color) {
  // Set the head pixel to the new color.
  CRGB head = lightArray[0];

  unsigned long now = millis();

  // Shift the array if it's been long enough since last shifting,
  // or if a new color arrives.
  bool shouldUpdate =
    (now - lastCrawl > CRAWL_SPEED_MS)
    || (color != head);

  if (!shouldUpdate) {
    return;
  }

  lastCrawl = now;

  // Shift the array.
  for (int i = NUM_LEDS - 1; i > 0; --i) {
    lightArray[i] = lightArray[i - 1];
  }
  lightArray[0] = color;

  if (ENABLE_SPLIT_STRIP) {
    int centerLED = SPLIT_STRIP_CENTER;
    int LEDsPerSide = floor(NUM_LEDS / 2);

    // Crawl 'low' side (center down)
    CRGB *pixelColor = lightArray;
    for (int led = centerLED - 1; led >= centerLED - 1 - LEDsPerSide; led--) {
      leds[constrainBetween(led, 0, NUM_LEDS - 1)] = *pixelColor++;
    }

    // Crawl 'high' side (center up)
    pixelColor = lightArray;
    for (int led = centerLED; led < centerLED + LEDsPerSide; led++) {
      leds[constrainBetween(led, 0, NUM_LEDS - 1)] = *pixelColor++;
    }
    //    FastLED.show();
    return;
  } else {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = lightArray[i];
    }
  }
}

int constrainBetween(int value, int lower, int higher) {
  if (value < lower) {
    value = higher - (lower - value) + 1;
  } else if (value > higher) {
    value = lower + (value - higher) - 1;
  }
  return value;
}

void theaterChase(CRGB* leds, uint8_t num_leds, bool rainbow) {
  for (int i = 0; i < num_leds; i = i + 3) {
    if (i + chase < num_leds) {
      if (rainbow == true) {
        leds[i + chase] = CHSV(gHue + i, 255, 192);
      } else {
        leds[i + chase] = CRGB::White;
      }
    }
    if (i + chase - 1 >= 0 && i + chase - 1 < num_leds ) {
      leds[i + chase - 1] = CRGB::Black;
    }
    if (i + chase - 2 >= 0 && i + chase - 2 < num_leds ) {
      leds[i + chase - 2] = CRGB::Black;
    }
  }
}

int COLOR_RANGE = 256;
// Returns a pixel color for use by strip.setPixelColor().
// Automatically adjusts brightness.
// Takes a scale, from 0.0 to 1.0, indicating progression
// through the color rainbow.
CRGB pixelColorForScale(double scale) {
  float brightness = MAX_BRIGHTNESS * (scale + MIN_BRIGHTNESS);
  int c = COLOR_RANGE * scale; // Intentionally round to an int.
  return color(c, brightness);
}

// Color 1 from 256; brightness 0.0 to 1.0.
CRGB color(uint16_t color, float brightness)  {
  byte r, g, b;
  int range = color / 85;
  switch (range) {
    case 0: // Red to Yellow (1 to 85)
      r = 127 - color % 85;
      g = color % 85;
      b = 0;
      break;
    case 1: // Yellow to Teal (129 to 256)
      r = 0;
      g = 84 - color % 85;
      b = color % 85;
      break;
    case 2: // Teal to Purple (257 to 384)
      r = color % 85;
      g = 0;
      b = 84 - color % 85;
      break;
  }
  r *= brightness;
  g *= brightness;
  b *= brightness;
  return CRGB(r, g, b);
}



///////////
// sleep //
///////////

bool sleeping = false;
bool waiting = false;

bool sleep(double m) {
  int repeats = 0;
  unsigned long now = millis();

  if (abs(calibration - m) > SLEEP_SENSITIVITY) {
    lastSignificantMovementTime = now;
    waiting = false;
    sleeping = false;
    digitalWrite(ONBOARD_LED_PIN, sleeping ? HIGH : LOW);
    return false;
  } else {
    // Last significant movement time needs to be longer than sleep wait time.
    if ((now - lastSignificantMovementTime) < SLEEP_WAIT_TIME_MS) {
      // Haven't waited long enough.
      sleeping = false;
      digitalWrite(ONBOARD_LED_PIN, sleeping ? HIGH : LOW);
      return false;
    } else {
      sleeping = true;
      digitalWrite(ONBOARD_LED_PIN, sleeping ? HIGH : LOW);
      return true;
    }
    waiting = true;
  }
}

////////////
// wakeup //
////////////

bool wakeup() {
  accelPoll();
  double m = getMagnitude(getCurrentReading());

  if (abs(calibration - m) > SLEEP_SENSITIVITY) {
    return true;
  }
  return false;
}


// Changes the colors of the strip, from the current value to the given value.
void fadeOut(int red, int green, int blue, int wait) {
  bool timeToGo = false;
  while (!timeToGo) {
    //    CRGB lightArray = leds;
    timeToGo = true;
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t *p,
              r = leds[i].r,
              g = leds[i].g,
              b = leds[i].b;
      if (r > red) {
        r -= 1;
        timeToGo = false;
      } else if (r < red) {
        r += 1;
        timeToGo = false;
      }
      if (g > green) {
        g -= 1;
        timeToGo = false;
      } else if (g < green) {
        g += 1;
        timeToGo = false;
      }
      if (b > blue) {
        b -= 1;
        timeToGo = false;
      } else if (b < blue) {
        b += 1;
        timeToGo = false;
      }
      //      if (timeToGo) {
      //        return;
      //      }
      leds[i] = CRGB(r, g, b);
    }
    FastLED.show();
    delay(wait);
    buttons();
    if (wakeup()) {
      return;
    }
  }
}

void breathe() {
  fadeOut(3, 0, 0, 20);
  for (int i = 0; i < 100; i++) {
    if (wakeup()) {
      return;
    }
    delay(10);
    buttons();
  }
  fadeOut(24, 0, 0, 20);
}
