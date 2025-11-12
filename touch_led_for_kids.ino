#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include "WiFi.h"
#include "driver/adc.h"
#include "esp_sleep.h"
#include "driver/gpio.h"


// PINs
#define TOUCH_SENSOR_PIN        2
#define LED_RING_PIN            5
#define INTERNAL_LED_PIN        8


#define LED_RING_NUM_PIXELS     8
#define LED_DEFAULT_BRIGHTNESS  10
#define SERIAL_BAUD_RATE        115200

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_RING_NUM_PIXELS, LED_RING_PIN, NEO_GRB + NEO_KHZ800);
int LED_DEFAULT_COLOR = pixels.Color(255, 108, 0);



Preferences preferences;
unsigned long prefLastWrite = millis();
int prefBrightness = LED_DEFAULT_BRIGHTNESS;


int brightness = LED_DEFAULT_BRIGHTNESS;
int brightnessModifier = 1;



// the setup function runs once when you press reset or power the board
void setup() {
  Serial.println("=== setup start");
  loadPreferences();
  initSerialBaudRate();
  turnOffInternalLed();
  disableWifi();
  setLedDefaults();
  // enableGpioWakeUp();
  Serial.println("=== setup end");
}

void loadPreferences() {
  preferences.begin("settings", false); // false = read/write
  prefBrightness = preferences.getInt("brightness", LED_DEFAULT_BRIGHTNESS);
  brightness = prefBrightness;
  Serial.printf("prefBrightness is %d\n", prefBrightness);
  preferences.end();
}

void writePreferences() {
  // write at the earliest 20 secs after last write
  if(millis() - prefLastWrite < 1000*20) {
    return;
  }
  preferences.begin("settings", false); // false = read/write
  prefBrightness = preferences.getInt("brightness", LED_DEFAULT_BRIGHTNESS);

  if(prefBrightness != brightness) {
    preferences.putInt("brightness", brightness);
    prefLastWrite = millis();
    blinkLeds();
    Serial.println("preferences saved");
  }
  preferences.end();
}


void initSerialBaudRate() {
  Serial.printf("setting serial to %d\n", SERIAL_BAUD_RATE);
  Serial.begin(SERIAL_BAUD_RATE);

}
void turnOffInternalLed() {
  Serial.println("turning off internal led...");
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  digitalWrite(INTERNAL_LED_PIN, HIGH);  // turn the LED on (HIGH is the voltage level)
}

void setWifiSleep() {
  Serial.println("setting wifi sleep to save battery...");
  WiFi.setSleep(true);
  // Serial.printf("WiFi-Sleep: %d \n",WiFi.getSleep());
}

void disableWifi(){
    // adc_power_off();
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
}

void updateLedStatus(uint8_t brightness, uint32_t color/*, uint32_t delayms*/) {
  for(int i=0;i<LED_RING_NUM_PIXELS;i++){
    // pixels.setPixelBrightness()
    pixels.setBrightness(brightness);
    pixels.setPixelColor(i, color); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    /*
    if(delayms > 0) {
      delay(delayms); // Delay for a period of time (in milliseconds).
    }
    */
  }
}
void blinkLeds() {
  updateLedStatus(0, LED_DEFAULT_COLOR);
  delay(100);
  updateLedStatus(100, LED_DEFAULT_COLOR);
  delay(100);
  updateLedStatus(0, LED_DEFAULT_COLOR);
  delay(100);
  updateLedStatus(brightness, LED_DEFAULT_COLOR);
}


void setLedDefaults() {
  Serial.printf("setting default LED status - brightness: %d, color: %#08x\n", LED_DEFAULT_BRIGHTNESS, LED_DEFAULT_COLOR);
  updateLedStatus(brightness, LED_DEFAULT_COLOR);
}

void setBrightnessValue(int newBrightness) {
  if(newBrightness < 0) {
    newBrightness = 0;
  }
  if(newBrightness > 255) {
    newBrightness = 255;
  }

  if(newBrightness >= 0 && newBrightness <= 255) {
    brightness = newBrightness;
    updateLedStatus(brightness, LED_DEFAULT_COLOR);
  }
}

void changeBrightness(int modifier) {
  Serial.printf("changeBrightness: %d\n", modifier);
  int newBrightness = brightness += modifier;
  setBrightnessValue(newBrightness);
}

void toggleBrightness() {
  if(brightness <= 0) {
    brightnessModifier = 1;
  } else if(brightness >= 255) {
    brightnessModifier = -1;
  }
  changeBrightness(brightnessModifier);
}




/*
const gpio_num_t wakeUpPin = GPIO_NUM_2;

int wakeup_gpio;

void IRAM_ATTR handleInterrupt1() {
    wakeup_gpio = TOUCH_SENSOR_PIN;
}

void enableGpioWakeUp() {
  gpio_wakeup_enable(TOUCH_SENSOR_PIN, GPIO_INTR_HIGH_LEVEL); // Trigger wake-up on high level
  attachInterrupt(digitalPinToInterrupt(TOUCH_SENSOR_PIN), handleInterrupt1, RISING);
}
*/

// ISR for buttonPin1


int currentTouchState = LOW;
int lastTouchState = LOW;
unsigned long currentTouchStateDurationMs = 0;
bool touchActive = false;
unsigned long lastTouchStateChangeMs = millis();
int touchCount = 0;
bool touchSkipNextRelease = false;

void loop() {  
  currentTouchState = digitalRead(TOUCH_SENSOR_PIN);
  touchActive = currentTouchState == HIGH;

  if(currentTouchState != lastTouchState) {
    lastTouchState = currentTouchState;
    lastTouchStateChangeMs = millis();
    if(!touchSkipNextRelease && !touchActive) {
      touchCount++;
    }
    touchSkipNextRelease = false;
  }

  currentTouchStateDurationMs = millis() - lastTouchStateChangeMs;

  
  if(currentTouchStateDurationMs > 250 && (touchCount > 0 || touchActive)) {
    Serial.printf("touch action: touchCount: %d, touchActive: %d, touchSkipNextRelease: %d\n", touchCount, touchActive, touchSkipNextRelease);

    if(touchActive) {
        if(currentTouchStateDurationMs > 10000) {
          Serial.printf("You pressed the touch button for 10 seconds.\n");
        }

        switch(touchCount) {
          case 2:
            changeBrightness(-1);
            break;
          case 1:
            changeBrightness(1);
            break;
          default:
            toggleBrightness();
            break;
        }
    } else {
        switch(touchCount) {
          case 5:
            writePreferences();
            break;
          case 4:
            setBrightnessValue(120);
            break;
          case 3:
            setBrightnessValue(10);
            break;
          case 2:
            changeBrightness(5);
            break;
          default:
            changeBrightness(-5);
            break;            
        }
    }



    // reset state after processed touch event
    if(touchActive) {
      // next touch-release has to be skipped after longpress
      touchSkipNextRelease = true;
    } else {
      touchCount = 0;
    }
  }
  

  /*
  currentTouchState = digitalRead(TOUCH_SENSOR_PIN);
  if(currentTouchState == HIGH) {
    if(brightness >= 100) {
      brightnessModifier = -1;
    } else if(brightness <= 0) {
      brightnessModifier = 1;
    }
    brightness += brightnessModifier;
    updateLedStatus(brightness, color, 0);
    delay(10);
  }


  if(currentTouchState != lastTouchState) {
    lastTouchStateDuration = millis() - lastTouchStateChangeMs;
    lastTouchStateChangeMs = millis();
    if(currentTouchState == HIGH) {
      touchPressed();
    } else {
      touchReleased();
    }
    lastTouchState = currentTouchState;
  }
  */
  // Serial.printf("state: %d\n", state);

  // Serial.println("loop end");
}
