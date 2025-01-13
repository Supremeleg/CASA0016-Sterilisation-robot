#include "esp_camera.h"
#include <AsyncUDP.h>
#include <WiFi.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

volatile int IO12;//in1
volatile int IO13;//in2
volatile int IO15;//in3
volatile int IO14;//in4
volatile int PWM1;
volatile int PWM2;
String udp001_rx;
AsyncUDP udp001;

void left() {
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
  pinMode(IO12, OUTPUT);
  digitalWrite(IO12,LOW);
  pinMode(IO13, OUTPUT);
  digitalWrite(IO13,HIGH);
  pinMode(IO15, OUTPUT);
  digitalWrite(IO15,HIGH);
  pinMode(IO14, OUTPUT);
  digitalWrite(IO14,LOW);
}

void front() {
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
  pinMode(IO12, OUTPUT);
  digitalWrite(IO12,HIGH);
  pinMode(IO13, OUTPUT);
  digitalWrite(IO13,LOW);
  pinMode(IO15, OUTPUT);
  digitalWrite(IO15,HIGH);
  pinMode(IO14, OUTPUT);
  digitalWrite(IO14,LOW);
}

void right() {
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
  pinMode(IO12, OUTPUT);
  digitalWrite(IO12,HIGH);
  pinMode(IO13, OUTPUT);
  digitalWrite(IO13,LOW);
  pinMode(IO15, OUTPUT);
  digitalWrite(IO15,LOW);
  pinMode(IO14, OUTPUT);
  digitalWrite(IO14,HIGH);
}

void back() {
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 255);
  pinMode(IO12, OUTPUT);
  digitalWrite(IO12,LOW);
  pinMode(IO13, OUTPUT);
  digitalWrite(IO13,HIGH);
  pinMode(IO15, OUTPUT);
  digitalWrite(IO15,LOW);
  pinMode(IO14, OUTPUT);
  digitalWrite(IO14,HIGH);
}

void fRight() {
  analogWrite(PWM1, 120);
  analogWrite(PWM2, 255);
  pinMode(IO12, OUTPUT);
  digitalWrite(IO12,HIGH);
  pinMode(IO13, OUTPUT);
  digitalWrite(IO13,LOW);
  pinMode(IO15, OUTPUT);
  digitalWrite(IO15,HIGH);
  pinMode(IO14, OUTPUT);
  digitalWrite(IO14,LOW);
}

void stop() {
  pinMode(IO12, OUTPUT);
  digitalWrite(IO12,LOW);
  pinMode(IO13, OUTPUT);
  digitalWrite(IO13,LOW);
  pinMode(IO15, OUTPUT);
  digitalWrite(IO15,LOW);
  pinMode(IO14, OUTPUT);
  digitalWrite(IO14,LOW);
  delay(30);
}

void fLeft() {
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 120);
  pinMode(IO12, OUTPUT);
  digitalWrite(IO12,HIGH);
  pinMode(IO13, OUTPUT);
  digitalWrite(IO13,LOW);
  pinMode(IO15, OUTPUT);
  digitalWrite(IO15,HIGH);
  pinMode(IO14, OUTPUT);
  digitalWrite(IO14,LOW);
}

void bLeft() {
  analogWrite(PWM1, 120);
  analogWrite(PWM2, 255);
  pinMode(IO12, OUTPUT);
  digitalWrite(IO12,LOW);
  pinMode(IO13, OUTPUT);
  digitalWrite(IO13,HIGH);
  pinMode(IO15, OUTPUT);
  digitalWrite(IO15,LOW);
  pinMode(IO14, OUTPUT);
  digitalWrite(IO14,HIGH);
}

void bRight() {
  analogWrite(PWM1, 255);
  analogWrite(PWM2, 120);
  pinMode(IO12, OUTPUT);
  digitalWrite(IO12,LOW);
  pinMode(IO13, OUTPUT);
  digitalWrite(IO13,HIGH);
  pinMode(IO15, OUTPUT);
  digitalWrite(IO15,LOW);
  pinMode(IO14, OUTPUT);
  digitalWrite(IO14,HIGH);
}

const char* ssid = "sterilizer";
const char* password = "12345678";

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  IO12 = 12;
  IO13 = 13;
  IO15 = 15;
  IO14 = 14;
  PWM1 = 1;
  PWM2 = 2;
  udp001_rx = "";
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

s->set_vflip(s,1);
s->set_hmirror(s,1);
//s->set_framesize(s, FRAMESIZE_QVGA);

  WiFi.begin("AD2B Hyperoptic Fibre Broadband", "Vtcnn5Uxt8GL");
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
   delay(10);

  udp001.listen(2333);
  udp001.onPacket([](AsyncUDPPacket packet){
    String udp001_rx(packet.data(),packet.length());

  if (udp001_rx == "f") {
    front();
  } else if (udp001_rx == "b") {
    back();
  } else if (udp001_rx == "l") {
    left();
  } else if (udp001_rx == "r") {
    right();
  } else if (udp001_rx == "s") {
    stop();
  } else if (udp001_rx == "fl") {
    fLeft();
  } else if (udp001_rx == "fr") {
    fRight();
  } else if (udp001_rx == "bl") {
    bLeft();
  } else if (udp001_rx == "br") {
    bRight();
  } else if (udp001_rx == "1") {
    pinMode(4, OUTPUT);
    digitalWrite(4,HIGH);
  }else if (udp001_rx == "0") {
    pinMode(4, OUTPUT);
    digitalWrite(4,LOW);
  }

  });
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
  delay(10000);
}
