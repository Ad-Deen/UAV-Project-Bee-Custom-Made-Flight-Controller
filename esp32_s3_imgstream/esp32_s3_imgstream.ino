#include "esp_camera.h"
#include "ESP32_OV5640_AF.h"
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Hive";
const char* password = "12345678";
const char* receiverIP = "192.168.4.2";  // Your PC IP here
const uint16_t receiverPort = 12345;   //camera1 port
// const uint16_t receiverPort = 12346;  //camera2 port

// const char* ssid = "Explore";
// const char* password = "Explore.us";
// const char* receiverIP = "192.168.0.113";  // Your PC IP here
// const uint16_t receiverPort = 12345;  //camera1 port
// const uint16_t receiverPort = 12346;  //camera2 port

WiFiUDP udp;
const unsigned int localPort = 54321;
static uint16_t frame_id = 0;
const int MAX_UDP_PACKET_SIZE = 1000;  // safe chunk size
uint8_t packet[6 + MAX_UDP_PACKET_SIZE];  // header + payload

// ESP32-S3 WROOM + OV5640 Camera Pin Map
#define PWDN_GPIO_NUM    38
#define RESET_GPIO_NUM   -1  // software reset

#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5

#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM      11

#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM    13

OV5640 ov5640 = OV5640();

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  // Serial.print("Connected! IP address: ");
  // Serial.println(WiFi.localIP());

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
config.pin_sccb_sda = SIOD_GPIO_NUM;   // ✅ corrected field name
config.pin_sccb_scl = SIOC_GPIO_NUM;   // ✅ corrected field name

config.pin_pwdn = PWDN_GPIO_NUM;
config.pin_reset = RESET_GPIO_NUM;

config.xclk_freq_hz = 20000000;
config.pixel_format = PIXFORMAT_JPEG;
config.frame_size = FRAMESIZE_VGA;
config.jpeg_quality = 5;
config.fb_count = 2;
config.fb_location = CAMERA_FB_IN_PSRAM;
config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  ov5640.start(sensor);

  if (ov5640.focusInit() == 0) {
    Serial.println("OV5640_Focus_Init Successful!");
  }

  if (ov5640.autoFocusMode() == 0) {
    Serial.println("OV5640_Auto_Focus Successful!");
  }
  udp.begin(localPort);  // Local port
}

void loop() {
  static uint16_t frame_id = 0;
  // static uint32_t last_fps_time = 0;
  // static uint16_t frame_count = 0;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb || fb->format != PIXFORMAT_JPEG) {
    if (fb) esp_camera_fb_return(fb);
    return;
  }

  int total_len = fb->len;
  int chunks = (total_len + MAX_UDP_PACKET_SIZE - 1) / MAX_UDP_PACKET_SIZE;

  for (int i = 0; i < chunks; i++) {
    int chunk_size = (i == chunks - 1) ? (total_len - i * MAX_UDP_PACKET_SIZE) : MAX_UDP_PACKET_SIZE;
    packet[0] = (frame_id >> 8) & 0xFF;
    packet[1] = frame_id & 0xFF;
    packet[2] = (i >> 8) & 0xFF;
    packet[3] = i & 0xFF;
    packet[4] = (chunk_size >> 8) & 0xFF;
    packet[5] = chunk_size & 0xFF;
    memcpy(packet + 6, fb->buf + i * MAX_UDP_PACKET_SIZE, chunk_size);

    udp.beginPacket(receiverIP, receiverPort);
    udp.write(packet, chunk_size + 6);
    udp.endPacket();
    // delayMicroseconds(100);  // Short delay to avoid overrunning PC receiver
  }

  esp_camera_fb_return(fb);
  frame_id++;
  // frame_count++;

  // Calculate FPS every second
  // uint32_t now = millis();
  // if (now - last_fps_time >= 1000) {
  //   Serial.printf("FPS: %d\n", frame_count);
  //   frame_count = 0;
  //   last_fps_time = now;
  // }
  // Serial.println("sent");
}
