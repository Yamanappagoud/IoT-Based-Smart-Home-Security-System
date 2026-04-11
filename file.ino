#include "WiFi.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <HTTPClient.h>
#include <DFRobotDFPlayerMini.h>  // Add DFPlayer Mini library

// Pin definitions for ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Pin definitions for external components
#define PIR_PIN           GPIO_NUM_13  // PIR sensor input pin
#define BUZZER_PIN        GPIO_NUM_2   // Buzzer output pin
#define GREEN_LED_PIN     GPIO_NUM_14  // Green LED pin
#define RED_LED_PIN       GPIO_NUM_15  // Red LED pin
#define MP3_TX            GPIO_NUM_12  // Connect to RX pin of DFPlayer Mini
#define MP3_RX            GPIO_NUM_4   // Connect to TX pin of DFPlayer Mini
#define FLASH_LED_PIN     4           // Built-in flash LED on ESP32-CAM

// DFPlayer Mini configuration
DFRobotDFPlayerMini dfPlayer;
#define HUMAN_DETECTED_SOUND   1       // 1.mp3 file number for human detection
#define OTHER_DETECTED_SOUND   2       // 2.mp3 file number for other detections

// WiFi credentials
const char* ssid = "Kuteera A405";
const char* password = "9900857576";

// API endpoint - Change localhost to actual IP address of your server
const char* apiEndpoint = "http://192.168.0.104:5000/detect";  // Replace with your server's IP

// Add debug flag for verbose output
#define DEBUG_MODE true

// Motion detection variables
bool motionDetected = false;
bool previousMotionState = false;
unsigned long lastPhotoTime = 0;
const unsigned long MIN_PHOTO_INTERVAL = 5000; // Minimum 5 seconds between photos

// New timing variables
unsigned long lastPIRCheckTime = 0;
unsigned long PIR_CHECK_INTERVAL = 50; // Check PIR sensor every 50ms (more responsive)
unsigned long lastLEDUpdateTime = 0;
unsigned long LED_UPDATE_INTERVAL = 100; // Update LEDs every 100ms

// Camera error tracking
int consecutiveCameraErrors = 0;
const int MAX_CAMERA_ERRORS = 3;

// Function declarations
void takePhotoAndSend();
bool initCamera();
void testCamera();
void blinkLED(int pin, int times);
String getHostFromURL(const char* url);
String getPathFromURL(const char* url);
int getPortFromURL(const char* url);

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\nStarting ESP32-CAM Motion Detection System");
  
  // Initialize pins
  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(FLASH_LED_PIN, OUTPUT);  // Initialize flash LED pin
  
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, HIGH); // Red LED ON initially (no motion state)
  digitalWrite(FLASH_LED_PIN, LOW); // Ensure flash is off at startup

  // Signal startup with both LEDs
  blinkLED(GREEN_LED_PIN, 2);
  blinkLED(RED_LED_PIN, 2);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Connected to WiFi network with IP: ");
  Serial.println(WiFi.localIP());
  blinkLED(GREEN_LED_PIN, 3); // Signal WiFi connected
  
  // Initialize and test camera
  if (!initCamera()) {
    Serial.println("Camera init failed! Will restart in 5 seconds");
    blinkLED(RED_LED_PIN, 10); // Fast error blink
    delay(5000);
    ESP.restart();
  }
  
  // Test the camera to make sure it can capture images
  testCamera();
  
  Serial.println("System ready!");
  blinkLED(GREEN_LED_PIN, 3); // Signal system ready
  
  // Initialize DFPlayer Mini
  Serial1.begin(9600, SERIAL_8N1, MP3_RX, MP3_TX);
  if (!dfPlayer.begin(Serial1)) {
    Serial.println("Unable to begin DFPlayer Mini");
    Serial.println("Please check connections and SD card");
    blinkLED(RED_LED_PIN, 5); // Error indicator
  } else {
    Serial.println("DFPlayer Mini online");
    dfPlayer.volume(20); // Set volume (0-30) - fixed method name
  }
}

bool initCamera() {
  Serial.println("Initializing camera...");
  
  // Give the camera some time to start up
  delay(100);
  
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Try lower resolution first to ensure stability
  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA; // Lower than UXGA for reliability
    config.jpeg_quality = 12;           // Higher quality = more stable
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;  // Very small frame size for no PSRAM
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  
  // Set to more reliable settings
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAMESIZE_VGA);  // 640x480
    s->set_quality(s, 10);               // 10-63, lower is higher quality
    s->set_brightness(s, 2);             // -2,2 (increased for better visibility)
    s->set_contrast(s, 1);               // -2,2 (increased for better detection)
    s->set_saturation(s, 1);             // -2,2 (increased for better color)
    s->set_special_effect(s, 0);         // 0=none, 1=negative, 2=grayscale
    s->set_whitebal(s, 1);               // 0=disable, 1=enable
    s->set_awb_gain(s, 1);               // 0=disable, 1=enable
    s->set_wb_mode(s, 0);                // white balance mode
    s->set_exposure_ctrl(s, 1);          // 0=disable, 1=enable
    s->set_aec2(s, 1);                   // auto exposure
    s->set_gain_ctrl(s, 1);              // 0=disable, 1=enable
    s->set_agc_gain(s, 0);               // 0-30
    s->set_gainceiling(s, (gainceiling_t)0); // 0-6
    s->set_bpc(s, 0);                    // black pixel correction
    s->set_wpc(s, 1);                    // white pixel correction
    s->set_raw_gma(s, 1);                // gamma
    s->set_lenc(s, 1);                   // lens correction
    s->set_hmirror(s, 0);                // horizontal mirror
    s->set_vflip(s, 0);                  // vertical flip
    s->set_colorbar(s, 0);               // colorbar test pattern
  }
  
  Serial.println("Camera configuration complete");
  return true;
}

void testCamera() {
  Serial.println("Testing camera capture...");
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("TEST FAILED: Camera capture test failed");
    blinkLED(RED_LED_PIN, 5);
    
    // Try to fix the camera by power cycling it
    Serial.println("Attempting to reset the camera...");
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, HIGH);   // Power down the camera
    delay(1000);
    digitalWrite(PWDN_GPIO_NUM, LOW);    // Power up the camera
    delay(1000);
    
    // Reinitialize the camera
    if (!initCamera()) {
      Serial.println("Camera reinitialization failed! Restarting system...");
      delay(3000);
      ESP.restart();
    }
    
    // Test again after reset
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera still not working after reset. Will continue anyway.");
    } else {
      Serial.printf("TEST PASSED: Camera reset successful. Capture size: %uKB\n", fb->len / 1024);
      esp_camera_fb_return(fb);
    }
  } else {
    Serial.printf("TEST PASSED: Camera capture successful. Size: %uKB\n", fb->len / 1024);
    esp_camera_fb_return(fb);
  }
}

void blinkLED(int pin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(200);
    digitalWrite(pin, LOW);
    delay(200);
  }
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Read PIR sensor at specified intervals (without blocking)
  if (currentMillis - lastPIRCheckTime >= PIR_CHECK_INTERVAL) {
    // Read PIR sensor
    motionDetected = digitalRead(PIR_PIN);
    lastPIRCheckTime = currentMillis;
    
    // Handle state change immediately when motion is detected
    if (motionDetected && !previousMotionState) {
      Serial.println("Motion state changed: DETECTED");
      digitalWrite(GREEN_LED_PIN, HIGH);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
      
      // Take photo and send when motion is first detected
      Serial.println("Motion detected! Taking photo...");
      takePhotoAndSend();
      lastPhotoTime = currentMillis;
      
    } else if (!motionDetected && previousMotionState) {
      // Motion stopped
      Serial.println("Motion state changed: STOPPED");
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(BUZZER_PIN, LOW); // Deactivate buzzer
    } else if (motionDetected && (currentMillis - lastPhotoTime) > MIN_PHOTO_INTERVAL) {
      // Continued motion - take periodic photos based on interval
      Serial.println("Continued motion - taking periodic photo");
      takePhotoAndSend();
      lastPhotoTime = currentMillis;
    }
    
    // Update previous motion state for edge detection
    previousMotionState = motionDetected;
  }
  
  // If we've had multiple camera errors, try to reinitialize
  if (consecutiveCameraErrors >= MAX_CAMERA_ERRORS) {
    Serial.println("Too many consecutive camera errors. Attempting to reinitialize...");
    initCamera(); // Try to reinitialize the camera
    consecutiveCameraErrors = 0;
  }
  
  // Non-blocking LED blinking for active state indication (optional)
  if (motionDetected && (currentMillis - lastLEDUpdateTime >= LED_UPDATE_INTERVAL)) {
    static bool blinkState = false;
    blinkState = !blinkState;
    digitalWrite(GREEN_LED_PIN, blinkState ? HIGH : LOW);
    lastLEDUpdateTime = currentMillis;
  }
}

void takePhotoAndSend() {
  // Take multiple attempts if needed
  camera_fb_t *fb = NULL;
  int attempts = 0;
  
  while (fb == NULL && attempts < 3) {
    attempts++;
    Serial.printf("Camera capture attempt %d...\n", attempts);
    
    // Turn on flash before capture
    digitalWrite(FLASH_LED_PIN, HIGH);
    Serial.println("Flash turned ON");
    delay(200); // Increased delay to let flash stabilize better
    
    fb = esp_camera_fb_get();
    
    // Turn off flash immediately after capture
    digitalWrite(FLASH_LED_PIN, LOW);
    Serial.println("Flash turned OFF");
    
    if (!fb) {
      Serial.println("Camera capture failed");
      consecutiveCameraErrors++;
      delay(500); // Wait a bit before retrying
    }
  }
  
  if (!fb) {
    Serial.println("All camera capture attempts failed!");
    return;
  }
  
  // Reset error counter on successful capture
  consecutiveCameraErrors = 0;
  
  Serial.printf("Picture taken! Size: %uKB\n", fb->len / 1024);
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.begin(ssid, password);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
      delay(500);
      Serial.print(".");
      retries++;
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Failed to reconnect to WiFi");
      esp_camera_fb_return(fb);
      return;
    }
    Serial.println("WiFi reconnected");
  }
  
  WiFiClient client;
  
  // Create a boundary string
  String boundary = "----------------------------ESP32CAM";
  
  // Standard HTTP headers
  String crlf = "\r\n";
  String twoHyphens = "--";
  
  // Form multipart headers
  String head = twoHyphens + boundary + crlf;
  head += "Content-Disposition: form-data; name=\"image\"; filename=\"esp32cam.jpg\"" + crlf;
  head += "Content-Type: image/jpeg" + crlf + crlf;
  
  // Form closing boundary
  String tail = crlf + twoHyphens + boundary + twoHyphens + crlf;
  
  // Calculate content length
  size_t imageLen = fb->len;
  size_t extraLen = head.length() + tail.length();
  size_t totalLen = extraLen + imageLen;
  
  if (DEBUG_MODE) {
    Serial.printf("Image size: %u bytes (%u KB)\n", imageLen, imageLen/1024);
    Serial.printf("Image dimensions: %d x %d\n", fb->width, fb->height);
    Serial.printf("Total content length: %u bytes\n", totalLen);
    Serial.println("Server endpoint: " + String(apiEndpoint));
    Serial.println("Server host: " + getHostFromURL(apiEndpoint));
    Serial.println("Server port: " + String(getPortFromURL(apiEndpoint)));
  }
  
  Serial.println("Connecting to server...");
  // Fix the connect method by using c_str() to convert String to const char*
  // Also use the getPortFromURL() function instead of hardcoded port
  if (!client.connect(getHostFromURL(apiEndpoint).c_str(), getPortFromURL(apiEndpoint))) {
    Serial.println("Connection to server failed");
    esp_camera_fb_return(fb);
    return;
  }
  
  // Send HTTP POST request header
  client.println("POST " + getPathFromURL(apiEndpoint) + " HTTP/1.1");
  client.println("Host: " + getHostFromURL(apiEndpoint));
  client.println("User-Agent: ESP32-CAM");
  client.println("Connection: close");
  client.println("Content-Type: multipart/form-data; boundary=" + boundary);
  client.println("Content-Length: " + String(totalLen));
  client.println();
  
  // Send form headers
  client.print(head);
  
  // Send the image data directly
  uint8_t *fbBuf = fb->buf;
  size_t fbLen = fb->len;
  for (size_t n = 0; n < fbLen; n += 1024) {
    if (n + 1024 < fbLen) {
      client.write(fbBuf, 1024);
      fbBuf += 1024;
    } else {
      size_t remainder = fbLen - n;
      client.write(fbBuf, remainder);
    }
  }
  
  // Send the closing boundary
  client.print(tail);
  
  // Wait for response
  Serial.println("Request sent. Waiting for response...");
  
  long timeout = millis() + 15000; // Increased timeout to 15 seconds
  bool responseReceived = false;
  
  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      responseReceived = true;
      // Read the response
      String statusLine = client.readStringUntil('\r');
      Serial.println(statusLine);
      
      // Skip HTTP headers
      while (client.connected()) {
        String line = client.readStringUntil('\n');
        if (DEBUG_MODE) {
          Serial.println("Header: " + line);
        }
        if (line == "\r" || line.length() == 0) {
          break;
        }
      }
      
      // Read the actual response content
      String response = client.readString();
      Serial.println("Server response:");
      Serial.println(response);
      
      // Handle empty detections case explicitly
      if (response == "{\"detections\":[]}") {
        Serial.println("No objects detected in the image");
        dfPlayer.play(OTHER_DETECTED_SOUND); // Play generic sound
      } else if (response.indexOf("human") >= 0 || response.indexOf("person") >= 0) {
        Serial.println("Human detected - playing notification sound 1");
        dfPlayer.play(HUMAN_DETECTED_SOUND); // Play 1.mp3
      } else {
        Serial.println("Other object detected - playing notification sound 2");
        dfPlayer.play(OTHER_DETECTED_SOUND); // Play 2.mp3
      }
      
      break;
    }
    delay(10);
  }
  
  if (!responseReceived) {
    Serial.println("No response received within timeout period!");
  }
  
  // Close the connection
  client.stop();
  
  // Return the frame buffer back to the camera
  esp_camera_fb_return(fb);
}

// Extract host from URL (e.g., "192.168.0.104" from "http://192.168.0.104:5000/detect")
String getHostFromURL(const char* url) {
  String urlStr = String(url);
  urlStr.replace("http://", "");
  
  int colonPos = urlStr.indexOf(':');
  if (colonPos > 0) {
    return urlStr.substring(0, colonPos);
  }
  
  int slashPos = urlStr.indexOf('/');
  if (slashPos > 0) {
    return urlStr.substring(0, slashPos);
  }
  
  return urlStr;
}

// Extract path from URL (e.g., "/detect" from "http://192.168.0.104:5000/detect")
String getPathFromURL(const char* url) {
  String urlStr = String(url);
  urlStr.replace("http://", "");
  
  int slashPos = urlStr.indexOf('/');
  if (slashPos >= 0) {
    return urlStr.substring(slashPos);
  }
  
  return "/";
}

// Extract port from URL (e.g., 5000 from "http://192.168.0.104:5000/detect")
int getPortFromURL(const char* url) {
  String urlStr = String(url);
  urlStr.replace("http://", "");
  
  int colonPos = urlStr.indexOf(':');
  if (colonPos < 0) {
    return 80; // Default HTTP port
  }
  
  int slashPos = urlStr.indexOf('/', colonPos);
  if (slashPos < 0) {
    return urlStr.substring(colonPos + 1).toInt();
  }
  
  return urlStr.substring(colonPos + 1, slashPos).toInt();
}
