#include "esp_camera.h"
#include <WiFi.h>

// Select camera model
#define CAMERA_MODEL_AI_THINKER

// Car driving optimizations
#define ENABLE_PSRAM_OPTIMIZATION
#ifndef CAR_DRIVING_MODE
#define CAR_DRIVING_MODE
#endif
#define FRAME_BUFFER_COUNT 3  // Increased for smoother streaming

const char* ssid = "MATTER";
const char* password = "Youmatterhere";

// Performance monitoring
unsigned long lastFrameTime = 0;
unsigned long frameCount = 0;
float avgFPS = 0;

// Function declarations
camera_config_t getCameraConfig();

#if defined(CAMERA_MODEL_WROVER_KIT)
  // Wrover pin definitions
#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  // M5Stack pin definitions
#elif defined(CAMERA_MODEL_AI_THINKER)
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
#else
  #error "Camera model not selected"
#endif

#include <WebServer.h>
WebServer server(80);

// Optimized streaming with chunked transfer
void handle_jpg_stream() {
  WiFiClient client = server.client();
  
  // Set TCP no delay for real-time streaming
  client.setNoDelay(true);
  
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n";
  response += "Cache-Control: no-cache, no-store, must-revalidate\r\n";
  response += "Pragma: no-cache\r\n";
  response += "Expires: 0\r\n";
  response += "Access-Control-Allow-Origin: *\r\n\r\n";
  client.print(response);

  uint32_t frame_num = 0;
  unsigned long stream_start = millis();

  while (client.connected()) {
    unsigned long frame_start = millis();
    
    // Get frame with timeout
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed - skipping frame");
      delay(10);  // Short delay before retry
      continue;
    }

    // Send frame header
    String frame_header = "--frame\r\n";
    frame_header += "Content-Type: image/jpeg\r\n";
    frame_header += "Content-Length: " + String(fb->len) + "\r\n";
    frame_header += "X-Frame-Number: " + String(frame_num++) + "\r\n\r\n";
    
    if(client.print(frame_header) && client.write(fb->buf, fb->len) && client.print("\r\n")) {
      // Calculate and display FPS every 30 frames
      frameCount++;
      unsigned long current_time = millis();
      if (frameCount % 30 == 0) {
        avgFPS = 30000.0 / (current_time - lastFrameTime);
        lastFrameTime = current_time;
        Serial.printf("Streaming FPS: %.1f, Frame size: %d bytes, Heap: %d KB\n", 
                     avgFPS, fb->len, ESP.getFreeHeap()/1024);
      }
    }
    
    esp_camera_fb_return(fb);
    
    // Frame rate control for car driving (aim for 25-30 FPS for very smooth experience)
    unsigned long frame_time = millis() - frame_start;
    if (frame_time < 40) {  // Target ~25 FPS for stability
      delay(40 - frame_time);
    }
    
    // Yield to prevent blocking
    yield();
  }
  
  Serial.println("Client disconnected from stream");
}

// Optimized camera configuration for car driving
camera_config_t getCameraConfig() {
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
  config.xclk_freq_hz = 10000000;  // Reduced to 10MHz for better stability
  config.pixel_format = PIXFORMAT_JPEG;

  // Optimized settings for car driving - Lower quality for smoother performance
  #ifdef CAR_DRIVING_MODE
    if(psramFound()){
      config.frame_size = FRAMESIZE_CIF;     // 352x288 - smaller for better stability
      config.jpeg_quality = 20;              // Lower quality for faster processing
      config.fb_count = 2;                   // Reduced buffers for stability
      config.fb_location = CAMERA_FB_IN_PSRAM;
      config.grab_mode = CAMERA_GRAB_LATEST; // Always get latest frame
    } else {
      config.frame_size = FRAMESIZE_QVGA;    // 320x240 - very small for no PSRAM
      config.jpeg_quality = 25;              // Lower quality for speed
      config.fb_count = 1;                   // Single buffer for limited memory
      config.grab_mode = CAMERA_GRAB_LATEST;
    }
  #else
    // Original settings
    if(psramFound()){
      config.frame_size = FRAMESIZE_UXGA;
      config.jpeg_quality = 10;
      config.fb_count = 2;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.jpeg_quality = 12;
      config.fb_count = 1;
    }
  #endif

  return config;
}

void startCameraServer() {
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><title>ESP32 Car Camera</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{margin:0;background:#000;display:flex;justify-content:center;align-items:center;height:100vh;}";
    html += "img{max-width:100%;max-height:100%;border:2px solid #fff;}</style></head>";
    html += "<body><img src='/stream' alt='Car Camera Stream'></body></html>";
    server.send(200, "text/html", html);
  });
  
  server.on("/stream", HTTP_GET, handle_jpg_stream);
  
  server.on("/status", HTTP_GET, []() {
    String json = "{";
    json += "\"fps\":" + String(avgFPS, 1) + ",";
    json += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
    json += "\"psram\":" + String(ESP.getFreePsram()) + ",";
    json += "\"frames\":" + String(frameCount) + ",";
    json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + "";
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.begin();
  Serial.println("Camera server started with car driving optimizations");
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);  // Disable debug for better performance
  Serial.println();

  // Optimize CPU frequency for performance
  setCpuFrequencyMhz(240);  // Maximum frequency
  
  Serial.println("Starting ESP32 Car Camera with optimizations...");
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf("PSRAM Found: %s\n", psramFound() ? "Yes" : "No");
  Serial.printf("Free Heap: %d KB\n", ESP.getFreeHeap()/1024);
  if(psramFound()) {
    Serial.printf("Free PSRAM: %d KB\n", ESP.getFreePsram()/1024);
  }

  // Initialize camera with optimized config
  camera_config_t config = getCameraConfig();

  // Try multiple times to initialize camera
  esp_err_t err;
  int camera_init_attempts = 0;
  do {
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init attempt %d failed with error 0x%x\n", camera_init_attempts + 1, err);
      delay(1000);
      camera_init_attempts++;
    }
  } while (err != ESP_OK && camera_init_attempts < 3);
  
  if (err != ESP_OK) {
    Serial.println("Camera initialization failed after multiple attempts - restarting...");
    ESP.restart();
  }

  // Optimize camera sensor settings for car driving
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    #ifdef CAR_DRIVING_MODE
      // Car driving optimizations - Simplified settings for stability
      s->set_framesize(s, FRAMESIZE_CIF);     // 352x288 for speed and stability
      s->set_quality(s, 20);                  // Balanced quality/speed
      s->set_brightness(s, 0);                // Auto brightness
      s->set_contrast(s, 0);                  // Auto contrast  
      s->set_saturation(s, 0);                // Normal saturation
      s->set_exposure_ctrl(s, 1);             // Enable exposure control
      s->set_gain_ctrl(s, 1);                 // Enable auto gain
      s->set_bpc(s, 0);                       // Disable bad pixel correction for speed
      s->set_lenc(s, 1);                      // Enable lens correction
      s->set_hmirror(s, 0);                   // No horizontal mirror
      s->set_vflip(s, 0);                     // No vertical flip
      
      Serial.println("Camera configured for stable car driving mode");
    #else
      // Original simple config
      s->set_framesize(s, FRAMESIZE_QVGA);
    #endif
  }

  // Configure WiFi with optimizations
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Maximum power for better range
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi - restarting...");
    ESP.restart();
  }

  Serial.println("");
  Serial.println("WiFi connected successfully!");
  Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
  Serial.print("Camera Ready! Access at: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  Serial.print("Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");
  Serial.print("Status API: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/status");

  startCameraServer();
  
  // Initialize performance counters
  lastFrameTime = millis();
  frameCount = 0;
  
  Serial.println("Car camera system ready for driving!");
}

void loop() {
  server.handleClient();
  delay(1);  // Small delay to prevent blocking
}
