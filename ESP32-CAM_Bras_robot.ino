/*
  Minimum code for Autoconnect:
    https://hieromon.github.io/AutoConnect/gettingstarted.html

  Sources: 
    Camera streaming: https://github.com/Hieromon/AutoConnect/blob/master/examples/WebCamServer/WebCamServer.ino
    Caution: esp32ap wifi access point password: 12345678
    https://esp32io.com/tutorials/esp32-websocket

  Libraries:
    Autoconnect (available in Arduino library manager): https://github.com/Hieromon/AutoConnect
    WebSockets by Markus Sattler: https://github.com/Links2004/arduinoWebSockets
    Preference NVS memory access https://github.com/vshymanskyy/Preferences
    ArduinoJson 6.21.5

  D1 BoardNodeMCU ESP8266 (AzDelivery)
    Board: esp8266 --> Arduino
    Caution: 2019 & 2023 Driver must be uninstalled from Windows
    Compilation fails when ESP8266 selected
  ESP-EYE
    Board: ESP32 Dev Module
    Code test: Ok
  ESP32-CAM
    Board: AI Thinker ESP32-CAM
    Code test: Ok
  ESP32_DEVKIT
    Board: ESP32 dev kit 
    Code test: not tested


  url à tester: voir readme.md

*/

//#define ESP_EYE
#define ESP32_CAM
//#define ESP8266
//#define ESP32_DEVKIT

// DEBUG
#define DEBUG
#define BAUD_RATE 9600      // LU9685 baud rate

/**
 * Specify one of the following supported sensor modules to be actually applied.
 * However, not all of the devices have been fully tested. Activating the timer-
 * shot may cause a WDT RESET during streaming.
 * 
 * camera model declaration moved later in code
 */
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_AI_THINKER;
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_ESP_EYE;
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_M5STACK_ESP32CAM;
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_M5STACK_NO_PSRAM;
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_M5STACK_PSRAM;
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_M5STACK_UNITCAM;
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_M5STACK_V2_PSRAM;
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_M5STACK_WIDE;
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_TTGO_T_JOURNAL;
// const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_WROVER_KIT;



// Version
const char *applicationName = "Servo & esp32-cam test";
const char *filename = "ESP2-CAM_Bras_robot.ino";
const char *appVersion = "2.10";
const char *appTitle = "Servo v2.10";

#if defined(ARDUINO_ARCH_ESP8266)
  #include <ESP8266WiFi.h>
  #include <WiFiClient.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266mDNS.h>
  const char *host = "esp8266";
  ESP8266WebServer server(80);
#elif defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  const char *host = "esp32";
  WebServer server(80);
  // NVS access : Preference library
  #include <Preferences.h>
  Preferences preferences;
  const char *pref_namespace = "AC_data";    // Preference namespace
  #define READ_WRITE false
  #define READ_ONLY true
#endif
#include <AutoConnect.h>
#include <WebSocketsServer.h>
#include "ESP32WebCam.h"
#include <stdio.h>
#include <stdarg.h>

#if defined(ESP_EYE) 
  #define LED_BUILTIN 21
  const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_ESP_EYE;
#elif defined(ESP32_CAM)
  #define LED_BUILTIN 33
  const ESP32Cam::CameraId  model = ESP32Cam::CameraId::CAMERA_MODEL_AI_THINKER;
#elif defined(ESP32_DEVKIT)
  #define LED_BUILTIN 22  
#elif defined(ESP8266)
  //LED_BUILTIN already declared
#endif

// Servo 

static const int angle_max[] = {180,120,180,180,180,180,180,180,180,180,180,180,180,180,180,180};
static const int angle_min[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static const int angle_init[] = {90,45,140,90,90,90,90,90,90,90,90,90,90,90,90,90};
char debug_string[512];

/* replaced by onRoot
static const char HOME_PAGE[] PROGMEM = R"(
{
  "uri": "/",
  "title": "Home page",
  "menu": false,
  "element": [
    {
      "name": "text",
      "type": "ACText",
      "value": "Home page",
      "style": "font-family:Arial;font-size:18px;font-weight:400;color:#191970"
    },
    {
      "name": "text2",
      "type": "ACText",
      "value": "List files",
      "style": "font-family:Arial;font-size:18px;font-weight:400;color:#191970",
      "uri": "/list?dir=/"
    }    
  ]
}
)";
*/

static const char InputPage[] PROGMEM = R"(
{
  "name" : "inputForm",
  "uri" : "/input",
  "menu" : true,
  "element" : [
    {
      "name": "input",
      "type": "ACInput",
      "label": "Input"
    },
    {
      "name": "saveInput",
      "type": "ACSubmit",
      "value": "Save",
      "uri": "/"
    }
  ]
}
)";

static const char LED_ONOFF[] PROGMEM = R"(
{
  "uri": "/led",
  "title": "LED",
  "menu": true,
  "element": [
    {
      "name": "caption",
      "type": "ACText",
      "value": "BUILT-IN LED",
      "style": "font-weight:bold;font-size:25px;text-align:center;",
      "posterior": "div"
    },
    {
      "name": "onoff",
      "type": "ACButton",
      "value": "ON"
    }
  ]
}
)";


// Transition destination for CAMERA_SETUP_PAGE
// It will invoke the handler as setSensor function for setting the image sensor.
// The `response` is ana attribute added in AutoConnect v1.3.2 to suppress the
// HTTP response from AutoConnect. AutoConnectAux handlers with the
// `"response":false` can return their own HTTP response.
const char  CAMERA_SETUP_EXEC[] = R"*(
{
  "title": "Camera",
  "uri": "/set",
  "response": false,
  "menu": false
}
)*";

void debug_printf( const char * format, ... )
{
  #if defined(DEBUG)
    char buffer[256];
    va_list args;
    va_start (args, format);
    vsnprintf (buffer,256,format, args);
    Serial.print(buffer);
    va_end (args);
  #endif
}
void debug_printf( String s )
{
  #if defined(DEBUG)
    Serial.print(s);
  #endif
}
// format bytes to string
String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

String getContentType(String filename) {
  if (server.hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  }
  return "text/plain";
}

bool handleFileRead(String path) {
  if (path.endsWith("/")) {
    path += "index.html";
  }
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  debug_printf("handleFileRead:");
  //debug_printf("handleFileRead: %s\n", path);
  debug_printf(path);
  debug_printf("\n");
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
    if (SPIFFS.exists(pathWithGz)) {
      path += ".gz";
    }
    debug_printf("Opening file\n");
    File file = SPIFFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    debug_printf("File closed\n");
    return true;
  } else {
    Serial.print("File not found:");
    Serial.println(path);
  }
  return false;
}

// Source: https://github.com/Hieromon/AutoConnect/blob/master/examples/WebCamServer/WebCamServer.ino
// ESP32WebCam instance; Hosts a different web server than the one hosted by AutoConect.
// If you want to assign a different port than the default, uncomment the
// following two lines and enable _cameraServerPort value. _cameraServerPort is
// the port on which the http server started by ESP32WebCam will listen.
const uint16_t _cameraServerPort = 82; // Default is 3000.
ESP32WebCam webcam(_cameraServerPort);
//ESP32WebCam webcam;

// ===================== AutoConnect global var ==================================================
AutoConnect portal(server);
AutoConnectConfig config;
AutoConnectAux auxInput;
AutoConnectAux auxMqttSettings;
AutoConnectAux auxHelloWorld;
AutoConnectAux auxMqttSave;
AutoConnectAux auxCameraSetup;
AutoConnectAux FSBedit("/edit", "Edit");      // add "Edit" to AutoConnect Menu
//AutoConnectAux FSBlist("/list?dir=\"/\"", "List");
AutoConnectAux FSBlist("/list?dir=/", "List"); // add "List" to AutoConnect Menu
//holds the current upload
File fsUploadFile;

// ===================== File system Event handler ===========================================================
void handleFileList() {
  if (!server.hasArg("dir")) {
    server.send(500, "text/plain", "BAD ARGS");
    return;
  }
  
  String path = server.arg("dir");
  debug_printf("handleFileList: %s\n", path);
  #ifdef ARDUINO_ARCH_ESP8266
    Dir dir = SPIFFS.openDir(path);
  #elif defined(ARDUINO_ARCH_ESP32)
    File root = SPIFFS.open(path);
  #endif
  path = String();

  String output = "[";
  #ifdef ARDUINO_ARCH_ESP8266
    while (dir.next()) {
      File entry = dir.openFile("r");
      if (output != "[") {
        output += ',';
      }
      bool isDir = false;
      output += "{\"type\":\"";
      output += (isDir) ? "dir" : "file";
      output += "\",\"name\":\"";
      output += String(entry.name()).substring(1);
      output += "\",\"size\":\"";
      output += String(entry.size());
      output += "\"}";
      entry.close();
    }
  #elif defined(ARDUINO_ARCH_ESP32)
    if(root.isDirectory()){
      File file = root.openNextFile();
      while(file){
        if (output != "[") {
          output += ',';
        }
        output += "{\"type\":\"";
        output += (file.isDirectory()) ? "dir" : "file";
        output += "\",\"name\":\"";
        output += String(file.name()).substring(0);
        output += "\",\"size\":\"";
        output += String(file.size());
        output += "\"}";
        file = root.openNextFile();
      }
    }
  #endif
  
  output += "]";
  server.send(200, "text/json", output);
}

void handleFileUpload() {
  if (server.uri() != "/edit") {
    return;
  }
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) {
      filename = "/" + filename;
    }
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    //Serial.print("handleFileUpload Data: "); Serial.println(upload.currentSize);
    if (fsUploadFile) {
      fsUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {
      fsUploadFile.close();
    }
    Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
  }
}

void handleFileDelete() {
  if (server.args() == 0) {
    return server.send(500, "text/plain", "BAD ARGS");
  }
  String path = server.arg(0);
  Serial.println("handleFileDelete: " + path);
  if (path == "/") {
    return server.send(500, "text/plain", "BAD PATH");
  }
  if (!SPIFFS.exists(path)) {
    return server.send(404, "text/plain", "FileNotFound");
  }
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate() {
  if (server.args() == 0) {
    return server.send(500, "text/plain", "BAD ARGS");
  }
  String path = server.arg(0);
  Serial.println("handleFileCreate: " + path);
  if (path == "/") {
    return server.send(500, "text/plain", "BAD PATH");
  }
  if (SPIFFS.exists(path)) {
    return server.send(500, "text/plain", "FILE EXISTS");
  }
  File file = SPIFFS.open(path, "w");
  if (file) {
    file.close();
  } else {
    return server.send(500, "text/plain", "CREATE FAILED");
  }
  server.send(200, "text/plain", "");
  path = String();
}

// Event handler that attaches to an AutoConnectButton named `led`.
// This event handler receives a reference to AutoConnectButton as `led`
// and a reference to the AutoConnectAux of the page rendered in the client
// browser.
void ledOnOff(AutoConnectButton& me, AutoConnectAux& ledOnOff) {
  debug_printf("ledOnOff execution: %s",  me.value);
  if (me.value == "ON") {
    // Since "ON" has been passed from the AutoConnectButton as `led`. Let the
    // LED turns on.
    #if defined(ESP_EYE) || defined(ESP32_DEVKIT) || defined(ESP8266)
      digitalWrite(LED_BUILTIN, HIGH);
    #elif defined(ESP32_CAM)
      digitalWrite(LED_BUILTIN, LOW);
    #endif
    // Direct assignment to AutoConnectElement values is not reflected on the
    // web page; use the `response` function to update the value of the element
    // on the web page.
    me.response("OFF");
    // The `on` event handler attached to AutoConnectElements can override the
    // value and attributes of other elements placed on that AutoConnectAux page.
    // For example, a following statement changes the font color of the `caption`
    // element along with a LED blinking.
    ledOnOff["caption"].response("style", "{\"color\":\"red\"}");
  }
  if (me.value == "OFF") {
    // Since a value "OFF" has been passed from the AutoConnectButton as `led`.
    // Let the LED turns off.
    #if defined(ESP_EYE) || defined(ESP32_DEVKIT) || defined(ESP8266)
      digitalWrite(LED_BUILTIN, LOW);
    #elif defined(ESP32_CAM)
      digitalWrite(LED_BUILTIN, HIGH);
    #endif
    me.response("ON");
    ledOnOff["caption"].response("style", "{\"color\":\"black\"}");
  }
}


// ===================== Cam procedures =============================================================
// Endpoint that leads request to the root page to webcamview.html
const char*  const _viewerUrl = "/webcam"; 
// Viewer-UI content
const char*  const _webcamserver_html = "/webcamview.html";

// AutoConnectAUx entry point
const char*  const _sensorUrl = "/_setting";
const char*  const _setUrl = "/set";

// Choose the file system properly to fit the SD card interface of the ESP32
// module you are using.
// In typical cases, SD is used for the VSPI interface, and MMC is used for the
// HS2 interface.
const char*  const _fs = "sd";
//const char*  const _fs = "mmc";

// Specifying the time zone and assigning NTP.
// Required to add the correct local time to the export file name of the
// captured image. This assignment needs to be localized.
// This sketch works even if you omit the NTP server specification. In that
// case, the suffix timestamp of the captured image file is the elapsed time
// since the ESP module was powered on.
const char*  const _tz = "UTC+1";
const char*  const _ntp1 = "europe.pool.ntp.org";
const char*  const _ntp2 = "pool.ntp.org";

// AutoConnectAux handler for CAMERA_SETUP_EXEC
// Read the camera status via the ESP32 Camera Driver and update it with the
// sensor parametes defined in each AutoConnect element in CAMERA_SETUP?PAGE.
String setSensor(AutoConnectAux& aux, PageArgument& args) {
  AutoConnectAux& cameraSetup = *portal.aux(_sensorUrl);    // /_setting
  camera_status_t status;

  // Take over the current settings
  webcam.sensor().getStatus(&status);

  // Framesize
  const String& resolution = cameraSetup["res"].as<AutoConnectSelect>().value();
  debug_printf(resolution);
  if (resolution == "UXGA(1600x1200)")
    status.framesize = FRAMESIZE_UXGA;
  else if (resolution == "SXGA(1280x1024)")
    status.framesize = FRAMESIZE_SXGA;
  else if (resolution == "XGA(1024x768)")
    status.framesize = FRAMESIZE_XGA;
  else if (resolution == "SVGA(800x600)")
    status.framesize = FRAMESIZE_SVGA;
  else if (resolution == "VGA(640x480)")
    status.framesize = FRAMESIZE_VGA;
  else if (resolution == "CIF(400x296)")
    status.framesize = FRAMESIZE_CIF;
  else if (resolution == "QVGA(320x240)")
    status.framesize = FRAMESIZE_QVGA;
  else if (resolution == "HQVGA(240x176)")
    status.framesize = FRAMESIZE_HQVGA;
  else if (resolution == "QQVGA(160x120)")
    status.framesize = FRAMESIZE_QQVGA;

  // Pixel granularity
  status.quality = cameraSetup["qua"].as<AutoConnectRange>().value;

  // Color solid adjustment
  status.contrast = cameraSetup["con"].as<AutoConnectRange>().value;
  status.brightness = cameraSetup["bri"].as<AutoConnectRange>().value;
  status.saturation = cameraSetup["sat"].as<AutoConnectRange>().value;

  // SE
  const String& se = cameraSetup["se"].as<AutoConnectSelect>().value();
  debug_printf(se);
  if (se == "No Effect")
    status.special_effect = 0;
  if (se == "Negative")
    status.special_effect = 1;
  if (se == "Grayscale")
    status.special_effect = 2;
  if (se == "Red Tint")
    status.special_effect = 3;
  if (se == "Green Tint")
    status.special_effect = 4;
  if (se == "Blue Tint")
    status.special_effect = 5;
  if (se == "Sepia")
    status.special_effect = 6;

  // White Balance effection
  status.awb = cameraSetup["awb"].as<AutoConnectCheckbox>().checked ? 1 : 0;
  status.awb_gain = cameraSetup["wbg"].as<AutoConnectCheckbox>().checked ? 1 : 0;
  const String& wbMode = cameraSetup["wbm"].as<AutoConnectSelect>().value();
  debug_printf(wbMode);
  if (wbMode == "Auto")
    status.wb_mode = 0;
  else if (wbMode == "Sunny")
    status.wb_mode = 1;
  else if (wbMode == "Cloudy")
    status.wb_mode = 2;
  else if (wbMode == "Office")
    status.wb_mode = 3;
  else if (wbMode == "Home")
    status.wb_mode = 4;

  // Automatic Exposure Control, Turn off AEC to set the exposure level.
  status.aec = cameraSetup["aec"].as<AutoConnectCheckbox>().checked ? 1 : 0;
  status.aec2 = cameraSetup["dsp"].as<AutoConnectCheckbox>().checked ? 1 : 0;
  status.ae_level = cameraSetup["ael"].as<AutoConnectRange>().value;
  status.aec_value = cameraSetup["exp"].as<AutoConnectRange>().value;

  // Automatic Gain Control
  status.agc = cameraSetup["agc"].as<AutoConnectCheckbox>().checked ? 1 : 0;
  status.agc_gain = cameraSetup["agv"].as<AutoConnectRange>().value - 1;
  status.gainceiling = cameraSetup["acl"].as<AutoConnectRange>().value - 1;

  // Gamma (GMA) function is to compensate for the non-linear characteristics of
  // the sensor. Raw gamma compensates the image in the RAW domain.
  status.raw_gma = cameraSetup["gma"].as<AutoConnectCheckbox>().checked ? 1 : 0;
  
  // Defect pixel cancellation, Black pixel and White pixel
  status.bpc = cameraSetup["bpc"].as<AutoConnectCheckbox>().checked ? 1 : 0;
  status.wpc = cameraSetup["wpc"].as<AutoConnectCheckbox>().checked ? 1 : 0;

  // Lense correction, According to the area where each pixel is located,
  // the module calculates a gain for the pixel, correcting each pixel with its
  // gain calculated to compensate for the light distribution due to lens curvature.
  status.lenc = cameraSetup["lec"].as<AutoConnectCheckbox>().checked ? 1 : 0;

  // Mirror and Flip
  status.hmirror = cameraSetup["hmi"].as<AutoConnectCheckbox>().checked ? 1 : 0;
  status.vflip = cameraSetup["vfl"].as<AutoConnectCheckbox>().checked ? 1 : 0;

  // Reducing image frame UXGA to XGA
  status.dcw = cameraSetup["dcw"].as<AutoConnectCheckbox>().checked ? 1 : 0;

  // Reflecting the setting values on the sensor
  if (!webcam.sensor().setStatus(status))
    Serial.println("Failed to set camera sensor");

  // Sends a redirect to forward to the root page displaying the streaming from
  // the camera.
  aux.redirect(viewerUrl().c_str());

  return String();
}

// Assemble the query string for webcamview.html according the configuration
// by the sketch.
// The webcamview.html accepts the following query parameters:
// - host=HOSTNAME                            (default: 0.0.0.0)
// - port=PORT_NUMBER                         (default: 3000)
// - stream=STREAMING_PATH                    (default: _stream)
// - capture=CAPTURING_PATH                   (default: _capture)
// - prompt=PROMPTING_PATH_FOR_REMOTE_CONTROL (default: _prompt)
// - setting=AUTOCONNECTAUX_SETTING_PATH      (default: undefined)
// - fs={sd|mmc}                              (default: mmc)
// - ac=AUTOCONNECT_ROOT_PATH                 (default: _ac) 
// - title=VIEWER_PAGE_HEADER_TITLE           (default: undefined)
String viewerUrl(void) {
  String  url = String(_viewerUrl)
              + "?host=" + WiFi.localIP().toString()
              + "&port=" + String(webcam.getServerPort())
              + "&capture=" + String(webcam.getCapturePath())
              + "&prompt=" + String(webcam.getPromptPath())
              + "&stream=" + String(webcam.getStreamPath())
              + "&fs=" + String(_fs)
              + "&ac=" AUTOCONNECT_URI
              + "&setting=" + String(_sensorUrl)
              + "&title=" + String(WiFi.getHostname());
  return url;
}

// Directs the request to the root to webcamview.html.
// This function exists only to assemble the query string.
void handleCamPage(void) {
  server.sendHeader("Location", viewerUrl());
  server.send(302, "text/plain", "");
  server.client().stop();
}

// Read webcamview.html from SPIFFS and send it to the client as response.
// The query string given by handleRootPage is taken over tby the 302-redirect.
void handleViewPage(void) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi is disconnected.");
    Serial.println("Viewer will not be available until connection is restored.");
    server.send(500, "text/plain", "WiFi is not connected. Viewer will not be available until connection is restored.");
    return;
  }

  File viewPage = SPIFFS.open(_webcamserver_html, FILE_READ);
  if (viewPage) {
    server.streamFile(viewPage, "text/html");
    close(viewPage);
  }
  else {
    Serial.printf("Viewer %s could not load, check your data folder and upload it.\n", _webcamserver_html);
    server.send(500, "text/plain", String(_webcamserver_html) + " open failed");
  }
}

// ===================== MQTT procedures =============================================================
// The handler called before HTML generating
String initMqttSettingsForm(AutoConnectAux& aux, PageArgument& args) {
  debug_printf("Init mqtt settings\n");
  AutoConnectInput& mqttserver = aux.getElement<AutoConnectInput>("mqttserver");
  // Set initial value for the input box in the form.
  //mqttserver.value = "Initial value";  
  mqttserver.value = getMqttSettings();

  //AutoConnectText& caption = aux.getElement<AutoConnectText>("caption");
  //caption.value = "Input mqtt broker settings";
  return String();
}

// HTTP Post event handler for mqttSettings page
// https://hieromon.github.io/AutoConnect/acupload.html#basic-steps-of-the-file-upload-sketch
String postMqttSettings(AutoConnectAux& aux, PageArgument& args) {
  // When mqtt_Settings Save button is clicked --> invoke HTTP POST to /mqtt_save page
  // aux: target page = /mqtt_save
  // args: HTTP post arguments from source page = /mqtt_settings
  AutoConnectText& saveStatusMsg = aux.getElement<AutoConnectText>("caption2");
  saveStatusMsg.value = "MQTT settings saved";
  
  saveMqttSettings(args.arg("mqttserver"));
  return String();
}
/*
void onMqttSave(AutoConnectAux& aux, PageArgument& args) {
  auxMqttSettings.fetchElement();    // Preliminary acquisition

  // For this steps to work, need to call fetchElement function beforehand.
  String value = auxMqttSettings["mqttserver"].value;
  Serial.println("MQTT server:" + value);

}
*/

// ===================== Preferences =============================================================

size_t saveMqttSettings(String brocker){
  Serial.printf("Saving setting to SPIFFS: %s\n", brocker);
  preferences.begin(pref_namespace, READ_WRITE);
  size_t r = preferences.putString("mqttBroker", brocker);
  preferences.end();
  return r;
}

String getMqttSettings(){
  preferences.begin(pref_namespace, READ_ONLY);
  String r = preferences.getString("mqttBroker", "");
  preferences.end();
  return r;
}


/* ===================== LU9685 & webSocket ============================================
addr: the hardware address of the module
num: channels 0-15
OFF is 0-180, and no signal is output when OFF is greater than or equal to 200, and the servo is released at this time
*/

// Select LU9685 address
#if defined(ESP32_CAM)
  // ESP32-CAM connected to LU9685 via Serial port
  #define LU9685_adrr1 0x08   // Solder bridge on A2. Caution UART address != I2C Address.
#elif defined(ESP32_DEVKIT)
  // ESP32 Devkitc V4 connected to LU9685 via I2C
  #define LU9685_adrr1 0x04 // Solder bridge on A2. Caution UART address != I2C Address. 
#elif defined(ESP_EYE)
  // no LU9685 connected
#elif defined(ESP8266)
  // to be completed
#endif

#define AC_DEBUG
WebSocketsServer webSocket = WebSocketsServer(81);

void reset_LU9685() {
  Serial.println("Reseting LU9685:\n");
  #ifdef ESP32_DEVKIT
    Wire.beginTransmission(LU9685_adrr1);
    Wire.write(0xFB);
    Wire.write(0xFB);
    Wire.endTransmission();
  #elif defined(ESP32_CAM)
    Serial.write(0xFA);
    Serial.write(LU9685_adrr1);
    Serial.write(0xFB);
    Serial.write(0xFB);
    Serial.write(0xFE);
  #elif defined(ESP_EYE)
    // do nothing
  #elif defined(ESP8266)
    // to be completed
  #endif
  debug_printf("\n");
}

void setPWM(int servo, int angle){
  if(angle < angle_min[servo]) angle = angle_min[servo];
  if(angle > angle_max[servo]) angle = angle_max[servo];
  debug_printf("Set servo %i to %i\n", servo, angle);

  #ifdef ESP32_DEVKIT
    byte error;
    Wire.beginTransmission(LU9685_adrr1);
    Wire.write(servo);
    Wire.write(angle);
    error = Wire.endTransmission();
    if (error != 0) {
      Serial.printf("I2C error:%s\n", error)
    }
  #elif defined(ESP32_CAM)
    Serial.write(0xFA);
    Serial.write(LU9685_adrr1);
    Serial.write(servo);
    Serial.write(angle);
    Serial.write(0xFE);
    debug_printf("\n");
  #elif defined(ESP_EYE)
    Serial.println("No servo on ESP_EYE");
  #elif defined(ESP8266)
    Serial.println("Not implemented on ESP8266");  
  #endif
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
  /*
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] Received text: %s\n", num, payload);
      // Send a response back to the client
      webSocket.sendTXT(num, "Received:  " + String((char*)payload));
      break;
  }
  */
  if(type == WStype_TEXT){
    if(payload[0] == 'S'){ // S0=90
      uint16_t servoNum = (uint16_t) strtol((const char *) &payload[1], NULL, 16);
      uint16_t servoVal = (uint16_t) strtol((const char *) &payload[3], NULL, 10);

      debug_printf("cmd: S%i=%i\n", servoNum, servoVal);

      setPWM(servoNum, servoVal);
      webSocket.sendTXT(num, "\nReceived:  " + String((char*)payload));
    }

    else{
      char buffer[length];
      for(int i = 0; i < length; i++)
        buffer[i] = payload[i];
      debug_printf("%s\n",buffer);
    }
  }
}

void initServos(){
  for(int i = 0; i < 16; i++){
    setPWM(i, angle_init[i]);
  }
}

void time_sync_notification(struct timeval *tv)
{
  debug_printf("Notification of a time synchronization event");
}
// ===================== Setup ===========================================================


void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);
  Serial.print("\n");
  Serial.printf("\n%s  v%s (%s)\n", applicationName, appVersion, filename);
  
  // Led initialization -led off)
  pinMode(LED_BUILTIN, OUTPUT);
  #if defined(ESP_EYE) || defined(ESP32_DEVKIT) || defined(ESP8266)
    digitalWrite(LED_BUILTIN, LOW);
  #elif defined(ESP32_CAM)
    digitalWrite(LED_BUILTIN, HIGH);
  #endif

  #ifdef ARDUINO_ARCH_ESP8266
    debug_printf("Configured for ESP8266 architecture\n");
  #elif defined(ARDUINO_ARCH_ESP32)
    debug_printf("Configured for ESP32 architecture\n");
  #endif

  #ifdef ESP32_CAM
    debug_printf("Configured for ESP32-CAM\n");
  #elif defined(ESP32_DEVKIT)
    debug_printf("Configured for ESP32 devkit C V4\n");
    Wire.begin();        // i2c bus (address optional for master)
  #elif defined(ESP_EYE)
    debug_printf("Configured for ESP-EYE\n");
  #elif defined(ESP8266)
    debug_printf("Configured for ESP8266\n");    
  #endif

  reset_LU9685();

  Serial.setDebugOutput(true);
  
  // Initialize the image sensor during the start phase of the sketch.
  esp_err_t err = webcam.sensorInit(model);
  if (err != ESP_OK)
    Serial.printf("Camera init failed 0x%04x\n", err);



  // AutoConnect configuration
  config.title = appTitle; 
  
  /*
  config.auth = AC_AUTH_DIGEST;
  config.authScope = AC_AUTHSCOPE_AUX;
  config.username = "user";
  config.password = "password";
  */
  portal.config(config);
  portal.home("/");  // default value = /

 
  // Load AutoConnect custom web page from SPIFFS
  // AutoConnect custom web pages with json
  // https://hieromon.github.io/AutoConnect/acjson.html
  debug_printf("Loading custom AC pages...\n");

  //AutoConnectAux input page;
  auxInput.load(InputPage);
  //portal.join(auxInput);

  SPIFFS.begin();
  {
    File json_file;

    //json_file = SPIFFS.open("index.html", "r");
    //portal.load(json_file);
    //portal.on("")

    // Loading the image sensor configurarion UI provided by AutoConnectAux.
    json_file = SPIFFS.open("/camera_setup.json", "r");
    if(!auxCameraSetup.load(json_file)) {
      Serial.println("ERROR: camera_setup page load failed");
    }
    //portal.load(json_file);  // Autre solution

    if (portal.load(FPSTR(CAMERA_SETUP_EXEC)))
    portal.on(_setUrl, setSensor);

    portal.load(FPSTR(LED_ONOFF));

    json_file = SPIFFS.open("/hello_world.json", "r");
    if(!auxHelloWorld.load(json_file)){
      Serial.println("ERROR: hello_world page load failed");
    };
    //portal.load(json_file); //Autre solution
    json_file.close();

    json_file = SPIFFS.open("/mqtt_settings.json", "r");
    if(!auxMqttSettings.load(json_file)){
      Serial.println("ERROR: mqtt_settings page load failed");
    };
    //portal.load(json_file); //Autre solution
    json_file.close();

    json_file = SPIFFS.open("/mqtt_save.json", "r");
    if(!auxMqttSave.load(json_file)){
      Serial.println("ERROR: mqtt_save page load failed");
    };
    //portal.load(json_file); //Autre solution
    json_file.close();
  }
  SPIFFS.end();

  portal.join({auxInput, auxCameraSetup, auxHelloWorld, auxMqttSettings, auxMqttSave, FSBedit, FSBlist});

  // give your custom web page the ability to handle events using Fetch API
  // https://hieromon.github.io/AutoConnect/acinteract.html#allow-autoconnectelements-to-have-event-processing
  // Note: La déclaration de la page /led n'a pas utilisé AutoConnectAux, donc on récupère la réfrérence maintenant
  AutoConnectAux& led = portal.locate("/led");  // Récupère une référence vers l'AutoConnectAux de la page /led
  AutoConnectButton& onOff = led["onoff"].as<AutoConnectButton>();  // onOff est la référence vers AutoConnectButton qui hérite de AutoConnectElement 
  onOff.on(ledOnOff); // Register the event handler with the AutoConnectElement::on function

  // mqttSettings form:
  auxMqttSettings.on(initMqttSettingsForm, AC_EXIT_AHEAD);
  //auxMqttSettings.on(appendMqttSettingsForm, AC_EXIT_LATER);      // Caution: this new handler override the previous one

  // mqttSettings form HTTP POST --> Invoke /mqtt_save page 
  // mqtt_save page handler configured with AutoConnectAux object
  auxMqttSave.on(postMqttSettings); // HTTP Post event handler configuration

  //Web page out of AutoConnect : root page
  //server.on("/", onRoot);  // Register the on-page handler (onRoot was inline HTML string)

  //Web page out of AutoConnect : list directory
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, []() {
    if (!handleFileRead("/edit.htm")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, []() {
    server.send(200, "text/plain", "");
  }, handleFileUpload);


  server.on("/ws_basic", HTTP_GET, []() {
    if (!handleFileRead("/ws_basic.html")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  server.on("/ws_servo2", HTTP_GET, []() {
    if (!handleFileRead("/ws_servo2.html")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  server.on("/ws_servo1", HTTP_GET, []() {
    if (!handleFileRead("/ws_servo1.html")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  // InputPage HTTP POST handler
  // Page handler configured with AutoConnectAux object
  //auxInput.on(postInputPage);
  
  // Page handler configured with AutoConnect object (portal)
  // portal.on("/input", postInputPage);


  // Undocumented feature
  //AutoConnectAux FSBedit("/edit", "Edit");
  //AutoConnectAux FSBlist("/list?dir=\"/\"", "List");
  //portal.join({ FSBedit, FSBlist });


  //called when the url is not defined here
  //use it to load static content from SPIFFS (used for index.html)
  //Replacement as follows to make AutoConnect recognition.
  //server.onNotFound([](){
  portal.onNotFound([](){
    debug_printf("uri not found : %s\n", server.uri());
    if(!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });
  debug_printf("404 handler started");


  Serial.println("portal about to begin: join the Wifi esp32ap AP with your phone");
  Serial.println("Then a window is opened on the phone to configure the Wifi SSID and password.");
  Serial.println("If this operation has been done, the following logs will ask to browse the url.");
  if (portal.begin()) {
    Serial.println("Web server started:" + WiFi.localIP().toString());
    //Serial.println("  Browse url: http://" + WiFi.localIP().toString() + "/");
    //Serial.println("  Browse url: http://" + WiFi.localIP().toString() + "/list?dir=/");
    Serial.println("Disconnect command will reset the full configuration");

    // Allow hostname.local reach in the local segment by mDNS.
    // MDNS.begin(WiFi.getHostname());
    // exemple de Wifi.getHostname() : esp32-773E11.local

    if (MDNS.begin(host)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("Open http://%s.local \n", host);
      Serial.printf("Autoconnect menu: http://%s.local/_ac \n", host);
    }
    else {
      Serial.println("mDNS start failed");
    }

    // By configuring NTP, the timestamp appended to the capture filename will
    // be accurate. But this procedure is optional. It does not affect ESP32Cam
    // execution.
    //configTzTime(_tz, _ntp1 ,_ntp2);
    configTzTime("CET-1CEST,M3.5.0/03,M10.5.0/03", _ntp1);
    // Activate ESP32WebCam while WiFi is connected.

    //sntp_set_time_sync_notification_cb(time_sync_notification);
    //configTzTime("CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", _ntp1);

    #if defined(DEBUG)
      #define MAX_SIZE 64
      char buffer[MAX_SIZE];

      struct tm timeInfo;
      getLocalTime(&timeInfo);
      strftime(buffer, MAX_SIZE, "%d/%m/%Y %H:%M:%S", &timeInfo);
      Serial.println(buffer);
    #endif

    err = webcam.startCameraServer();

    if (err == ESP_OK) {
      // This sketch has two endpoints. One assigns the root page as the
      // entrance, and the other assigns a redirector to lead to the viewer-UI
      // which we have placed in SPIFFS with the name webcamview.html.
      server.on("/cam", handleCamPage);
      server.on(_viewerUrl, handleViewPage);
      Serial.printf("Camera streaming server  %s ready, port %u\n", WiFi.localIP().toString().c_str(), webcam.getServerPort());
      Serial.printf("Endpoint Capture:%s, Prompt:%s, Stream:%s\n", webcam.getCapturePath(), webcam.getPromptPath(), webcam.getStreamPath());
    }
    else
      Serial.printf("Camera server start failed 0x%04x\n", err);
  } else {
    Serial.println("portal.begin failure!");
  }



  // List all files stored in SPIFFS in Serial (not in web server)
  SPIFFS.begin();
  {
    #if defined(ARDUINO_ARCH_ESP8266)
      Dir dir = SPIFFS.openDir("/");
      while (dir.next()) {
        String fileName = dir.fileName();
        size_t fileSize = dir.fileSize();
        Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
      }
    #elif defined(ARDUINO_ARCH_ESP32)
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while (file) {
        String fileName = file.name();
        size_t fileSize = file.size();
        Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
        file = root.openNextFile();
      }
    #endif
    Serial.printf("\n");
  }

  // Servo control
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  initServos();

}

void loop() {
  portal.handleClient();

  #ifdef ARDUINO_ARCH_ESP8266
    MDNS.update();
  #endif
  webSocket.loop();
  // Attention Serial est utilisé par LU9685 sur ESP32-CAM
  if(Serial.available() > 0){     // forward incoming Serial command to webSocket
    char c[] = {(char)Serial.read()};
    webSocket.broadcastTXT(c, sizeof(c));
  }

  // Allow CPU to switch to other tasks.
  delay(1);
}