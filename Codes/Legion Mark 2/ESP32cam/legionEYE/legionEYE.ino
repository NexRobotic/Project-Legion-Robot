#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"          // disable brownout problems
#include "soc/rtc_cntl_reg.h" // disable brownout problems
#include "esp_http_server.h"

// Replace with your network credentials
const char *ssid = "LegionEYE";
const char *password = "123456789";

bool drivemode = false;

#define PART_BOUNDARY "123456789000000000000987654321"

#define CAMERA_MODEL_AI_THINKER
// #define CAMERA_MODEL_M5STACK_PSRAM
// #define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM
// #define CAMERA_MODEL_M5STACK_PSRAM_B
// #define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 25
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 32
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 25
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 17
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM_B)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 22
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 32
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#else
#error "Camera model not selected"
#endif

static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <title>ESP32-CAM Robot</title>
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <style>
      body {
        font-family: Arial;
        text-align: center;
        margin: 0px auto;
        padding-top: 30px;
      }
      table {
        margin-left: auto;
        margin-right: auto;
      }
      td {
        padding: 8 px;
      }
      .button {
        background-color: #2f4468;
        border: none;
        color: white;
        padding: 10px 20px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 18px;
        margin: 6px 3px;
        cursor: pointer;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0, 0, 0, 0);
      }
      img {
        width: auto;
        max-width: 100%;
        height: auto;
      }
      .main {
        justify-content: center;
        display: flex;
        flex-wrap: wrap;
        gap: 2rem;
      }
      .video {
        display: inline;
        background-color: aliceblue;
        align-self: center;
        width: auto;
        height: auto;
      }
      .mode-text {
        font-weight: bold;
        font-size: 16px;
        color: #6c757d; /* Default Grey */
        text-align: center;
        margin-top: 10px;
      }
      .control-bar {
        background-color: #2f4468;
        height: auto;
        width: 100%;
        display: flex;
        flex-direction: row;
        flex-wrap: wrap;
      }
      .control-bar-field {
        height: auto;
        width: auto;
        display: block;
        flex: 1;
      }
      .control-panel {
        display: flex;
        flex-direction: row;
        gap: 1rem;
        background-color: aquamarine;
        justify-content: center;
      }
      .bodycontrol {
        background-color: beige;
        display: inline-flex;
        justify-content: center;
        flex-direction: column;
      }
      .mastercontrol {
        display: inline-flex;
        flex-wrap: wrap;
        background-color: coral;
      }
      .heightcontrol {
        display: inline-flex;
        flex-direction: column;
        flex-wrap: wrap;
      }
    </style>
  </head>
  <body>
    <div class="main">
      <div class="video">
        <h1>LEGION</h1>
        <img src="" id="photo" />
      </div>

      <div class="control-bar">
        <div class="control-bar-field">
          <button
            class="button"
            id="driveModeButton"
            onclick="setMode('drive')"
          >
            DRIVE MODE
          </button>
        </div>
        <div class="control-bar-field">
          <button class="button" id="walkModeButton" onclick="setMode('walk')">
            WALK MODE
          </button>
        </div>
        <div class="control-bar-field">
          <div class="mode-text">
            Current mode is <span id="currentMode">walk</span>
          </div>
        </div>
      </div>
      <div class="control-panel">
        <div class="bodycontrol">
          <label for="zdd" class="slider-label">z-axis:</label>
          <input
            type="range"
            id="zdd"
            class="slider"
            data-axis="zd"
            min="-100"
            max="0"
            value="-50"
            oninput="updateSliderValue(this)"
          />
          <span id="zd" class="slider-value">-50</span>
          <br />

          <label for="ydd" class="slider-label">y-axis:</label>
          <input
            type="range"
            id="ydd"
            class="slider"
            data-axis="yd"
            min="10"
            max="100"
            value="62"
            oninput="updateSliderValue(this)"
          />
          <span id="yd" class="slider-value">62</span>
          <br />

          <label for="xdd" class="slider-label">x-axis:</label>
          <input
            type="range"
            id="xdd"
            class="slider"
            data-axis="xd"
            min="10"
            max="100"
            value="62"
            oninput="updateSliderValue(this)"
          />
          <span id="xd" class="slider-value">62</span>
          <br />

          <label for="lss" class="slider-label">Leg Speed:</label>
          <input
            type="range"
            id="lss"
            class="slider"
            data-axis="ls"
            min="4"
            max="12"
            value="8"
            oninput="updateSliderValue(this)"
          />
          <span id="ls" class="slider-value">8</span>
          <br />
          <label for="sss" class="slider-label">Movement Speed:</label>
          <input
            type="range"
            id="sss"
            class="slider"
            data-axis="ss"
            min="30"
            max="100"
            value="60"
            oninput="updateSliderValue(this)"
          />
          <span id="ss" class="slider-value">60</span>
          <br />

          <button class="button" onclick="resetAllValues()">Reset</button>
        </div>

        <div class="motion-control">
          <table>
            <tr>
              <td colspan="3" align="center">
                <button
                  class="button"
                  onmousedown="toggleCheckbox('move','f');"
                  ontouchstart="toggleCheckbox('move','f');"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  Forward
                </button>
              </td>
            </tr>
            <tr>
              <td align="center">
                <button
                  class="button"
                  onmousedown="toggleCheckbox('move','l');"
                  ontouchstart="toggleCheckbox('move','l');"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  Left
                </button>
              </td>
              <td align="center">
                <button
                  class="button"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  Stop
                </button>
              </td>
              <td align="center">
                <button
                  class="button"
                  onmousedown="toggleCheckbox('move','r');"
                  ontouchstart="toggleCheckbox('move','r');"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  Right
                </button>
              </td>
            </tr>
            <tr>
              <td colspan="3" align="center">
                <button
                  class="button"
                  onmousedown="toggleCheckbox('move','b');"
                  ontouchstart="toggleCheckbox('move','b');"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  Backward
                </button>
              </td>
            </tr>
          </table>
        </div>

        <div class="drive-motion-control">
          <table>
            <tr>
              <td colspan="3" align="center">
                <button
                  class="button"
                  onmousedown="toggleCheckbox('move','w');"
                  ontouchstart="toggleCheckbox('move','w');"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  M-Forward
                </button>
              </td>
            </tr>
            <tr>
              <td align="center">
                <button
                  class="button"
                  onmousedown="toggleCheckbox('move','a');"
                  ontouchstart="toggleCheckbox('move','a');"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  M-Left
                </button>
              </td>
              <td align="center">
                <button
                  class="button"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  Stop
                </button>
              </td>
              <td align="center">
                <button
                  class="button"
                  onmousedown="toggleCheckbox('move','d');"
                  ontouchstart="toggleCheckbox('move','d');"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  M-Right
                </button>
              </td>
            </tr>
            <tr>
              <td colspan="3" align="center">
                <button
                  class="button"
                  onmousedown="toggleCheckbox('move','x');"
                  ontouchstart="toggleCheckbox('move','x');"
                  onmouseup="toggleCheckbox('move','s');"
                  ontouchend="toggleCheckbox('move','s');"
                >
                  M-Backward
                </button>
              </td>
            </tr>
          </table>
        </div>

        <div class="mastercontrol">
          <div class="heightcontrol">
            <h6>Height Control</h6>
            <div class="heightcontrol1">
              <button class="button" onclick="adjustheight('u');">
                UP 
              </button>
            </div>
            <div class="heightcontrol2">
              <span id="heightspan" class="slider-value">50</span>
              <br />
            </div>
            <div class="heightcontrol3">
              <button class="button" onclick="adjustheight('v');">
                Down
              </button>
            </div>
          </div>
          
        </div>
      </div>
    </div>
    <script>
      function toggleCheckbox(action, value) {
        var xhr = new XMLHttpRequest();
        // Construct the URL with action and value
        xhr.open("GET", `/action?go=${action}&value=${value}`, true);
        xhr.send();
      }

      window.onload = document.getElementById("photo").src =
        window.location.href.slice(0, -1) + ":81/stream";

      let currentMode = null;


      function adjustheight(val) {
        toggleCheckbox("height", val);
        // Get the heightspan element
        const heightSpan = document.getElementById("heightspan");
      
        // Parse the current value as an integer
        let currentValue = parseInt(heightSpan.textContent, 10);
      
        // Adjust the value based on the input
        if (val === 'u') {
          currentValue += 5; // Increase value by 5
        } else if (val === 'v') {
          currentValue -= 5; // Decrease value by 5
    
        }
        // Update the textContent of the heightSpan
        heightSpan.textContent = currentValue;
      }
      

      function updateSliderValue(axis, value) {
        document.getElementById(axis).textContent = value;
        toggleCheckbox(axis, value);
      }

      function updateSliderValue(slider) {
        const axis = slider.getAttribute("data-axis");
        const value = slider.value;

        // Update the span text
        document.getElementById(axis).textContent = value;

        // Send value to toggleCheckbox
        toggleCheckbox(axis, value);
      }

      function resetAllValues() {
        // Define default values for sliders
        const defaults = {
          xd: 62,
          yd: 62,
          zd: -50,
          ls: 8,
          ss: 60,
        };

        // Loop through defaults and reset sliders and spans
        Object.keys(defaults).forEach((axis) => {
          const slider = document.querySelector(`input[data-axis="${axis}"]`);
          const defaultValue = defaults[axis];

          // Update slider and span values
          slider.value = defaultValue;
          document.getElementById(axis).textContent = defaultValue;

          // Send reset value to toggleCheckbox
          toggleCheckbox(axis, defaultValue);
        });
      }

      function setMode(mode) {
        toggleCheckbox("mode", mode);
        // Set the new mode
        currentMode = mode;
        document.getElementById("currentMode").textContent = mode;

        // Update button styles based on the active mode
        document
          .getElementById("driveModeButton")
          .classList.toggle("active", currentMode === "Drive");
        document
          .getElementById("walkModeButton")
          .classList.toggle("active", currentMode === "Walk");
      }
    </script>
  </body>
</html>






)rawliteral";

static esp_err_t index_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t stream_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  while (true)
  {
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    }
    else
    {
      if (fb->width > 400)
      {
        if (fb->format != PIXFORMAT_JPEG)
        {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted)
          {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        }
        else
        {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK)
    {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
    {
      break;
    }
    // Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

// static esp_err_t cmd_handler(httpd_req_t *req){
//   char*  buf;
//   size_t buf_len;
//   char variable[32] = {0,};

//   buf_len = httpd_req_get_url_query_len(req) + 1;
//   if (buf_len > 1) {
//     buf = (char*)malloc(buf_len);
//     if(!buf){
//       httpd_resp_send_500(req);
//       return ESP_FAIL;
//     }
//     if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
//       if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
//       } else {
//         free(buf);
//         httpd_resp_send_404(req);
//         return ESP_FAIL;
//       }
//     } else {
//       free(buf);
//       httpd_resp_send_404(req);
//       return ESP_FAIL;
//     }
//     free(buf);
//   } else {
//     httpd_resp_send_404(req);
//     return ESP_FAIL;
//   }

//   sensor_t * s = esp_camera_sensor_get();
//   int res = 0;

//   if(!strcmp(variable, "forward")) {
//     Serial.println("f");
//   }
//   else if(!strcmp(variable, "left")) {
//     Serial.println("l");
//   }
//   else if(!strcmp(variable, "right")) {
//     Serial.println("r");
//   }
//   else if(!strcmp(variable, "backward")) {
//     Serial.println("b");
//   }
//   else if(!strcmp(variable, "stop")) {
//     Serial.println("s");
//   }
//   else {
//     res = -1;
//   }

//   if(res){
//     return httpd_resp_send_500(req);
//   }

//   httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
//   return httpd_resp_send(req, NULL, 0);
// }

static esp_err_t cmd_handler(httpd_req_t *req)
{
  char *buf;
  size_t buf_len;
  char action[32] = {0};
  char value[32] = {0};

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    buf = (char *)malloc(buf_len);
    if (!buf)
    {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      // Parse "go" (action) parameter
      if (httpd_query_key_value(buf, "go", action, sizeof(action)) == ESP_OK)
      {
        //Serial.printf("Action: %s\n", action);
      }

      // Parse "value" parameter
      if (httpd_query_key_value(buf, "value", value, sizeof(value)) == ESP_OK)
      {
        //Serial.printf("Value: %s\n", value);
      }
    }
    else
    {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  }
  else
  {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  // Handle specific actions and values
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(action, "xd")||!strcmp(action, "yd")||!strcmp(action, "zd")||!strcmp(action, "ls"))
  {
    // Perform height adjustment using the provided value
    Serial.print(action);
    Serial.print(",");
    Serial.println(value);
    // Add code to adjust hardware height based on the value
  }
  else if (!strcmp(action, "ss")){
    Serial.print(action);
    Serial.print(",");
    Serial.println(value);
  }
  else if (!strcmp(action, "height")){
    Serial.println(value);
  }

  else if (!strcmp(action, "mode"))
  {
    if (!strcmp(value, "drive"))
    {
      //Serial.println("drive mode activated");
      // drivemode = true;
      Serial.println("h");

    }
    else if (!strcmp(value, "walk"))
    {
      drivemode = false;
      //Serial.println("Walk mode activated");
    }
    else
    {
      res = -1; // Unknown mode
    }
  }

  else if (!strcmp(action, "move"))
  {
    // if (drivemode)
    // {
    //   // Prepend "M" to the value if drivemode is true
    //   Serial.print("m");
    // }
    // Print the value (e.g., F, B, L, R, S)
    Serial.println(value);
  }
  

  else
  {
    res = -1; // Unknown action
  }

  if (res)
  {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_handler,
      .user_ctx = NULL};

  httpd_uri_t cmd_uri = {
      .uri = "/action",
      .method = HTTP_GET,
      .handler = cmd_handler,
      .user_ctx = NULL};
  httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL};
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

  Serial.begin(115200);
  Serial.setDebugOutput(false);

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

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1); // Flip vertically
  s->set_hmirror(s, 1); // Flip horizontally
  
  // Wi-Fi connection
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("WiFi AP started");
  Serial.print("AP IP address: ");

  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.softAPIP());

  // Start streaming web server
  startCameraServer();
}

void loop()
{
}