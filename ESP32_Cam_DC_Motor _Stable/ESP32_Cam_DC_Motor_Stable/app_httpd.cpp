// app_httpd.cpp - Complete working version

#include "globals.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

// Add these extern declarations for auto calibration
extern bool auto_calibration_active;
extern void startAutomaticCalibration();
extern void stopAutomaticCalibration();

typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.printf("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

  size_t fb_len = 0;
  if (fb->format == PIXFORMAT_JPEG) {
    fb_len = fb->len;
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    jpg_chunking_t jchunk = { req, 0 };
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
    fb_len = jchunk.len;
  }
  esp_camera_fb_return(fb);
  int64_t fr_end = esp_timer_get_time();
  Serial.printf("JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.printf("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.printf("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
  }

  last_frame = 0;
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf;
  size_t buf_len;
  char variable[32] = {
    0,
  };
  char value[32] = {
    0,
  };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK && httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int val = atoi(value);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
  } else if (!strcmp(variable, "quality")) res = s->set_quality(s, val);
  else if (!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
  else if (!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
  else if (!strcmp(variable, "saturation")) res = s->set_saturation(s, val);
  else if (!strcmp(variable, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)val);
  else if (!strcmp(variable, "colorbar")) res = s->set_colorbar(s, val);
  else if (!strcmp(variable, "awb")) res = s->set_whitebal(s, val);
  else if (!strcmp(variable, "agc")) res = s->set_gain_ctrl(s, val);
  else if (!strcmp(variable, "aec")) res = s->set_exposure_ctrl(s, val);
  else if (!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
  else if (!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
  else if (!strcmp(variable, "awb_gain")) res = s->set_awb_gain(s, val);
  else if (!strcmp(variable, "agc_gain")) res = s->set_agc_gain(s, val);
  else if (!strcmp(variable, "aec_value")) res = s->set_aec_value(s, val);
  else if (!strcmp(variable, "aec2")) res = s->set_aec2(s, val);
  else if (!strcmp(variable, "dcw")) res = s->set_dcw(s, val);
  else if (!strcmp(variable, "bpc")) res = s->set_bpc(s, val);
  else if (!strcmp(variable, "wpc")) res = s->set_wpc(s, val);
  else if (!strcmp(variable, "raw_gma")) res = s->set_raw_gma(s, val);
  else if (!strcmp(variable, "lenc")) res = s->set_lenc(s, val);
  else if (!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
  else if (!strcmp(variable, "wb_mode")) res = s->set_wb_mode(s, val);
  else if (!strcmp(variable, "ae_level")) res = s->set_ae_level(s, val);
  else {
    res = -1;
  }

  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';

  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

// Manual calibrate handler
static esp_err_t calibrate_handler(httpd_req_t *req) {
  Serial.println("[DEBUG] Manual Calibrate button pressed");

  extern void calibrateIMU();
  calibrateIMU();

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "Manual calibration complete", HTTPD_RESP_USE_STRLEN);
}

// Automatic calibrate handler
static esp_err_t auto_calibrate_handler(httpd_req_t *req) {
  Serial.println("[DEBUG] Automatic Calibration button pressed");

  // Check if calibration is already active
  if (auto_calibration_active) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Calibration already in progress", HTTPD_RESP_USE_STRLEN);
  }

  // Start the automatic calibration sequence
  startAutomaticCalibration();

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "Automatic calibration started", HTTPD_RESP_USE_STRLEN);
}

// Emergency stop handler
static esp_err_t emergency_stop_handler(httpd_req_t *req) {
  Serial.println("[DEBUG] Emergency stop requested");

  stopAutomaticCalibration();

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "Emergency stop executed", HTTPD_RESP_USE_STRLEN);
}

// Update the status handler to get gyro-based positions
static esp_err_t gyro_status_handler(httpd_req_t *req) {
  char buffer[64];
  // Use the IMU-based positions directly
  extern float imu_pan_position;
  extern float imu_tilt_position;
  
  snprintf(buffer, sizeof(buffer), "{\"pan_angle\": %.1f, \"tilt_angle\": %.1f}", 
           imu_pan_position, imu_tilt_position);

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buffer, HTTPD_RESP_USE_STRLEN);
}

// Motor control handlers
#define MOTOR_ROUTE(uri_path, flag_var, state, message) \
  static esp_err_t uri_path##_handler(httpd_req_t *req) { \
    Serial.println(message); \
    flag_var = state; \
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); \
    httpd_resp_set_type(req, "text/plain"); \
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN); \
  }

MOTOR_ROUTE(pan_left, panLeft, true, "Pan Left START");
MOTOR_ROUTE(pan_right, panRight, true, "Pan Right START");
MOTOR_ROUTE(tilt_up, tiltUp, true, "Tilt Up START");
MOTOR_ROUTE(tilt_down, tiltDown, true, "Tilt Down START");

// Stop handlers
static esp_err_t pan_stop_handler(httpd_req_t *req) {
  Serial.println("Pan STOP");
  panLeft = false;
  panRight = false;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t tilt_stop_handler(httpd_req_t *req) {
  Serial.println("Tilt STOP");
  tiltUp = false;
  tiltDown = false;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

// Speed control handler
static esp_err_t set_speed_handler(httpd_req_t *req) {
  Serial.println("[DEBUG] set_speed_handler called");

  char query[128];
  char param[16];

  size_t query_len = httpd_req_get_url_query_len(req);
  if (query_len == 0) {
    Serial.println("[ERROR] No query string found");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Missing value parameter", HTTPD_RESP_USE_STRLEN);
  }

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (httpd_query_key_value(query, "value", param, sizeof(param)) == ESP_OK) {
      int value = atoi(param);
      MotorSpeed = constrain(value, 0, 255);

      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      httpd_resp_set_type(req, "text/plain");

      char response[32];
      snprintf(response, sizeof(response), "Speed set to %d", MotorSpeed);

      Serial.printf("[DEBUG] Motor speed updated to: %d\n", MotorSpeed);
      return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    } else {
      Serial.println("[ERROR] 'value' parameter not found in query string");
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      httpd_resp_set_type(req, "text/plain");
      return httpd_resp_send(req, "Missing 'value' parameter", HTTPD_RESP_USE_STRLEN);
    }
  } else {
    Serial.println("[ERROR] Failed to get query string");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Failed to parse query", HTTPD_RESP_USE_STRLEN);
  }
}

// Index handler with working buttons
static esp_err_t index_handler(httpd_req_t *req) {
  visitCount++;
  Serial.printf("User visited index page. Total visits: %d\n", visitCount);
  httpd_resp_set_type(req, "text/html");

  String page = "";

  // HTML structure with responsive CSS
  page += "<!DOCTYPE html><html><head>";
  page += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0\">";
  page += "<title>EcoRig Pan-Tilt Control</title>";
  page += "<style>";

  // Base styles
  page += "* { box-sizing: border-box; margin: 0; padding: 0; }";
  page += "body { font-family: Arial, sans-serif; background: #f0f0f0; }";
  page += "button { user-select: none; -webkit-user-select: none; -webkit-tap-highlight-color: rgba(0,0,0,0); touch-action: manipulation; font-size: 16px; border: none; cursor: pointer; transition: all 0.2s ease; }";

  // Calibration button styles
  page += ".calibrate-btn { background-color: #ff6b6b; color: white; border: none; padding: 10px 20px; border-radius: 5px; margin: 5px; }";
  page += ".calibrate-btn:hover { background-color: #ff5252; transform: translateY(-1px); box-shadow: 0 4px 8px rgba(0,0,0,0.2); }";
  page += ".calibrate-btn:active { background-color: #ff3838; transform: translateY(0); box-shadow: 0 2px 4px rgba(0,0,0,0.2); }";

  // Auto calibration button styles
  page += ".auto-calibrate-btn { background-color: #9c27b0; color: white; border: none; padding: 10px 20px; border-radius: 5px; margin: 5px; }";
  page += ".auto-calibrate-btn:hover { background-color: #7b1fa2; transform: translateY(-1px); box-shadow: 0 4px 8px rgba(0,0,0,0.2); }";
  page += ".auto-calibrate-btn:active { background-color: #6a1b9a; transform: translateY(0); box-shadow: 0 2px 4px rgba(0,0,0,0.2); }";
  page += ".auto-calibrate-btn:disabled { background-color: #ccc; cursor: not-allowed; transform: none; box-shadow: none; }";

  // Emergency stop button
  page += ".emergency-btn { background-color: #f44336; color: white; border: none; padding: 8px 16px; border-radius: 5px; margin: 5px; font-size: 14px; }";
  page += ".emergency-btn:hover { background-color: #d32f2f; }";

  // Control button styles
  page += ".control-btn { transition: all 0.2s ease; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  page += ".control-btn:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.2); }";
  page += ".control-btn:active { transform: translateY(0); box-shadow: 0 2px 6px rgba(0,0,0,0.3); filter: brightness(0.9); }";
  page += ".control-btn:disabled { background-color: #ccc !important; cursor: not-allowed; transform: none; box-shadow: none; }";
  page += ".up-btn:hover:not(:disabled) { background: #45a049 !important; }";
  page += ".down-btn:hover:not(:disabled) { background: #45a049 !important; }";
  page += ".left-btn:hover:not(:disabled) { background: #1976D2 !important; }";
  page += ".right-btn:hover:not(:disabled) { background: #1976D2 !important; }";

  // Mobile-first styles
  page += ".container { display: flex; flex-direction: column; min-height: 100vh; padding: 10px; }";
  page += ".video-section { flex: 1; margin-bottom: 20px; text-align: center; }";
  page += ".controls-section { flex: 1; }";
  page += ".video-stream { width: 100%; max-width: 400px; transform: rotate(180deg); border-radius: 10px; }";

  // Desktop styles (screens wider than 768px)
  page += "@media (min-width: 768px) {";
  page += "  .container { flex-direction: row; padding: 20px; gap: 30px; }";
  page += "  .video-section { flex: 1; margin-bottom: 0; display: flex; align-items: center; justify-content: center; }";
  page += "  .controls-section { flex: 1; min-width: 400px; }";
  page += "  .video-stream { width: 100%; max-width: 600px; height: auto; }";
  page += "}";

  // Large desktop styles (screens wider than 1200px)
  page += "@media (min-width: 1200px) {";
  page += "  .container { max-width: 1400px; margin: 0 auto; }";
  page += "  .video-stream { max-width: 800px; }";
  page += "}";

  page += "</style>";
  page += "</head><body>";

  // Container start
  page += "<div class=\"container\">";

  // Video Section
  page += "<div class=\"video-section\">";
  page += "<img class=\"video-stream\" src='http://" + WiFiAddr + ":81/stream' alt='Live Stream'>";
  page += "</div>";

  // Controls Section
  page += "<div class=\"controls-section\">";

  // Calibration buttons section
  page += "<div style='text-align:center; margin-bottom: 20px;'>";
  page += "<button class='calibrate-btn' onclick=\"manualCalibrate()\">Manual Calibration</button>";
  page += "<button class='auto-calibrate-btn' id='autoCalBtn' onclick=\"startAutoCalibration()\">Automatic Calibration</button>";
  page += "<button class='emergency-btn' onclick=\"emergencyStop()\">Emergency Stop</button>";
  page += "<p style='font-size: 12px; color: #666; margin-top: 5px;'>Manual: Sets current position as reference | Auto Discovery: Min-Max-Center sequence (6 seconds)</p>";
  page += "<p id='autoCalStatus' style='font-size: 14px; color: #9c27b0; margin-top: 5px; font-weight: bold; display: none;'>Automatic calibration in progress...</p>";
  page += "</div>";

  // Movement controls
  page += "<div style='text-align:center; margin-bottom: 30px;'>";
  page += "<button class='control-btn up-btn' id='upBtn' style='width:90px;height:80px; margin: 5px; background: #4CAF50; color: white; border-radius: 10px;' onmousedown=\"startMove('/tilt_up')\" onmouseup=\"stopMove('/tilt_stop')\" ontouchstart=\"startMove('/tilt_up')\" ontouchend=\"stopMove('/tilt_stop')\">Up</button><br/>";
  page += "<button class='control-btn left-btn' id='leftBtn' style='width:90px;height:80px; margin: 5px; background: #2196F3; color: white; border-radius: 10px;' onmousedown=\"startMove('/pan_left')\" onmouseup=\"stopMove('/pan_stop')\" ontouchstart=\"startMove('/pan_left')\" ontouchend=\"stopMove('/pan_stop')\">Left</button>";
  page += "<button class='control-btn right-btn' id='rightBtn' style='width:90px;height:80px; margin: 5px; background: #2196F3; color: white; border-radius: 10px;' onmousedown=\"startMove('/pan_right')\" onmouseup=\"stopMove('/pan_stop')\" ontouchstart=\"startMove('/pan_right')\" ontouchend=\"stopMove('/pan_stop')\">Right</button><br/>";
  page += "<button class='control-btn down-btn' id='downBtn' style='width:90px;height:80px; margin: 5px; background: #4CAF50; color: white; border-radius: 10px;' onmousedown=\"startMove('/tilt_down')\" onmouseup=\"stopMove('/tilt_stop')\" ontouchstart=\"startMove('/tilt_down')\" ontouchend=\"stopMove('/tilt_stop')\">Down</button>";
  page += "</div>";

  // Position visualization
  page += "<div style=\"text-align:center; margin-bottom: 30px;\">";
  page += "<div style=\"position: relative; width: 150px; height: 150px; margin: auto; background: white; border-radius: 50%; border: 2px solid #ddd; box-shadow: 0 4px 8px rgba(0,0,0,0.1);\">";
  
  // Pan Arrow - FIXED: Pivot from center of circle
  page += "<div id=\"panArrow\" style=\"position: absolute; top: 45px; left: 73px; width: 4px; height: 30px; background: black; transform-origin: bottom center; transform: rotate(0deg); transition: transform 0.3s ease-in-out;\">";
  page += "<div style=\"position: absolute; bottom: 100%; left: 50%; width: 0; height: 0; border-left: 6px solid transparent; border-right: 6px solid transparent; border-bottom: 10px solid black; transform: translateX(-50%);\"></div>";
  page += "</div>";

  // Tilt Line
  page += "<div id=\"tiltLine\" style=\"position: absolute; top: 75px; left: 75px; width: 70px; height: 3px; background: red; transform-origin: left center; transform: rotate(0deg); transition: transform 0.3s ease-in-out;\"></div>";

  // Tilt limit indicators
  page += "<div style=\"position: absolute; top: 75px; left: 75px; width: 70px; height: 1px; background: rgba(255,0,0,0.3); transform-origin: left center; transform: rotate(25deg);\"></div>";
  page += "<div style=\"position: absolute; top: 75px; left: 75px; width: 70px; height: 1px; background: rgba(255,0,0,0.3); transform-origin: left center; transform: rotate(-25deg);\"></div>";
  page += "<div style=\"position: absolute; top: 75px; left: 75px; width: 70px; height: 1px; background: rgba(255,0,0,0.2); border-top: 1px dotted rgba(255,0,0,0.4); transform-origin: left center; transform: rotate(30deg);\"></div>";
  page += "<div style=\"position: absolute; top: 75px; left: 75px; width: 70px; height: 1px; background: rgba(255,0,0,0.2); border-top: 1px dotted rgba(255,0,0,0.4); transform-origin: left center; transform: rotate(-30deg);\"></div>";

  // Pan limit indicators
  page += "<div style=\"position: absolute; top: 50%; left: 50%; width: 75px; height: 2px; background: rgba(255,165,0,0.5); transform-origin: left center; transform: rotate(-90deg);\"></div>";
  page += "<div style=\"position: absolute; top: 50%; left: 50%; width: 75px; height: 2px; background: rgba(255,165,0,0.5); transform-origin: left center; transform: rotate(90deg);\"></div>";
  page += "<div style=\"position: absolute; top: 50%; left: 50%; width: 75px; height: 1px; border-top: 2px dotted rgba(255,0,0,0.3); transform-origin: left center; transform: rotate(-180deg);\"></div>";
  page += "<div style=\"position: absolute; top: 50%; left: 50%; width: 75px; height: 1px; border-top: 2px dotted rgba(255,0,0,0.3); transform-origin: left center; transform: rotate(180deg);\"></div>";

  // Degree labels
  page += "<div style=\"position: absolute; top: 0; left: 50%; transform: translateX(-50%); font-size: 14px;\">0&deg;</div>";
  page += "<div style=\"position: absolute; top: 50%; right: 0; transform: translateY(-50%); font-size: 14px;\">90&deg;</div>";
  page += "<div style=\"position: absolute; bottom: 0; left: 50%; transform: translateX(-50%); font-size: 14px;\">180&deg;</div>";
  page += "<div style=\"position: absolute; top: 50%; left: 0; transform: translateY(-50%); font-size: 14px;\">270&deg;</div>";
  
  // Tilt labels
  page += "<div style=\"position: absolute; top: 50px; right: -15px; font-size: 10px; color: red;\">+25&deg;</div>";
  page += "<div style=\"position: absolute; bottom: 50px; right: -15px; font-size: 10px; color: red;\">-25&deg;</div>";
  page += "<div style=\"position: absolute; top: 72px; right: -10px; font-size: 10px; color: red;\">0&deg;</div>";
  page += "<div style=\"position: absolute; top: 42px; right: -20px; font-size: 8px; color: #999;\">+30&deg;</div>";
  page += "<div style=\"position: absolute; bottom: 42px; right: -20px; font-size: 8px; color: #999;\">-30&deg;</div>";
  
  // Center dot
  page += "<div style=\"position: absolute; top: 50%; left: 50%; width: 4px; height: 4px; background: black; border-radius: 50%; transform: translate(-50%, -50%);\"></div>";
  
  page += "</div>"; // End visualization circle

  // Status displays
  page += "<p id=\"panDisplay\" style=\"font-size: 18px; margin-top: 10px; font-weight: bold;\">Pan: 0.0&deg;</p>";
  page += "<p id=\"tiltDisplay\" style=\"font-size: 16px; color: #666; margin-bottom: 10px;\">Tilt: 0.0&deg;</p>";
  page += "<p style=\"font-size: 12px; color: #999;\">Software Limits: Pan +/-90&deg; | Tilt +/-25&deg;</p>";
  page += "<p style=\"font-size: 10px; color: #aaa;\">Physical Limits: Pan +/-180&deg; | Tilt +/-30&deg;</p>";
  page += "<p style=\"font-size: 10px; color: #aaa;\">Black Arrow: Pan | Red Line: Tilt</p>";
  page += "</div>"; // End visualization section
  
  // Speed control
  page += "<div style=\"text-align:center; margin-top: 20px; padding: 20px; background: white; border-radius: 10px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);\">";
  page += "<label for=\"speedSlider\" style=\"font-size:16px; font-weight: bold;\">Motor Speed:</label><br/>";
  page += "<input type=\"range\" id=\"speedSlider\" min=\"0\" max=\"255\" value=\"200\" style=\"width: 80%; margin: 10px 0;\" oninput=\"updateSpeed(this.value)\">";
  page += "<p id=\"speedValue\" style=\"font-size: 18px; font-weight: bold; color: #333;\">200</p>";
  page += "</div>";

  // Close sections
  page += "</div>";  // controls-section
  page += "</div>";  // container

  // JavaScript - FIXED FUNCTIONS
  page += "<script>";
  page += "console.log('EcoRig Control Interface Loaded');";
  page += "var xhttp = new XMLHttpRequest();";
  page += "function send(p){xhttp.open('GET', p, true);xhttp.send();}";

  // Fixed AJAX function
  page += "function sendRequest(url) {";
  page += "  console.log('Sending request to:', url);";
  page += "  fetch(url)";
  page += "    .then(response => response.text())";
  page += "    .then(data => console.log('Response:', data))";
  page += "    .catch(error => console.error('Error:', error));";
  page += "}";

  // Movement functions
  page += "function startMove(endpoint) {";
  page += "  console.log('Starting movement:', endpoint);";
  page += "  sendRequest(endpoint);";
  page += "}";

  page += "function stopMove(endpoint) {";
  page += "  console.log('Stopping movement:', endpoint);";
  page += "  sendRequest(endpoint);";
  page += "}";

  // Calibration functions
  page += "function manualCalibrate() {";
  page += "  console.log('Manual calibration requested');";
  page += "  sendRequest('/calibrate');";
  page += "  alert('Position calibrated!');";
  page += "}";

  page += "function emergencyStop() {";
  page += "  console.log('Emergency stop requested');";
  page += "  sendRequest('/emergency_stop');";
  page += "  autoCalActive = false;";
  page += "  document.getElementById('autoCalBtn').disabled = false;";
  page += "  document.getElementById('autoCalStatus').style.display = 'none';";
  page += "  disableControlButtons(false);";
  page += "  alert('Emergency stop executed!');";
  page += "}";

  // Auto calibration status tracking
  page += "var autoCalActive = false;";

  page += "function startAutoCalibration() {";
  page += "  if (autoCalActive) {";
  page += "    console.log('Auto calibration already active');";
  page += "    return;";
  page += "  }";
  page += "  console.log('Starting auto calibration');";
  page += "  sendRequest('/auto_calibrate');";
  page += "  autoCalActive = true;";
  page += "  document.getElementById('autoCalBtn').disabled = true;";
  page += "  document.getElementById('autoCalStatus').style.display = 'block';";
  page += "  disableControlButtons(true);";
  page += "  console.log('Auto calibration sequence: Pan(Left 0.5s → Right 1.0s → Left 0.5s) then Tilt(Up 0.5s → Down 1.0s → Up 0.5s)');";
  page += "  setTimeout(() => {";
  page += "    autoCalActive = false;";
  page += "    document.getElementById('autoCalBtn').disabled = false;";
  page += "    document.getElementById('autoCalStatus').style.display = 'none';";
  page += "    disableControlButtons(false);";
  page += "    alert('Automatic calibration completed!\\nSequence: Boundary Discovered for both axes\\nPosition set to center (0°, 0°)');";
  page += "  }, 6000);";
  page += "}";

  page += "function disableControlButtons(disable) {";
  page += "  const buttons = ['upBtn', 'downBtn', 'leftBtn', 'rightBtn'];";
  page += "  buttons.forEach(id => {";
  page += "    const btn = document.getElementById(id);";
  page += "    if (btn) btn.disabled = disable;";
  page += "  });";
  page += "}";

  page += "function updatePanDisplay() {";
  page += "  fetch('/gyro_status')";
  page += "    .then(res => res.json())";
  page += "    .then(data => {";
  page += "      const panAngle = parseFloat(data.pan_angle);";
  page += "      const tiltAngle = parseFloat(data.tilt_angle);";
  page += "      document.getElementById('panDisplay').innerHTML = 'Pan: ' + panAngle.toFixed(1) + '&deg;';";
  page += "      document.getElementById('tiltDisplay').innerHTML = 'Tilt: ' + tiltAngle.toFixed(1) + '&deg;';";
  page += "      document.getElementById('panArrow').style.transform = 'rotate(' + panAngle + 'deg)';";  // FIXED: Keep translateX(-50%) for centering
  page += "      document.getElementById('tiltLine').style.transform = 'rotate(' + tiltAngle + 'deg)';";
  page += "      const panArrow = document.getElementById('panArrow');";
  page += "      const panArrowhead = panArrow.querySelector('div');";
  page += "      if (Math.abs(panAngle) > 85) {";
  page += "        panArrow.style.background = 'red';";
  page += "        panArrowhead.style.borderBottomColor = 'red';";
  page += "      } else if (Math.abs(panAngle) > 75) {";
  page += "        panArrow.style.background = 'orange';";
  page += "        panArrowhead.style.borderBottomColor = 'orange';";
  page += "      } else {";
  page += "        panArrow.style.background = 'black';";
  page += "        panArrowhead.style.borderBottomColor = 'black';";
  page += "      }";
  page += "      const tiltLine = document.getElementById('tiltLine');";
  page += "      if (Math.abs(tiltAngle) > 23) {";
  page += "        tiltLine.style.background = 'darkred';";
  page += "        tiltLine.style.height = '4px';";
  page += "      } else if (Math.abs(tiltAngle) > 20) {";
  page += "        tiltLine.style.background = 'orange';";
  page += "        tiltLine.style.height = '3px';";
  page += "      } else {";
  page += "        tiltLine.style.background = 'red';";
  page += "        tiltLine.style.height = '3px';";
  page += "      }";
  page += "      const tiltDisplay = document.getElementById('tiltDisplay');";
  page += "      if (Math.abs(tiltAngle) > 23) {";
  page += "        tiltDisplay.style.color = 'red';";
  page += "      } else if (Math.abs(tiltAngle) > 20) {";
  page += "        tiltDisplay.style.color = 'orange';";
  page += "      } else {";
  page += "        tiltDisplay.style.color = '#666';";
  page += "      }";
  page += "    })";
  page += "    .catch(err => {";
  page += "      console.error('Error fetching gyro status:', err);";
  page += "      document.getElementById('panDisplay').innerHTML = 'Pan: Connection Error';";
  page += "      document.getElementById('tiltDisplay').innerHTML = 'Tilt: Connection Error';";
  page += "    });";
  page += "}";

  page += "function updateSpeed(val) {";
  page += "  document.getElementById('speedValue').innerText = val;";
  page += "  sendRequest('/set_speed?value=' + val);";
  page += "}";

  page += "// Start position updates";
  page += "setInterval(updatePanDisplay, 500);";
  page += "console.log('All functions initialized successfully');";
  page += "</script>";
  page += "</body></html>";

  return httpd_resp_send(req, page.c_str(), page.length());
}

// URI handler definitions - ALL DECLARED AFTER THE HANDLER FUNCTIONS
static httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
static httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
static httpd_uri_t pan_left_uri = { .uri = "/pan_left", .method = HTTP_GET, .handler = pan_left_handler, .user_ctx = NULL };
static httpd_uri_t pan_right_uri = { .uri = "/pan_right", .method = HTTP_GET, .handler = pan_right_handler, .user_ctx = NULL };
static httpd_uri_t pan_stop_uri = { .uri = "/pan_stop", .method = HTTP_GET, .handler = pan_stop_handler, .user_ctx = NULL };
static httpd_uri_t tilt_up_uri = { .uri = "/tilt_up", .method = HTTP_GET, .handler = tilt_up_handler, .user_ctx = NULL };
static httpd_uri_t tilt_down_uri = { .uri = "/tilt_down", .method = HTTP_GET, .handler = tilt_down_handler, .user_ctx = NULL };
static httpd_uri_t tilt_stop_uri = { .uri = "/tilt_stop", .method = HTTP_GET, .handler = tilt_stop_handler, .user_ctx = NULL };
static httpd_uri_t gyro_status_uri = { .uri = "/gyro_status", .method = HTTP_GET, .handler = gyro_status_handler, .user_ctx = NULL };
static httpd_uri_t set_speed_uri = { .uri = "/set_speed", .method = HTTP_GET, .handler = set_speed_handler, .user_ctx = NULL };
static httpd_uri_t calibrate_uri = { .uri = "/calibrate", .method = HTTP_GET, .handler = calibrate_handler, .user_ctx = NULL };
static httpd_uri_t auto_calibrate_uri = { .uri = "/auto_calibrate", .method = HTTP_GET, .handler = auto_calibrate_handler, .user_ctx = NULL };
static httpd_uri_t emergency_stop_uri = { .uri = "/emergency_stop", .method = HTTP_GET, .handler = emergency_stop_handler, .user_ctx = NULL };

// Server startup function
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  // IMPORTANT: Increase the maximum URI handlers to accommodate all endpoints
  config.max_uri_handlers = 20;
  config.max_resp_headers = 8;

  Serial.printf("[DEBUG] Config - max_uri_handlers: %d, max_resp_headers: %d\n",
                config.max_uri_handlers, config.max_resp_headers);

  ra_filter_init(&ra_filter, 20);
  Serial.printf("Starting web server on port: %d\n", config.server_port);

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    Serial.println("[DEBUG] Camera server started successfully");

    // Register all URI handlers
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &pan_left_uri);
    httpd_register_uri_handler(camera_httpd, &pan_right_uri);
    httpd_register_uri_handler(camera_httpd, &pan_stop_uri);
    httpd_register_uri_handler(camera_httpd, &tilt_up_uri);
    httpd_register_uri_handler(camera_httpd, &tilt_down_uri);
    httpd_register_uri_handler(camera_httpd, &tilt_stop_uri);
    httpd_register_uri_handler(camera_httpd, &gyro_status_uri);
    httpd_register_uri_handler(camera_httpd, &set_speed_uri);
    httpd_register_uri_handler(camera_httpd, &calibrate_uri);
    httpd_register_uri_handler(camera_httpd, &auto_calibrate_uri);
    httpd_register_uri_handler(camera_httpd, &emergency_stop_uri);

    Serial.println("[DEBUG] All URI handlers registered successfully");

  } else {
    Serial.println("[ERROR] Failed to start camera server");
  }

  // Start stream server
  config.server_port = 81;
  config.ctrl_port = 32769;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("[DEBUG] Stream server started on port 81");
  } else {
    Serial.println("[ERROR] Failed to start stream server");
  }
}

void setupLedFlash(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}