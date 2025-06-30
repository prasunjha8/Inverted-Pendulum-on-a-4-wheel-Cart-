#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <MPU6050.h>

// ---------------- MOTOR PINS ----------------
#define IN1 5   // D1
#define IN2 4   // D2
#define IN3 14  // D5
#define IN4 12  // D6
#define ENA 13  // D7
#define ENB 15  // D8

// ---------------- PID VARIABLES ----------------
float Kp = 9.8, Ki = 0.5, Kd = 4.0;
float setpoint = 90.0;
float prev_error = 0;
float integral = 0;

// ---------------- KALMAN VARIABLES ----------------
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
float angle = 0.0, bias = 0.0;
float P[2][2] = {{0, 0}, {0, 0}};

// ---------------- MPU6050 ----------------
MPU6050 mpu;
unsigned long lastTime = 0;

// ---------------- WiFi + Web ----------------
const char* ssid = "NO-INTERNET";
const char* password = "mypasswordismummy";
ESP8266WebServer server(80);

// ---------------- HTML Page ----------------
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>PID Tuner</title>
  <style>
    body { font-family: Arial; margin: 20px; text-align: center; }
    input { width: 80%%; }
    .slider-box { margin-bottom: 20px; }
  </style>
</head>
<body>
  <h2>Real-Time PID Controller</h2>
  <div class="slider-box">
    <label>Kp: <span id="KpVal">9.8</span></label><br>
    <input type="range" min="0" max="50" step="0.1" value="9.8" id="Kp" oninput="updateVal('Kp')">
  </div>
  <div class="slider-box">
    <label>Ki: <span id="KiVal">0.5</span></label><br>
    <input type="range" min="0" max="10" step="0.1" value="0.5" id="Ki" oninput="updateVal('Ki')">
  </div>
  <div class="slider-box">
    <label>Kd: <span id="KdVal">4.0</span></label><br>
    <input type="range" min="0" max="20" step="0.1" value="4.0" id="Kd" oninput="updateVal('Kd')">
  </div>

  <script>
    function updateVal(param) {
      let val = document.getElementById(param).value;
      document.getElementById(param + "Val").innerText = val;
      fetch("/set?" + param + "=" + val);
    }
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin(0, 2);  // D3 = GPIO0 (SDA), D4 = GPIO2 (SCL)
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  stopMotors();

  // WiFi Setup
  WiFi.softAP(ssid, password);
  Serial.println("WiFi started. Connect to: " + String(ssid));

  // Web Server Routes
  server.on("/", []() {
    server.send(200, "text/html", htmlPage);
  });

  server.on("/set", []() {
    if (server.hasArg("Kp")) Kp = server.arg("Kp").toFloat();
    if (server.hasArg("Ki")) Ki = server.arg("Ki").toFloat();
    if (server.hasArg("Kd")) Kd = server.arg("Kd").toFloat();
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  delay(1000);
  Serial.println("System ready. PID sliders available on web.");
}

void loop() {
  server.handleClient();

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0 || dt > 0.1) { lastTime = now; return; }

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accAngle = atan2(ax, ay) * 180.0 / PI;
  float gyroRate = gy / 131.0;

  angle = getKalmanAngle(accAngle, gyroRate, dt);

  float output = computePID(setpoint, angle, dt);
  output = constrain(output, -191, 191);

  int pwm = abs((int)output);
  int dir = (output > 0) ? 1 : (output < 0 ? -1 : 0);

  move(dir, pwm);

  Serial.print("Angle: "); Serial.print(angle, 2);
  Serial.print(" | Output: "); Serial.print(output, 1);
  Serial.print(" | Dir: "); Serial.print(dir);
  Serial.print(" | PWM: "); Serial.print(pwm);
  Serial.print(" | Kp: "); Serial.print(Kp);
  Serial.print(" | Ki: "); Serial.print(Ki);
  Serial.print(" | Kd: "); Serial.println(Kd);

  lastTime = now;
}

// ---------------- Kalman Filter ----------------
float getKalmanAngle(float newAngle, float newRate, float dt) {
  float rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle;
  angle += K[0] * y;
  bias  += K[1] * y;

  float P00_temp = P[0][0], P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

// ---------------- PID ----------------
float computePID(float setpoint, float currentAngle, float dt) {
  float error = setpoint - currentAngle;

  if (abs(error) < 0.5) {
    error = 0;
    integral = 0;
  }

  integral += error * dt;
  integral = constrain(integral, -50, 50);

  float derivative = (error - prev_error) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;

  prev_error = error;
  return output;
}

// ---------------- Motor Control ----------------
void move(int dir, int pwm) {
  pwm = constrain(pwm, 0, 191);

  if (dir == -1) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (dir == 1) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
