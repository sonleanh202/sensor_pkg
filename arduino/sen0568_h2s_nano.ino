const int SENSOR_PIN = A0;
float RAW_CLEAN = 967.5;
float RAW_50PPM = 1023.0;
float DEAD_BAND_ADC = 5.0;
bool SENSOR_INCREASES = true;

float readAverageRaw()
{
  long sum = 0;
  for (int i = 0; i < 30; i++) {
    sum += analogRead(SENSOR_PIN);
    delay(10);
  }
  return sum / 30.0;
}

float estimatePPM(float raw)
{
  float delta = SENSOR_INCREASES ? (raw - RAW_CLEAN) : (RAW_CLEAN - raw);

  if (delta < DEAD_BAND_ADC) {
    return 0.0;
  }

  float span = SENSOR_INCREASES ? (RAW_50PPM - RAW_CLEAN) : (RAW_CLEAN - RAW_50PPM);
  if (span <= 0.0) {
    return 0.0;
  }

  float ppm = delta * 50.0 / span;
  if (ppm < 0.0) ppm = 0.0;
  if (ppm > 50.0) ppm = 50.0;
  return ppm;
}

void setup()
{
  Serial.begin(9600);
  delay(1500);
}

void loop()
{
  float raw = readAverageRaw();
  float ppm = estimatePPM(raw);

  Serial.print("H2S,");
  Serial.print(raw, 1);
  Serial.print(",");
  Serial.println(ppm, 2);

  delay(500);
}
