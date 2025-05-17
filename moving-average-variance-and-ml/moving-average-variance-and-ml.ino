#include <Wire.h>
#include <Adafruit_BME280.h>

// --- USER CONFIG ---
#define BME_SDA 42
#define BME_SCL 41
#define I2C_FREQ 100000L

// Rolling window size
#define WINDOW_SIZE  10
// Delay between samples, in ms
#define DELAY_MS     500

Adafruit_BME280 bme;

// Ring buffer for temperature
#define MAX_BUFFER_SIZE 50
float ringBuffer[MAX_BUFFER_SIZE];
int   bufferHead  = 0;
int   sampleCount = 0;

// Calculate rolling mean
float calcMean(int count) {
  float sum = 0;
  for (int i = 0; i < count; i++) {
    int idx = (bufferHead - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
    sum += ringBuffer[idx];
  }
  return (count > 0) ? (sum / count) : 0;
}

// Calculate rolling variance
float calcVariance(int count, float mean) {
  float sum = 0;
  for (int i = 0; i < count; i++) {
    int idx = (bufferHead - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
    float diff = ringBuffer[idx] - mean;
    sum += diff * diff;
  }
  // population variance
  return (count > 1) ? (sum / count) : 0;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=== Collecting Temperature & Variance Data ===");

  Wire.begin(BME_SDA, BME_SCL, I2C_FREQ);

  if (!bme.begin(0x76)) {
    if (!bme.begin(0x77)) {
      Serial.println("BME280 not found. Stopping.");
      while(true) { delay(100); }
    }
  }

  // Optional forced mode:
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_NONE,
                  Adafruit_BME280::SAMPLING_NONE,
                  Adafruit_BME280::FILTER_OFF);

  // Print header for CSV output
  // We'll also have a "label" column that you manually control from the serial monitor
  Serial.println("timestamp,temperature,variance,label");
}

void loop() {
  // If using forced mode, must trigger measurement each time
  bme.takeForcedMeasurement();
  float temp = bme.readTemperature();

  // Insert into ring buffer
  ringBuffer[bufferHead] = temp;
  bufferHead = (bufferHead + 1) % MAX_BUFFER_SIZE;
  if (sampleCount < MAX_BUFFER_SIZE) sampleCount++;

  // Compute rolling stats for min(WINDOW_SIZE, sampleCount) samples
  int currentWindow = (sampleCount < WINDOW_SIZE) ? sampleCount : WINDOW_SIZE;
  float mean    = calcMean(currentWindow);
  float variance = calcVariance(currentWindow, mean);

  // We'll label data as "normal" unless we do something special
  // (You can do something more dynamic; for demonstration, it's always "normal" here.)
  String label = "normal";
  
  if (variance > 0.7){
    label = "anomaly";
  }
  
  if (temp > 45.0){
     label = "anomaly"
  }

  // Get current timestamp in milliseconds (just approximate if no real-time clock)
  unsigned long timeNow = millis();

  // Print CSV row: timestamp, temperature, variance, label
  Serial.print(temp, 2);
  Serial.print(",");
  Serial.print(variance, 4);
  Serial.print(",");
  Serial.println(label);

  delay(DELAY_MS);
}
