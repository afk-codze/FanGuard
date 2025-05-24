#include "MPU6500-library.h"

extern MPU6500_WE my_mpu6500(MPU6500_ADDR); 
volatile bool motion_anomaly = false;

void IRAM_ATTR motion_isr() { 
  motion_anomaly = true;
}

void enable_motion_interrupt(uint8_t threshold) { 
  // Enable the Wake-on-Motion interrupt with the calculated threshold
  my_mpu6500.enableInterrupt(MPU6500_WOM_INT);
  my_mpu6500.setWakeOnMotionThreshold(threshold);
  my_mpu6500.enableWakeOnMotion(MPU6500_WOM_ENABLE, MPU6500_WOM_COMP_DISABLE);

  // Attach interrupt handler
  attachInterrupt(digitalPinToInterrupt(INT_PIN), motion_isr, RISING); // Using motion_isr

  Serial.print("Motion detection enabled with threshold: ");
  Serial.print(threshold);
  Serial.print(" (");
  Serial.print(threshold * 4);
  Serial.println(" mg)");
}

void acc_init(){ 
  if(!my_mpu6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
    my_mpu6500.enableAccDLPF(true);
    my_mpu6500.setAccRange(MPU6500_ACC_RANGE_2G);
    my_mpu6500.setAccDLPF(MPU6500_DLPF_2);
    my_mpu6500.setIntPinPolarity(MPU6500_ACT_HIGH);
    my_mpu6500.enableIntLatch(true);
    my_mpu6500.enableClearIntByAnyRead(true);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, HIGH);
  }
}

void build_baseline_mpu6500(void *args){ 
  Serial.println("Phase 1: Determining average baseline acceleration (gravity component)...");

  // First get the average readings to establish baseline with gravity component
  float sum_x = 0, sum_y = 0, sum_z = 0; // Renamed sumX, sumY, sumZ
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    xyzFloat accel = my_mpu6500.getGValues();
    sum_x += accel.x;
    sum_y += accel.y;
    sum_z += accel.z;

    if (i % 50 == 0) {
      Serial.print("Collecting baseline averages: ");
      Serial.print(i);
      Serial.print("/");
      Serial.println(BASELINE_SAMPLES);
    }
    vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD));
  }

  // Calculate average baseline (with gravity)
  float baseline_x = sum_x / BASELINE_SAMPLES; 
  float baseline_y = sum_y / BASELINE_SAMPLES; 
  float baseline_z = sum_z / BASELINE_SAMPLES; 

  Serial.println("\nBaseline average accelerations (with gravity):");
  Serial.print("X: ");
  Serial.print(baseline_x, 3);
  Serial.print("g, Y: ");
  Serial.print(baseline_y, 3);
  Serial.print("g, Z: ");
  Serial.print(baseline_z, 3);
  Serial.println("g");

  // Now transition to phase 2
  Serial.println("\nPhase 2: Measuring maximum deviations from baseline...");
  // Second phase: measure deviations from baseline
  float max_deviation = 0; 

  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    xyzFloat accel = my_mpu6500.getGValues();
    float dev_x = abs(accel.x - baseline_x); 
    float dev_y = abs(accel.y - baseline_y); 
    float dev_z = abs(accel.z - baseline_z); 

    float max_dev_for_sample = dev_x > dev_y ? (dev_x > dev_z ? dev_x : dev_z) : (dev_y > dev_z ? dev_y : dev_z);
    max_deviation = max_deviation > max_dev_for_sample ? max_deviation : max_dev_for_sample;


    if (i % 50 == 0) {
      Serial.print("Measuring deviations: ");
      Serial.print(i);
      Serial.print("/");
      Serial.print(BASELINE_SAMPLES);
      Serial.print(" | Current max deviation: ");
      Serial.print(max_deviation, 3);
      Serial.println("g");
    }
    vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD));
  }

  // Apply safety margin to maximum observed deviation
  float threshold_g = max_deviation * THRESHOLD_MULTIPLIER; 

  // Ensure minimum threshold is at least 0.063g (16 units) for reliable detection
  threshold_g = (threshold_g < 0.063f) ? 0.063f : threshold_g;
  // Convert to MPU6500 threshold units (4mg per unit)
  uint8_t raw_threshold = (uint8_t)(threshold_g * 1000.0f / 4.0f);

  // Convert to MPU6500 threshold units (4mg per unit)
  calculated_threshold = (raw_threshold > 255) ? 255 : raw_threshold; 
  calculated_threshold = (calculated_threshold < 16) ? 16 : calculated_threshold;


  Serial.println("\nBaseline establishment complete!");
  Serial.println("--------------------------------");
  Serial.print("Maximum observed deviation: ");
  Serial.print(max_deviation, 3);
  Serial.println("g");
  Serial.print("Calculated threshold: ");
  Serial.print(threshold_g, 3);
  Serial.print("g (");
  Serial.print(calculated_threshold);
  Serial.print(" units, ");
  Serial.print(calculated_threshold * 4);
  Serial.println(" mg)");



  xTaskNotifyGive((TaskHandle_t)args);
  vTaskDelete(NULL);
}