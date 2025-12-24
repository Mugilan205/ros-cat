// Simple Test Sketch for Wheel Odometry
// This sends fake encoder ticks to verify wheel_odom.py works
// Upload to Arduino via Arduino IDE

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Arduino ready");
}

void loop() {
  // Send fake left_ticks,right_ticks every 100ms
  // x position will increment in ROS
  Serial.println("10,10");
  delay(100);
}
