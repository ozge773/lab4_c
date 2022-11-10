
void printComplementary(){

  float gyro_roll_vel = ((180/PI)*(g.x))-1; 
  // Filter implementation 
  int Fs = 5;
  int dt = 1;
  int decim = 1;
  //float gyro_roll = ((180/PI)*(g.x))-1;
  //float time = (0:decim:size(gyro_roll_vel,1)-1)/Fs;
  float gyro_roll = gyro_roll + (gyro_roll_vel * dt);

  if(gyro_roll >180) {
    gyro_roll =gyro_roll- 360;

  }
  else if(gyro_roll < -180){
    gyro_roll = gyro_roll +360;
  }

  // Raw accel Output

  float accell_roll= atan2(a.y, a.z) * -180/PI;

  float tau = 0.1;
  float alpha = tau / (tau +dt);
  float comp_roll = (alpha * gyro_roll )+ (1-alpha) * accell_roll;
  //Serial.print(a.y);
  Serial.print(comp_roll);

}
