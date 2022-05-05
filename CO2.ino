
float MGRead(int mg_pin) {
  int i;
  float v = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    v += analogRead(mg_pin);
    delay(READ_SAMPLE_INTERVAL);
  }
  v = (v / READ_SAMPLE_TIMES) * 5 / 1024 ;
  return v;
}
int MGGetPercentage(float volts, float *pcurve) {
  if ((volts / DC_GAIN ) >= ZERO_POINT_VOLTAGE) {
    return -1;
  } else {
    return pow(10, ((volts / DC_GAIN) - pcurve[1]) / pcurve[2] + pcurve[0]);
  }
}

float readCO2(){
  
  volts = MGRead(MG_PIN);
  percentage = MGGetPercentage(volts,CO2Curve);
  return percentage;
}
