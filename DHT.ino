
float readTempAndHumid() {

  humi = dht.readHumidity();
  tempc = dht.readTemperature();
  tempf = dht.readTemperature(true);

  return tempc, tempf;
}
