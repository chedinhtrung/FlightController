#include "dps310.h"
#include "Wire.h"

void Dps310Altimeter::setup(){

    Wire.beginTransmission(0x76);
    // Configurations

    // High precision pressure measurement according to recommendation
    Wire.write(0x06);
    Wire.write(0x66);
    Wire.endTransmission();

    delay(100);

    Wire.beginTransmission(0x76);
    Wire.write(0x07);
    Wire.write(0xE6);
    Wire.endTransmission();

    delay(100);
    
    // enable T shift and P shift (mandatory)
    Wire.beginTransmission(0x76);
    Wire.write(0x09);
    Wire.write(0x0C);
    Wire.endTransmission();

    delay(100);

    // Set continuous measurement of temp and pressure
    Wire.beginTransmission(0x76);
    Wire.write(0x08);
    Wire.write(0x07);
    Wire.endTransmission();

    delay(300);

    get_calibrations();
    delay(300);
    get_initial();
    delay(100);
}

int32_t twosComplement(int32_t val, uint8_t bits) {
  if (val & ((uint32_t)1 << (bits - 1))) {
    val -= (uint32_t)1 << bits;
  }
  return val;
}

void Dps310Altimeter::get_calibrations(){
    Wire.beginTransmission(DPS310ADDR);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.requestFrom(DPS310ADDR, 18);

    uint8_t coeffs[18];
  for (uint8_t addr = 0; addr < 18; addr++) {
    coeffs[addr] = Wire.read();
  }
  c0 = ((uint16_t)coeffs[0] << 4) | (((uint16_t)coeffs[1] >> 4) & 0x0F);
  c0 = twosComplement(c0, 12);

  c1 = twosComplement((((uint16_t)coeffs[1] & 0x0F) << 8) | coeffs[2], 12);

  c00 = ((uint32_t)coeffs[3] << 12) | ((uint32_t)coeffs[4] << 4) |
         (((uint32_t)coeffs[5] >> 4) & 0x0F);
  c00 = twosComplement(c00, 20);

  c10 = (((uint32_t)coeffs[5] & 0x0F) << 16) | ((uint32_t)coeffs[6] << 8) |
         (uint32_t)coeffs[7];
  c10 = twosComplement(c10, 20);

  c01 = twosComplement(((uint16_t)coeffs[8] << 8) | (uint16_t)coeffs[9], 16);
  c11 = twosComplement(((uint16_t)coeffs[10] << 8) | (uint16_t)coeffs[11], 16);
  c20 = twosComplement(((uint16_t)coeffs[12] << 8) | (uint16_t)coeffs[13], 16);
  c21 = twosComplement(((uint16_t)coeffs[14] << 8) | (uint16_t)coeffs[15], 16);
  c30 = twosComplement(((uint16_t)coeffs[16] << 8) | (uint16_t)coeffs[17], 16);

  /*
  Serial.print("c0 = "); Serial.println(c0);
  Serial.print("c1 = "); Serial.println(c1);
  Serial.print("c00 = "); Serial.println(c00);
  Serial.print("c10 = "); Serial.println(c10);
  Serial.print("c01 = "); Serial.println(c01);
  Serial.print("c11 = "); Serial.println(c11);
  Serial.print("c20 = "); Serial.println(c20);
  Serial.print("c21 = "); Serial.println(c21);
  Serial.print("c30 = "); Serial.println(c30);
  */
}

void Dps310Altimeter::get_initial(){
    float sum = 0.0;
    initial = 0.0;
    for (int i = 0; i<20; i++){
        sum += read();
        delay(250);
    }
    initial = sum/20.0;
}

float Dps310Altimeter::read(){
    Wire.beginTransmission(DPS310ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(DPS310ADDR, 6);
    uint8_t data[6];
    for (int i = 0; i < 6; i++){
        data[i] = Wire.read();
    }

    int32_t p_r = (data[0] << 16) | (data[1] << 8) | data[2];
    int32_t t_r = (data[3] << 16) | (data[4] << 8) | data[5];

    int32_t p_raw = twosComplement(p_r, 24);
    int32_t t_raw = twosComplement(t_r, 24);

    float p_sc = ((float)p_raw)/1040384;
    float t_sc = ((float)t_raw)/1040384;
    float p_comp =
      (int32_t)c00 +
      p_sc * ((int32_t)c10 +
                p_sc * ((int32_t)c20 + p_sc * (int32_t)c30)) +
      t_sc * ((int32_t)c01 +
        p_sc * ((int32_t)c11 + p_sc * (int32_t)c21));
    float alt=44330*(1-pow(p_comp/101325.0, 1/5.255))*100 - initial;
    raw_alt = (int16_t)(alt * 10);
    return alt;
}