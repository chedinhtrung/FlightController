#include "altimeter.h"
#include "Wire.h"

void Altimeter::setup_altimeter(){
    Wire.beginTransmission(ALTADDR);
    Wire.write(0xF4);               //ctrl_meas register
    Wire.write(0x57);               //temp oversampling 2, press. oversampling 16 (indoor nav guideline), mode normal
    Wire.endTransmission();

    Wire.beginTransmission(ALTADDR);
    Wire.write(0xF5);
    Wire.write(0x14);               // Standby 0.5ms, IIR coefficient 16
    Wire.endTransmission();

    get_calibrations();
    delay(100);
    get_startalt();
}

void Altimeter::get_calibrations(){
    uint8_t data[24], i=0;
    Wire.beginTransmission(ALTADDR);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(ALTADDR, 24);   
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    dig_T1 = (data[1] << 8) | data[0]; 
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6]; 
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22]; 
    delay(250);
}

float Altimeter::read(){
    Wire.beginTransmission(0x76);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(0x76,6);
    uint32_t press_msb = Wire.read();
    uint32_t press_lsb = Wire.read();
    uint32_t press_xlsb = Wire.read();
    uint32_t temp_msb = Wire.read();
    uint32_t temp_lsb = Wire.read();
    uint32_t temp_xlsb = Wire.read();
    unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4);
    unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);
    signed long int var1, var2;
    var1 = ((((adc_T >> 3) - ((signed long int )dig_T1 <<1)))* ((signed long int )dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T>>4) - ((signed long int )dig_T1)))>> 12) * ((signed long int )dig_T3)) >> 14;
    signed long int t_fine = var1 + var2;
    unsigned long int p;
    var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )dig_P6);
    var2 = var2 + ((var1*((signed long int )dig_P5)) <<1);
    var2 = (var2>>2)+(((signed long int )dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13))>>3)+((((signed long int )dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int )dig_P1)) >>15);
    if (var1 == 0) { p=0;}    
    p = (((unsigned long int )(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;
    if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
    else { p = (p / (unsigned long int )var1) * 2;  }
    var1 = (((signed long int )dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((signed long int )(p>>2)) * ((signed long int )dig_P8))>>13;
    p = (unsigned long int)((signed long int )p + ((var1 + var2+ dig_P7) >> 4));
    double pressure=(double)p/100;
    float alt=44330*(1-pow(pressure/1013.25, 1/5.255))*100 - startalt;
    return alt;
}

void Altimeter::get_startalt(){
    float start = 0;
    for (int i =0; i<100; i++){
        start += read();
        delay(40);
    }
    start /= 100;
    startalt = start;
}
