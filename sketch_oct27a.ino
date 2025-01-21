#include <Servo.h>

// Servo objects for each servo in the quadruped
Servo SFR, FFR, TFR;  // Front Right
Servo SFL, FFL, TFL;  // Front Lefta
Servo SBR, FBR, TBR;  // Back Right
Servo SBL, FBL, TBL;  // Back Left

// Servo pins for Arduino Mega
const int SFR_PIN = 2, FFR_PIN = 3, TFR_PIN = 4;  // Front Right Pins
const int SFL_PIN = 5, FFL_PIN = 6, TFL_PIN = 7;  // Front Left Pins
const int SBR_PIN = 8, FBR_PIN = 9, TBR_PIN = 10; // Back Right Pins
const int SBL_PIN = 11, FBL_PIN = 12, TBL_PIN = 13; // Back Left Pins

// Step data for each leg - Servo angles in microseconds 
// targets = [[0,5,15],[0,0,15],[0,-5,15]]
//const int FR_steps[18][3] = {{1278.0, 832.0, 1558.0}, {1278.0, 818.0, 1544.0}, {1278.0, 804.0, 1529.0}, {1278.0, 791.0, 1514.0}, {1278.0, 778.0, 1498.0}, {1278.0, 767.0, 1482.0}, {1278.0, 756.0, 1465.0}, {1278.0, 746.0, 1448.0}, {1278.0, 737.0, 1430.0}, {1278.0, 693.0, 1456.0}, {1278.0, 651.0, 1477.0}, {1279.0, 610.0, 1494.0}, {1279.0, 636.0, 1534.0}, {1279.0, 665.0, 1573.0}, {1279.0, 697.0, 1611.0}, {1279.0, 732.0, 1647.0}, {1278.0, 763.0, 1619.0}, {1278.0, 796.0, 1590.0}};
//const int FL_steps[18][3] = {{1260.0, 1837.0, 1054.0}, {1259.0, 1880.0, 1029.0}, {1259.0, 1921.0, 1008.0}, {1259.0, 1962.0, 990.0}, {1259.0, 1937.0, 950.0}, {1259.0, 1908.0, 911.0}, {1259.0, 1876.0, 873.0}, {1259.0, 1841.0, 837.0}, {1259.0, 1811.0, 864.0}, {1259.0, 1778.0, 894.0}, {1260.0, 1743.0, 926.0}, {1260.0, 1757.0, 940.0}, {1260.0, 1771.0, 955.0}, {1260.0, 1783.0, 971.0}, {1260.0, 1796.0, 986.0}, {1260.0, 1807.0, 1003.0}, {1260.0, 1818.0, 1020.0}, {1260.0, 1828.0, 1037.0}};
//const int BR_steps[18][3] = {{1254.0, 702.0, 1363.0}, {1254.0, 659.0, 1389.0}, {1254.0, 619.0, 1410.0}, {1254.0, 580.0, 1427.0}, {1254.0, 604.0, 1468.0}, {1254.0, 632.0, 1507.0}, {1254.0, 663.0, 1545.0}, {1254.0, 697.0, 1582.0}, {1254.0, 727.0, 1554.0}, {1254.0, 759.0, 1524.0}, {1254.0, 793.0, 1492.0}, {1254.0, 779.0, 1477.0}, {1254.0, 766.0, 1463.0}, {1254.0, 753.0, 1447.0}, {1254.0, 741.0, 1431.0}, {1254.0, 730.0, 1415.0}, {1254.0, 720.0, 1398.0}, {1254.0, 710.0, 1381.0}};
//const int BL_steps[18][3] = {{1282.0, 1760.0, 1033.0}, {1282.0, 1775.0, 1047.0}, {1282.0, 1789.0, 1063.0}, {1282.0, 1802.0, 1079.0}, {1282.0, 1814.0, 1095.0}, {1282.0, 1826.0, 1112.0}, {1282.0, 1836.0, 1130.0}, {1282.0, 1846.0, 1148.0}, {1282.0, 1855.0, 1166.0}, {1282.0, 1899.0, 1140.0}, {1283.0, 1941.0, 1118.0}, {1283.0, 1982.0, 1099.0}, {1283.0, 1957.0, 1057.0}, {1283.0, 1928.0, 1017.0}, {1283.0, 1896.0, 977.0}, {1283.0, 1860.0, 939.0}, {1283.0, 1829.0, 968.0}, {1282.0, 1796.0, 999.0}};

void setup() {
  Serial.begin(500000);
  Serial.setTimeout(50); 

  // Attach servos
  SFR.attach(SFR_PIN);
  FFR.attach(FFR_PIN);
  TFR.attach(TFR_PIN);

  SFL.attach(SFL_PIN);
  FFL.attach(FFL_PIN);
  TFL.attach(TFL_PIN);

  SBR.attach(SBR_PIN);
  FBR.attach(FBR_PIN);
  TBR.attach(TBR_PIN);

  SBL.attach(SBL_PIN);
  FBL.attach(FBL_PIN);
  TBL.attach(TBL_PIN);

}

void loop() {
  // We expect 12 integers each time (SFR, FFR, TFR, SFL, FFL, TFL, SBR, FBR, TBR, SBL, FBL, TBL)
  while (Serial.available()) {
    // Read a line of text up to '\n'
    String line = Serial.readStringUntil('\n');
    // Example line: "1500,1450,1400,1600,1550,1500,1300,1250,1200,1400,1350,1300"

    // Parse the 12 integers
    int values[12];
    int index = 0;

    // Split the line by commas
    char * c_line = strdup(line.c_str());
    char * token = strtok(c_line, ",");
    while (token != NULL && index < 12) {
      values[index++] = atoi(token);
      token = strtok(NULL, ",");
    }
    free(c_line);

    // If we got exactly 12 values, update the servos
    if (index == 12) {
      // Assign them to each servo
      SFR.writeMicroseconds(values[0]);
      FFR.writeMicroseconds(values[1]);
      TFR.writeMicroseconds(values[2]);

      SFL.writeMicroseconds(values[3]);
      FFL.writeMicroseconds(values[4]);
      TFL.writeMicroseconds(values[5]);

      SBR.writeMicroseconds(values[6]);
      FBR.writeMicroseconds(values[7]);
      TBR.writeMicroseconds(values[8]);

      SBL.writeMicroseconds(values[9]);
      FBL.writeMicroseconds(values[10]);
      TBL.writeMicroseconds(values[11]);

      // Optional debug
      Serial.print("Updated servo positions: ");
      Serial.println(line);
    }
  }
}
