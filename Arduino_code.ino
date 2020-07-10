// DEF Bibliotèques
#include <SoftwareSerial.h>
SoftwareSerial mySerial(15, 14); //pin 14 connected to LCD, 15 unconnected

//DEF Entrees / Sorties
//analog pins
const int pTHROTTLE = A0; //slide pot
const int pTX = A1;       //translation x-axis
const int pTY = A2;       //translation y-axis
const int pTZ = A3;       //translation z-axis
const int pRX = A4;       //rotation x-axis
const int pRY = A5;       //rotation y-axis
const int pRZ = A6;       //rotation z-axis

//digital pins
const int pPOWER = 2;       //power switch
const int pTB = 3;          //translation joystick button
const int pRB = 4;          //rotation joystick button
const int latchPin = 11;     //ST_CP - green
const int dataPin = 10;     //DS - yellow
const int clockPin = 12;    //SH_CP - blue
const int pMODE = 22;       //mode switch (used for debug mode)
const int pLCDx = 27;       //toggle switch x (used for LCD display modes)
const int pLCDy = 24;       //toggle switch y (used for LCD display modes)
const int pLCDz = 29;       //toggle switch z (used for LCD display modes)
const int pSAS = 26;        //SAS switch
const int pRCS = 31;        //RCS switch
const int pABORT = 28;      //Abort switch (safety switch, active high)
const int pARM = 30;        //Arm switch (safety switch, active high)
const int pSTAGE = 32;      //Stage button
const int pSTAGELED = 33;   //Stage button LED
const int pLIGHTS = 34;     //Lights button
const int pLIGHTSLED = 35;  //Lights button LED
const int pLADDER = 36;     //Ladder button (action group 5)
const int pLADDERLED = 37;  //Ladder button LED
const int pSOLAR = 38;      //Solar button (action group 6)
const int pSOLARLED = 39;   //Solar button LED
const int pCHUTES = 40;     //Chutes button (action group 7)
const int pCHUTESLED = 41;  //Chutes button LED
const int pGEARS = 42;      //Gears button
const int pGEARSLED = 43;   //Gears button LED
const int pBRAKES = 44;     //Brakes button
const int pBRAKESLED = 45;  //Brakes button LED
const int pACTION1 = 46;    //Action Group 1 button
const int pACTION1LED = 47; //Action Group 1 button LED
const int pACTION2 = 48;    //Action Group 2 button
const int pACTION2LED = 49; //Action Group 2 button LED
const int pACTION3 = 50;    //Action Group 3 button
const int pACTION3LED = 51; //Action Group 3 button LED
const int pACTION4 = 52;    //Action Group 4 button
const int pACTION4LED = 53; //Action Group 4 button LED

//DEF VARS BOUTONS
bool etatBout2; bool etatBout3;
bool etatNowBout4; bool etatLastBout4 = 0;
bool etatNowBout5; bool etatLastBout5 = 1;
bool etatNowBout6; bool etatLastBout6 = 1;
bool etatNowBout7; bool etatLastBout7 = 1;
bool etatNowBout8; bool etatLastBout8 = 1;
bool etatNowBout9; bool etatLastBout9 = 1;
bool etatNowBout10; bool etatLastBout10 = 1;
bool etatNowBout11; bool etatLastBout11 = 1;
bool etatNowBoutST; bool etatLastBoutST = 1;
bool etatNowBout13; bool etatLastBout13 = 1;
bool etatNowBout14; bool etatLastBout14 = 1;
bool etatNowBout15; bool etatLastBout15 = 1;
bool etatNowBout16; bool etatLastBout16 = 1;
bool etatNowBout17; bool etatLastBout17 = 1;
bool etatNowBout18; bool etatLastBout18 = 1;
bool etatNowBout19; bool etatLastBout19 = 1;
bool etatNowBout20; bool etatLastBout20 = 1;
bool etatNowBout21; bool etatLastBout21 = 1;
bool etatNowBout22; bool etatLastBout22 = 1;
bool etatNowBout23; bool etatLastBout23 = 1;
bool gearLED = false;
bool brakesLED = false;
bool solarLED = false;
bool chutesLED = false;
bool lightsLED = false;
bool ladderLED = false;
bool A1LED = false;
bool A2LED = false;
bool A3LED = false;
bool A4LED = false;

byte inputBytes[7];

byte vSF, vLF, vOX, vEL, vMP, SF, LF, OX, EL, MP;
int LCDmode;

//DEF Variables Utiles
bool newpacket;
String inputString;
bool stringComplet = false;
//DEF DATA LCD Screen
byte start = 0;
String Packet;
int gamemode;
float surfVitesse = 0.0; float Acceleration = 0.0;                                   //TakeOff Mode
float apo_alt = 0.0; long apo_time = 0; float peri_alt = 0.0; long peri_time = 0;    //Orbit Mode
long TimetoManeuver = 0; float remainingDV = 0;                                      //Maneuver Mode
float distanceToTarget = 0.0; float relativeVelocityToTarget = 0.0;                  //RDV Mode
byte Overheating = 0;                                                                //Re-entry Mode
float altitude = 0.0;  float machNumber = 0.0;                                       //Landing Mode
float surfAltitude = 0.0; float verticalVelocity = 0.0;                              //Flight Mode
//Unused Mode

double funds; double science;                                                        //Space Center Mode
                                                                                     //Tracking Station Mode
                                                                                     //VAB Mode
                                                                                     //SPH Mode
bool change = false;
bool armed = false;
bool handshake = false;
bool receive = false;

void Lecture()
{

  //Serial.println("ping");
  if(newpacket)
{
  start = 0;
  int packetType = readUntil().toInt();
  byte BP = readUntil().toInt();
    //Serial.println(BP);
    gearLED = bitRead(BP,7);
    brakesLED = bitRead(BP,6);
    solarLED = bitRead(BP,5);
    chutesLED = bitRead(BP,4);
    lightsLED = bitRead(BP,3);
    ladderLED = bitRead(BP,2);
    A1LED = bitRead(BP,1);
    A2LED = bitRead(BP,0);
    
    byte BP2 = readUntil().toInt();
    A3LED = bitRead(BP2,1);
    A4LED = bitRead(BP2,0);

    SF = readUntil().toInt();
    LF = readUntil().toInt();
    OX = readUntil().toInt();
    EL = readUntil().toInt();
    MP = readUntil().toInt();
  if(packetType == 0)// Game Scene DATA ( space center, flight, tracking station ... + Data like funds, science ...)
        {
          Acceleration = readUntil().toFloat();
          surfVitesse = readUntil().toFloat();
        }
    else if(packetType == 1)
    {
        apo_alt = readUntil().toFloat();
        peri_alt = readUntil().toFloat();
        apo_time = readUntil().toInt();
        peri_time = readUntil().toInt();
    }
    else if(packetType == 2)
      {
       TimetoManeuver = readUntil().toInt();
       remainingDV = readUntil().toFloat();
      }
      else if(packetType == 3)
      {
      distanceToTarget = readUntil().toFloat();
      relativeVelocityToTarget = readUntil().toFloat();
      }
      else if(packetType == 4)
      {
       Acceleration = readUntil().toFloat();
       Overheating = readUntil().toInt();
      }
      else if(packetType == 5)
      {
        altitude = readUntil().toFloat();
        machNumber = readUntil().toFloat();
      }
      else if(packetType == 6)
      {
      surfAltitude = readUntil().toFloat();
      verticalVelocity = readUntil().toFloat();
      }
      
    newpacket = false;
}
}

void Ecriture()
{
  digitalWrite(pGEARSLED,gearLED);
  digitalWrite(pBRAKESLED,brakesLED);
  digitalWrite(pSOLARLED,solarLED);
  digitalWrite(pCHUTESLED,chutesLED);
  digitalWrite(pLIGHTSLED,lightsLED);
  digitalWrite(pLADDERLED,ladderLED);
  digitalWrite(pACTION1LED,A1LED);
  digitalWrite(pACTION2LED,A2LED);
  digitalWrite(pACTION3LED,A3LED);
  digitalWrite(pACTION4LED,A4LED);
  gamemode = 5;
    if(gamemode == 1)// Space Center
    {
      clearLCD();
      //Funds
      char bufferfunds[17];
      String strfunds = "Fonds: ";
      if (funds < 99999999)
      {
        strfunds += String(funds, 0);
      }
      else if (funds < 99999999999)
      {
        strfunds += String((funds / 1000), 0);
        strfunds += " k";
      }
      else if (funds < 99999999999999)
      {
        strfunds += String((funds / 1000000), 0);
        strfunds += " m";
      }
      else
      {
        strfunds += String((funds / 1000000000), 0);
        strfunds += " M";
      }
      strfunds.toCharArray(bufferfunds, 17);
      writeLCD(bufferfunds);
      //Science
      jumpToLineTwo();
      char bufferScience[17];
      String strScience = "Science:";
      strScience += String(science, 0);
      strScience += " pts";
      strScience.toCharArray(bufferScience, 17);
      writeLCD(bufferScience);
    }

    else if(gamemode == 2) // VAB Editor
    {
      clearLCD();
      writeLCD("   Bienvenue");
      jumpToLineTwo();
      writeLCD("     au VAB");
    }

    else if(gamemode == 3) // SPH Editor
    {
      clearLCD();
      writeLCD("   Bienvenue");
      jumpToLineTwo();
      writeLCD("     au SPH");
    }

    else if(gamemode == 4)  // Tracking Station
    {
      clearLCD();
      writeLCD(" Bienvenue a la");
      jumpToLineTwo();
      writeLCD("Tracking Station");
    }

    else if(gamemode == 5)  // Flight
    {

      int powOX = 0.1 + pow(2, OX);
      int powEL = 0.1 + pow(2, EL);
      int powMP = 0.1 + pow(2, MP);
      int powSF = 0.1 + pow(2, SF);
      int powLF = 0.1 + pow(2, LF);

      //map the 8-bit 595 shift registers to the 10-bit LED bars, specific to the way I wired them
      inputBytes[0] = powMP >> 8;
      inputBytes[1] = powMP;
      inputBytes[2] =  powEL >> 2;
      inputBytes[3] = (powEL << 6) | (powOX >> 4);
      inputBytes[4] = (powOX << 4) | (powLF >> 6);
      inputBytes[5] = (powLF << 2) | (powSF >> 8);
      inputBytes[6] = powSF;

      digitalWrite(latchPin, LOW);
      for (int j = 0; j <= 6; j++)
      {
        byte inputByte = inputBytes[j];
        shiftOut(dataPin, clockPin, MSBFIRST, inputByte);
      }
      digitalWrite(latchPin, HIGH);
      byte LastLCDmode = LCDmode;
      LCDmode = !etatNowBout21 << 2 | !etatNowBout22 << 1 | !etatNowBout23;
      if (LCDmode != LastLCDmode){Serial.print("LCD");Serial.println(LCDmode);}
      
      if(LCDmode == 0)//MODE 0 : TakeOff Mode
      {
          //Vsurf
          clearLCD();
          char bufferVsurf[17];
          String strVsurf = "Vsurf: ";
          if(surfVitesse < 1000){strVsurf += String(surfVitesse, 1); strVsurf += " m/s";}
          else if(surfVitesse < 100000){strVsurf += String(surfVitesse, 0); strVsurf += " m/s";}
          else {strVsurf += String(surfVitesse/1000, 0); strVsurf += " km/s";}
          strVsurf.toCharArray(bufferVsurf, 17);
          writeLCD(bufferVsurf);
          //Acceleration (G)
          jumpToLineTwo();
          char bufferGee[17];
          String strGee = "Accel: ";
          strGee += String(Acceleration, 1);
          strGee += " G";
          strGee.toCharArray(bufferGee, 17);
          writeLCD(bufferGee);
      }

      else if(LCDmode == 1) //MODE 1: Orbit Mode
      {
          clearLCD();
          //Apoapsis
          char bufferAP[17];
          String strApo = "AP: ";
          if (apo_alt < 10000 && apo_alt > -10000)
          {
            strApo += String(apo_alt, 0);
            strApo += "m ";
          }
          else if ((apo_alt >= 10000 && apo_alt < 10000000) || (apo_alt <= -10000 && apo_alt > -10000000))
          {
            strApo += String((apo_alt / 1000), 0);
            strApo += "km ";
          }
          else if ((apo_alt >= 10000000 && apo_alt < 10000000000) || (apo_alt <= -10000000 && apo_alt > -10000000000))
          {
            strApo += String((apo_alt / 1000000), 0);
            strApo += "Mm ";
          }
          else
          {
            strApo += String((apo_alt / 1000000000), 0);
            strApo += "Gm ";
          }
          if(apo_time < 3600)
          {
            strApo += String(apo_time); //time to apoapsis
            strApo += " s";
          }
          else 
          {
            byte h = apo_time / 3600;
            strApo += String(h); //time to apoapsis
            strApo += " h";
          }
          
          strApo.toCharArray(bufferAP, 17);
          writeLCD(bufferAP);

          //Periapsis
          char bufferPE[17];
          String strPeri = "PE: ";
          if (peri_alt < 10000 && peri_alt > -10000)
          {
            strPeri += String(peri_alt, 0);
            strPeri += "m ";
          }
          else if ((peri_alt >= 10000 && peri_alt < 10000000) || (peri_alt <= -10000 && peri_alt > -10000000))
          {
            strPeri += String((peri_alt / 1000.0), 0);
            strPeri += "km ";
          }
          else if ((peri_alt >= 10000000 && peri_alt < 10000000000) || (peri_alt <= -10000000 && peri_alt > -10000000000))
          {
            strPeri += String((peri_alt / 1000000.0), 0);
            strPeri += "Mm ";
          }
          else
          {
            strPeri += String((peri_alt / 1000000000.0), 0);
            strPeri += "Gm ";
          }
          if(peri_time < 3600)
          {
            strPeri += String(peri_time); //time to apoapsis
            strPeri += " s";
          }
          else 
          {
            byte h = peri_time / 3600;
            strPeri += String(h); //time to apoapsis
            strPeri += " h";
          }
          
          strPeri.toCharArray(bufferPE, 17);
          jumpToLineTwo();
          writeLCD(bufferPE);
      }

      else if(LCDmode == 2) //MODE 2: Maneuver Mode
      {
          //MNTime
          clearLCD();
          char s[2];
          char m[2];
          char h[2];
          if(TimetoManeuver < 60)
          {
            dtostrf(TimetoManeuver, 2, 0, s);
            writeLCD("Tnode: ");
            writeLCD(s);
            writeLCD(" s");
          }
          else if(TimetoManeuver < 3600)
          {
            byte secs = TimetoManeuver % 60;
            byte mins = TimetoManeuver / 60;
            dtostrf(secs, 2, 0, s);
            dtostrf(mins, 2, 0, m);
            writeLCD("Tnode: ");
            writeLCD(m);
            writeLCD(" min ");
            writeLCD(s);
            writeLCD(" s");
          }
          else
          {
            byte hours = TimetoManeuver / 3600;
            byte mins = (TimetoManeuver / 3600) % 60;
            dtostrf(mins, 2, 0, m);
            dtostrf(hours, 4, 0, h);
            writeLCD("Tnode: ");
            writeLCD(h);
            writeLCD(" h ");
            writeLCD(m);
            writeLCD(" min");
          }
          //MNDeltaV
          jumpToLineTwo();
          char bufferMNDeltaV[17];
          String strMNDeltaV = "dV: ";
          strMNDeltaV += String(remainingDV, 1);
          strMNDeltaV += " m/s";
          strMNDeltaV.toCharArray(bufferMNDeltaV, 17);
          writeLCD(bufferMNDeltaV);
      }

      else if(LCDmode == 3)//MODE 3: Rendezvouz Mode
      {
          //Target Distance
          clearLCD();
          char bufferTargetDist[17];
          String strTargetDist = "TDist: ";
          strTargetDist += String(distanceToTarget, 0);
          strTargetDist += " m";
          strTargetDist.toCharArray(bufferTargetDist, 17);
          writeLCD(bufferTargetDist);
          //Target Velocity
          jumpToLineTwo();
          char bufferTargetV[17];
          String strTargetV = "TVel: ";
          strTargetV += String(relativeVelocityToTarget, 0);
          strTargetV += " m/s";
          strTargetV.toCharArray(bufferTargetV, 17);
          writeLCD(bufferTargetV);
      }

      else if(LCDmode == 4) //MODE 4: Re-Entry Mode
      {      
            //MaxOverHeat
            clearLCD();
            char t[3];
            dtostrf(Overheating, 3, 0, t);
            writeLCD("Heat: ");
            writeLCD(t);
            writeLCD("%");
            //Acceleration (G)
            jumpToLineTwo();
            char bufferGee[17];
            String strGee = "Decel: ";
            strGee += String(Acceleration, 0);
            strGee += " G";
            strGee.toCharArray(bufferGee, 17);
            writeLCD(bufferGee);
        }

      else if(LCDmode == 5) //MODE 5: Landing Mode
      {
          //RAlt
          clearLCD();
          char bufferRAtl[17];
          String strRAlt = "RAlt: ";
          strRAlt += String(surfAltitude, 0);
          strRAlt += " m";
          strRAlt.toCharArray(bufferRAtl, 17);
          writeLCD(bufferRAtl);
          //Vertical Velocity
          jumpToLineTwo();
          char bufferVVI[17];
          String strVVI = "VVI:  ";
          strVVI += String(verticalVelocity, 0);
          strVVI += " m/s";
          strVVI.toCharArray(bufferVVI, 17);
          writeLCD(bufferVVI);
      }

      else if(LCDmode == 6) //MODE 6: Flying Mode
      {
          //Alt
          clearLCD();
          char bufferAtl[17];
          String strAlt = "Alt:  ";
          strAlt += String(altitude, 0);
          strAlt += " m/s";
          strAlt.toCharArray(bufferAtl, 17);
          writeLCD(bufferAtl);
          //Mach Number
          jumpToLineTwo();
          char bufferMachNumber[17];
          String strMach = "Mach:";
          strMach += String(machNumber, 0);
          strMach.toCharArray(bufferMachNumber, 17);
          writeLCD(bufferMachNumber);
      }

      else if(LCDmode == 7) //MODE 7: Unused Mode
      {
          clearLCD();
          writeLCD("KerbalController");
      }

  }


}

void gestion_BP()
{
  //Read every Button state
  etatBout2 = digitalRead(pPOWER);
  etatBout3 = digitalRead(pMODE);
  etatNowBout4 = digitalRead(pABORT);
  etatNowBout5 = digitalRead(pARM);
  etatNowBout6 = digitalRead(pLIGHTS);
  etatNowBout7 = digitalRead(pSAS);
  etatNowBout8 = digitalRead(pRCS);
  etatNowBout9 = digitalRead(pLADDER);
  etatNowBout10 = digitalRead(pSOLAR);
  etatNowBout11 = digitalRead(pCHUTES);
  etatNowBoutST = digitalRead(pSTAGE);
  etatNowBout13 = digitalRead(pGEARS);
  etatNowBout14 = digitalRead(pBRAKES);
  etatNowBout15 = digitalRead(pACTION1);
  etatNowBout16 = digitalRead(pACTION2);
  etatNowBout17 = digitalRead(pACTION3);
  etatNowBout18 = digitalRead(pACTION4);
  etatNowBout19 = digitalRead(pTB);
  etatNowBout20 = digitalRead(pRB);
  etatNowBout21 = digitalRead(pLCDx);
  etatNowBout22 = digitalRead(pLCDy);
  etatNowBout23 = digitalRead(pLCDz);

  //For each button
  {
    if (etatNowBout6 != etatLastBout6) // LIGHTS
    {
      if (!etatNowBout6)
      {
        Serial.println("light");
        static bool ledlight = false;
        digitalWrite(pLIGHTSLED, ledlight);
        ledlight = !ledlight;
      }
      etatLastBout6 = etatNowBout6;
      delay(50);
    }

    if (etatNowBout7 != etatLastBout7) // SAS
    {
      Serial.println("sas");
      etatLastBout7 = etatNowBout7;
      delay(50);
    }

    if (etatNowBout8 != etatLastBout8) // RCS
    {
      Serial.println("rcs");
      etatLastBout8 = etatNowBout8;
      delay(50);
    }

    if (etatNowBout9 != etatLastBout9) // LADDER
    {
      if (!etatNowBout9)
      {
        Serial.println("ladder");
        static bool ledladder = false;
        digitalWrite(pLADDERLED, ledladder);
        ledladder = !ledladder;
      }
      etatLastBout9 = etatNowBout9;
      delay(50);
    }

    if (etatNowBout10 != etatLastBout10) // SOLAR
    {
      if (!etatNowBout10)
      {
        Serial.println("solar");
        static bool ledsolar = false;
        digitalWrite(pSOLARLED, ledsolar);
        ledsolar = !ledsolar;
      }
      etatLastBout10 = etatNowBout10;
      delay(50);
    }

    if (etatNowBout11 != etatLastBout11) // CHUTES
    {
      if (!etatNowBout11)
      {
        Serial.println("chutes");
        static bool ledchutes = false;
        digitalWrite(pCHUTESLED, ledchutes);
        ledchutes = !ledchutes;
      }
      etatLastBout11 = etatNowBout11;
      delay(50);
    }

    if (etatNowBout13 != etatLastBout13) // GEARS
    {
      if (!etatNowBout13)
      {
        Serial.println("gears");
        static bool ledgears = false;
        digitalWrite(pGEARSLED, ledgears);
        ledgears = !ledgears;
      }
      etatLastBout13 = etatNowBout13;
      delay(50);
    }

    if (etatNowBout14 != etatLastBout14) // BRAKES
    {
      if (!etatNowBout14)
      {
        Serial.println("brakes");
        static bool ledbrakes = false;
        digitalWrite(pBRAKESLED, ledbrakes);
        ledbrakes = !ledbrakes;
      }
      etatLastBout14 = etatNowBout14;
      delay(50);
    }

    if (etatNowBout15 != etatLastBout15) // ACTION 1
    {
      if (!etatNowBout15)
      {
        Serial.println("act1");
        static bool ledact1 = false;
        digitalWrite(pACTION1LED, ledact1);
        ledact1 = !ledact1;
      }
      etatLastBout15 = etatNowBout15;
      delay(50);
    }

    if (etatNowBout16 != etatLastBout16) // ACTION 2
    {
      if (!etatNowBout16)
      {
        Serial.println("act2");
        static bool ledact2 = false;
        digitalWrite(pACTION2LED, ledact2);
        ledact2 = !ledact2;
      }
      etatLastBout16 = etatNowBout16;
      delay(50);
    }

    if (etatNowBout17 != etatLastBout17) // ACTION 3
    {
      if (!etatNowBout17)
      {
        Serial.println("act3");
        static bool ledact3 = false;
        digitalWrite(pACTION3LED, ledact3);
        ledact3 = !ledact3;
      }
      etatLastBout17 = etatNowBout17;
      delay(50);
    }

    if (etatNowBout18 != etatLastBout18) // ACTION 4
    {
      if (!etatNowBout18)
      {
        Serial.println("act4");
        static bool ledact4 = false;
        digitalWrite(pACTION4LED, ledact4);
        ledact4
          = !ledact4;
      }
      etatLastBout18 = etatNowBout18;
      delay(50);
    }

    if (etatNowBout19 != etatLastBout19) // pTB
    {
      if (!etatNowBout19)
      {
        Serial.println("pTB");
      }
      etatLastBout19 = etatNowBout19;
      delay(50);
    }

    if (etatNowBout20 != etatLastBout20) // pRB
    {
      if (!etatNowBout20)
      {
        Serial.println("pRB");
      }
      etatLastBout20 = etatNowBout20;
      delay(50);
    }

    if (etatNowBout4 != etatLastBout4) // ABORT + STAGE
    {
      Serial.println("abort");
      etatLastBout4 = etatNowBout4;
    }

    if (etatNowBout5 == HIGH)
    {
      armed = true;
    }
    else
    {
      armed = false;
      digitalWrite(pSTAGELED, LOW);
    }
    if (armed)
    {
      long now = millis();
      static long lastnow = 0;
      static bool state = HIGH;
      if (now - lastnow > 500)
      {
        digitalWrite(pSTAGELED, state);
        state = !state;
        lastnow = now;
      }

      if (etatNowBoutST != etatLastBoutST) //STAGE
      {
        if (etatNowBoutST == 1)
        {
          Serial.println("boutST");
        }
        etatLastBoutST = etatNowBoutST;
        delay(50);
      }
    }
    unsigned long now = millis();
    static unsigned long lastnow = 0;
    if (now - lastnow > 100)
    {
    Serial.print("Th0");
    Serial.println(analogRead(pTHROTTLE));
    Serial.print("TX0");
    Serial.println(analogRead(pTX));
    Serial.print("TY0");
    Serial.println(analogRead(pTY));
    Serial.print("TZ0");
    Serial.println(analogRead(pTZ));
    Serial.print("RX0");
    Serial.println(analogRead(pRX));
    Serial.print("RY0");
    Serial.println(analogRead(pRY));
    Serial.print("RZ0");
    Serial.println(analogRead(pRZ));
    lastnow = now;
    }
  }
}

void controlsInit()
{
  pinMode(pTHROTTLE, INPUT);
  pinMode(pTX, INPUT);
  pinMode(pTY, INPUT);
  pinMode(pTZ, INPUT);
  pinMode(pRX, INPUT);
  pinMode(pRY, INPUT);
  pinMode(pRZ, INPUT);
  pinMode(pPOWER, INPUT_PULLUP);
  pinMode(pTB, INPUT_PULLUP);
  pinMode(pRB, INPUT_PULLUP);
  pinMode(pMODE, INPUT_PULLUP);
  pinMode(pLCDx, INPUT_PULLUP);
  pinMode(pLCDy, INPUT_PULLUP);
  pinMode(pLCDz, INPUT_PULLUP);
  pinMode(pSAS, INPUT_PULLUP);
  pinMode(pRCS, INPUT_PULLUP);
  pinMode(pABORT, INPUT_PULLUP);
  pinMode(pARM, INPUT);
  pinMode(pSTAGE, INPUT_PULLUP);
  pinMode(pSTAGELED, OUTPUT);
  pinMode(pLIGHTS, INPUT_PULLUP);
  pinMode(pLIGHTSLED, OUTPUT);
  pinMode(pLADDER, INPUT_PULLUP);
  pinMode(pLADDERLED, OUTPUT);
  pinMode(pSOLAR, INPUT_PULLUP);
  pinMode(pSOLARLED, OUTPUT);
  pinMode(pCHUTES, INPUT_PULLUP);
  pinMode(pCHUTESLED, OUTPUT);
  pinMode(pGEARS, INPUT_PULLUP);
  pinMode(pGEARSLED, OUTPUT);
  pinMode(pBRAKES, INPUT_PULLUP);
  pinMode(pBRAKESLED, OUTPUT);
  pinMode(pACTION1, INPUT_PULLUP);
  pinMode(pACTION1LED, OUTPUT);
  pinMode(pACTION2, INPUT_PULLUP);
  pinMode(pACTION2LED, OUTPUT);
  pinMode(pACTION3, INPUT_PULLUP);
  pinMode(pACTION3LED, OUTPUT);
  pinMode(pACTION4, INPUT_PULLUP);
  pinMode(pACTION4LED, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}
void clearLCD()
{
  //clear the LCD by writing all spaces
  jumpToStart();
  mySerial.write("                ");
  mySerial.write("                ");
  jumpToStart();
}

void jumpToStart()
{
  //jump to the start of the first line on the LCD
  mySerial.write(254);
  mySerial.write(128);
}

void jumpToLineTwo()
{
  //jump to the start of the second line on the LCD
  mySerial.write(254);
  mySerial.write(192);
}

void writeLCD(char myText[])
{
  //write text to the LCD
  mySerial.write(myText);
}
String readUntil()
{
  String result;
  while (Packet.charAt(start) != ';')
  {
    result += Packet.charAt(start);
    start ++;
  }
  start ++;
  return result;
}

void serialEvent()
{
    newpacket = true;
    Packet = Serial.readStringUntil('$');
    Serial.flush();
}

void Handshake()
{
  clearLCD();
  writeLCD("  Waiting for");
  jumpToLineTwo();
  writeLCD(" connection ...");
  start = 0;
  while (!handshake)
  {
    if (Serial.available())
    {
      if (Serial.peek() == 0x23)
      {
        Serial.read();
        delay(500); 
        String Packet = Serial.readStringUntil("$");
        if (Packet == "1234$")
        {
          Serial.println("4321");
          handshake = true;
          clearLCD();
          writeLCD("handshake");
          delay(500);
        }
        if (Packet == "4321$")
        {
          receive = true;
          clearLCD();
          writeLCD("receive");
          delay(500);
        }
      }
    }
  }
  clearLCD();
  writeLCD("KerbalController");
  jumpToLineTwo();
  writeLCD("   connected !  ");
}

void setup() {

  //SERIAL
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  Serial.begin(9600);
  mySerial.begin(9600);
  
  clearLCD();
  writeLCD("KerbalController");
  jumpToLineTwo();
  writeLCD("  initializing  ");

  controlsInit(); // Initialisations des I/O

  inputBytes[0] = B00000000;
  inputBytes[1] = B00000001;
  inputBytes[2] = B00000000;
  inputBytes[3] = B01000000;
  inputBytes[4] = B00010000;
  inputBytes[5] = B00000100;
  inputBytes[6] = B00000001;
  digitalWrite(latchPin, LOW);
  for (int j = 0; j <= 6; j++)
  {
    byte inputByte = inputBytes[j];
    shiftOut(dataPin, clockPin, MSBFIRST, inputByte);
  }
  digitalWrite(latchPin, HIGH);
  Handshake();
}

void loop()
{
  gestion_BP();
  Lecture();
  Ecriture();
}
