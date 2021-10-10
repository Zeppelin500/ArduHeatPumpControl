/*
# ArduHeatPumpControl

ArduHeatPumpControl V1.0  
Arduino based heat pump control unit with touch screen by Zeppelin500

Developed for a water-water heat pump but also usable for a brine-water heat pump.
Main components are an Arduino Mega, a Nextion display NX3224K024_11 and onewire DS18B20 sensors.

I´ve devoloped it for my defect Dimplex WI9TE control unit (Wärmepumpenmanager) but it should work in the most HPs with a thermostatic expansion valve.

### Programmed are (only) the basic usecases that I use:
* outside temperature controlled (heating curve)
* summer / winter mode
* direct underfloor heating without a mixing valve
* (drink) water heating
* water heating is prefered before heating
* electric provider blocking (EVU-Sperre)

### Highlights:
* touch display with differend pages to control and manipulate
* some safety precautions (high/low pressure, icing, flow, proof of plausibility of all temperatures, watchdog)
* Error memory with timestamp and state of all readings in EEPROM
* RTC
* heating curve offset and both hyseresis can be manipulated with touchscreen (heating curve is hardcoded)
* 6 independend onewire buses implemented, so you can replace a sensor without touching the software

The Project includes 2 files. The Arduino .ino file and the .hmi file for the Nextion display.
Although the display is high potent the "intelligence" is only written to the Arduino code. The display is only used to show values and notice touch buttons. 
Also the build in rtc with the battery is used for the Arduino time.   

Thanks to Seithan for the great Nextion libary "EasyNextionLibrary"! The Libary was the key to deal with the touch "easyly".

## Caution:
If you try to implement the control unit to your heat pump, you should be educated as electrician, because you have to deal with 400V!
This code is in real operation since about 10 month and it works perfect. Warning: If you don´t know what you are doing, your compressor may get damaged.

## Note: 
GUI and most comments are in german language, because I wrote it for myself. 

## Known issues:
First I used for the pressure failurs hardware interrupts. But I had some EMC problems, so I commented it out and programmed the failure detecting (laborius) manually around the switching moment of the contactor (causer of the emc problems).
The EMC problems are now fixed but the code works reliable. At the moment I´m not sure if I should delete or reimplement the interrupts. 

****************************************************
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version. This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************
 */

// Attention: All temperature values and calculations are multiplied with 10. Because the display do only understand integers, no floats. e.g.: 12,3°C is 123  

#include <MemoryFree.h>           // Speicherüberwachung zur Fehlersuche
#include "EasyNextionLibrary.h"   // Include EasyNextionLibrary   

#include <EEPROM.h>               // Libary für die Benutzung des EEPROM
#include <OneWire.h>              // Libary für den OneWire Bus
#include <DallasTemperature.h>    //Libary für die Temperatursensoren
#include <avr/wdt.h>              // Watchdog Libary


// OneWire data wire is plugged into port xx on the Arduino
// From each data wire one 4K7Ohm Resistor to VCC

#define ONE_WIRE_BUS2 48  // Temperatur Aussen (mTistA)
#define ONE_WIRE_BUS5 49  // Temperatur Warmwasserspeicher (mTistWW)
#define ONE_WIRE_BUS4 50  // Temperatur Rücklauf Heizwasser (mTistRL)
#define ONE_WIRE_BUS3 51  // Temperatur Brunnen Rücklauf (mTistBrRL)
#define ONE_WIRE_BUS6 52  // Temperatur Brunnen Vorlauflauf (mTistBrVL)
#define ONE_WIRE_BUS1 53  // Temperatur Vorlauf Heizwasser (mTistVL)

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire1(ONE_WIRE_BUS1);
OneWire oneWire2(ONE_WIRE_BUS2);
OneWire oneWire3(ONE_WIRE_BUS3);
OneWire oneWire4(ONE_WIRE_BUS4);
OneWire oneWire5(ONE_WIRE_BUS5);
OneWire oneWire6(ONE_WIRE_BUS6);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);
DallasTemperature sensor3(&oneWire3);
DallasTemperature sensor4(&oneWire4);
DallasTemperature sensor5(&oneWire5);
DallasTemperature sensor6(&oneWire6);


unsigned long getDataTimer = millis();

EasyNex myNex(Serial1); // Create an object of EasyNex class with the name < myNex >
                       // Das Display wird an Serial 1 des Arduino angeschlossen (Pin TX18,RX19)

// Pins für Eingänge ------------------------------------------------------------
const int evuPin = 4;       // Pin für EVU Sperre
const int durchflussPin = 5;// Pin für Paddelwächter

const int hdPin = 2;   // Hochdruckpressostat 
const int ndPin = 3;   // Niederdruckdruckpressostat

// Pins für Ausgänge ------------------------------------------------------------
const int brunnenPin = 26;    // Pin für Brunnenpumpe          active high  - solid state relais - direct
const int hzPin = 27;         // Pin für Heizungsumwälzpumpe   active low   - cheap relais board - inrush current limiter               
const int wwPin = 28;         // Pin für Warmwasserpumpe       active low   - cheap relais board - inrush current limiter  
const int kompPin = 29;       // Pin für Kompressor            active low   - cheap relais board - contactor

// Variablen für Eingänge ---------------------------------------------------------


bool evuSperre = 0;       
bool paddelwaechter = 1;
bool hdpresostat = 0;
bool ndpresostat = 0;



// Adressen für EEPROM
                       
int eeAddrOffset = 0; //EEPROM address to start reading from
int eeAddrWWsoll = 2;
int eeAddrWWhyst = 4;
int eeAddrHZhyst = 6;

int eeAddrFehler1 = 8; //Fehlerspeicher 1
int eeAddrFjahr1 =10 ;    // Jahr
int eeAddrFmonat1 = 12;   // Monat
int eeAddrFtag1 = 14;     // Tag
int eeAddrFstunde1 = 16;  // Stunde
int eeAddrFminute1 = 18;  // Minute
int eeAddrFsekunde1 = 20; // Sekunde

int eeAddrFehler2 = 22; //Fehlerspeicher 2
int eeAddrFjahr2 =24 ;
int eeAddrFmonat2 = 26;
int eeAddrFtag2 = 28;
int eeAddrFstunde2 = 30;
int eeAddrFminute2 = 32;
int eeAddrFsekunde2 = 34;

int eeAddrSommer = 36;  // Sommer - Winter Modus 

// Variablen

// validierte Messwerte ( die Messwerte mTistXX werden auf Plausibilität geprüft und dann in TistXX übernommen)

int TistA = 0;  // Temperatur IST Aussen   
int TistVL = 0; // Temperatur IST Vorlauf
int TistRL = 0; // Temperatur IST Rücklauf
int TsollRL = 0;  // Temperatur Soll Rücklauf
int TsollRLmO = 0;  // Temperatur Soll Rücklauf mit Offset
int TistBrVL = 0;   // Temperatur IST vor PrimärWärmetauscher
int TistBrRL = 0;  // Temperatur IST nach PrimärWärmetauscher 
int TistWW = 0;  // Temperatur IST Warmwasserspeicher
int TsollWW = 0;

// Messwerte direkt von den Sensoren

int mTistA = 0;  // Temperatur IST Aussen   
int mTistVL = 0; // Temperatur IST Vorlauf
int mTistRL = 0; // Temperatur IST Rücklauf
int mTistBrVL = 0;   // Temperatur IST vor PrimärWärmetauscher
int mTistBrRL = 0;  // Temperatur IST nach PrimärWärmetauscher 
int mTistWW = 0;  // Temperatur IST Warmwasserspeicher

//  Fehlerzähler zu den Sensoren (Nicht plausieble Messwerte werden je Sensor Hochgezählt --> zu Fehlersuche)
int TistAdsFcount = 0;
int TistVLdsFcount = 0;
int TistRLdsFcount = 0;
int TistBrVLdsFcount = 0;
int TistBrRLdsFcount = 0;
int TistWWdsFcount = 0;
int maxFehlmessungen = 100;

int zykluszeit = 0; // Arduino Rechenzeit zur Fehlersuche

// Variablen zur Temperaturmanipulation
int offset = 0; // Parallelverschieben der Heizkurve
int hzHyst = 0; // Hysterese der Heizung
int wwHyst = 0; // Hysterese der Warmwasser Bereitung


int aktuelleSeite = 0;  // Startseite des Displays


volatile byte fehlercode = 0;/* Fehlercodes
                        0=kein Fehler
                        1= kein Durchfluss primär 
                        2= Niederdruck ausgelöst
                        3= Hochduck ausgelöst
                        4= Eingefrierschutz ausgelöst
                        5= max VL überschritten
                        6= Durchflusswächter ohne Funktion
                        7= Temperatur Quelle zu gering
                        8= Temperatur unplausibel
                        9= Fehlmessung, Sensor defekt     
                        */
volatile int zustand = 0; /*  Das Hauptprogramm ist als Zustandsautomat ausgeführt 
                        0=Selbsttest
                        1=Werte ermitteln,
                        2=Spülen v. Kompressorstart Heizen, 
                        3=Spülen v. Kompressorstart Warmwasser,
                        4=Spülen n. Kompressorstop,
                        5=Heizen,
                        6=Warmwasser, 
                        7=taktsperre
                        8=Fehlerspülen,
                        9=Fehler,
                        10=Standby
                        11=EVU Sperre
                        12=Spülen EVU Sperre
                        13=Spülen Systemkontrolle
                        14=Systemkontrolle
                        15=Primärspülen
                        */
                        
// Flags für die Zustände bzw. Anforderungen
boolean anfHZ = false;                  // Anforderung Heizung
boolean anfWW = false;                  // Anforderung Warmwasser
boolean anfPreWW = false;               // Anforderung Warmwasser wenn Kompressor eh schon läuft.. (Hysterese halbiert, reduziert die Kompressorstarts massiv)
boolean sommer = false;                 // Sommer oder Wintermodus 
volatile boolean firstloop = false;     // Flag zur Abfrage ob in den Case das erste mal gesprungen wurde. (z.B. zum speichern der aktuellen Werte im Fehlerfall)
volatile boolean unresetfailure = false;// Flag zur Abfrage ob es einen nicht zurückgesetzten Fehler gibt.
 
unsigned long standbyTime = 0; // Wartezeit im Standby (Sommer und Winter unterscheiden sich)

#define LOOP_TIME 1000
unsigned long timer ;             // Timer für das Display
unsigned long timerSensorRefresh; // Timer zum Auslesen der Temperaturwerte
unsigned long timerCalc;          // Timer zur Neuberechnung der aktuellen Solltemperaturen aus der Heizkurve
unsigned long timerSwitchZustand; // Timer seit wann der aktuelle Zustand läuft.
unsigned long timerTaktsperre;    // Timer für die Taktsperre
unsigned long timerAnf;           // Timer zur Neuberechnung ob eine Warmwasseranforderung vorliegt
unsigned long timerDim;           // Timer zum Dimmen des Displays nach einer eingestellten Zeit --> noch nicht programmiert.
unsigned long timerZykluszeit;    // Timer zur Berechnung der Rechenzeit des Arduino
#define DATA_REFRESH_RATE 1000    // The time between each Data refresh of the Display page
unsigned long pageRefreshTimer = millis(); // Timer for DATA_REFRESH_RATE

bool newPageLoaded = false; // true when the page is first loaded ( lastCurrentPageId != currentPageId )

//------------Fehlerspeicher-------------------------------------------------------

int fehlerspeicher1 = 0;
int fJahr1 = 0;
int fMonat1 = 0;
int fTag1 = 0;
int fStunde1 = 0;
int fMinute1 = 0;
int fSekunde1 = 0;

int fehlerspeicher2 = 0;
int fJahr2 = 0;
int fMonat2 = 0;
int fTag2 = 0;
int fStunde2 = 0;
int fMinute2 = 0;
int fSekunde2 = 0;

int fTistA = 0;  // Temperatur IST Aussen   
int fTistVL = 0; // Temperatur IST Vorlauf
int fTistRL = 0; // Temperatur IST Rücklauf
int fTsollRL = 0;  // Temperatur Soll Rücklauf
int fTsollRLmO = 0;  // Temperatur Soll Rücklauf mit Offset
int fTistBrVL = 0;   // Temperatur IST vor PrimärWärmetauscher
int fTistBrRL = 0;  // Temperatur IST nach PrimärWärmetauscher 
int fTistWW = 0;  // Temperatur IST Warmwasserspeicher
int fTsollWW = 0;  // Temperatur soll Warmwasserspeicher

boolean fanfHZ = 0; // Anforderung Heizung
boolean fanfWW = 0; //ANforderung Warmwasser
boolean fsommer = 0; // Sommer/Winter
boolean fevuSperre = 0; //EVU Sperre

//--------------------------------------------------------------------------------------------------------------------------

void setup(){

  wdt_enable(WDTO_4S);  // Watchdog auf 4 s stellen
  
  sensor1.setWaitForConversion(false);  // makes it async
  sensor2.setWaitForConversion(false);  // makes it async
  sensor3.setWaitForConversion(false);  // makes it async
  sensor4.setWaitForConversion(false);  // makes it async
  sensor5.setWaitForConversion(false);  // makes it async
  sensor6.setWaitForConversion(false);  // makes it async
  
  myNex.begin(9600); // Begin the object with a baud rate of 9600
                     // If no parameter was given in the begin(), the default baud rate of 9600 will be used
  delay(500);        // Wait for Nextion to start
  myNex.writeStr("page 1"); // For synchronizing Nextion page in case of reset to Arduino
  delay(50);
  //myNex.lastCurrentPageId = 1; // At the first run of the loop, the currentPageId and the lastCurrentPageId
                               // must have different values, due to run the function firstRefresh()
  EEPROM.get(eeAddrOffset, offset);
  EEPROM.get(eeAddrWWsoll, TsollWW);
  EEPROM.get(eeAddrHZhyst, hzHyst);
  EEPROM.get(eeAddrWWhyst, wwHyst);
  EEPROM.get(eeAddrSommer, sommer);
  
  EEPROM.get(eeAddrFehler1, fehlerspeicher1);
  EEPROM.get(eeAddrFjahr1, fJahr1);
  EEPROM.get(eeAddrFmonat1, fMonat1);
  EEPROM.get(eeAddrFtag1, fTag1);
  EEPROM.get(eeAddrFstunde1, fStunde1);
  EEPROM.get(eeAddrFminute1, fMinute1);
  EEPROM.get(eeAddrFsekunde1, fSekunde1);

  EEPROM.get(eeAddrFehler2, fehlerspeicher2);
  EEPROM.get(eeAddrFjahr2, fJahr2);
  EEPROM.get(eeAddrFmonat2, fMonat2);
  EEPROM.get(eeAddrFtag2, fTag2);
  EEPROM.get(eeAddrFstunde2, fStunde2);
  EEPROM.get(eeAddrFminute2, fMinute2);
  EEPROM.get(eeAddrFsekunde2, fSekunde2);
  
  // Start up the library
  sensor1.begin();
  sensor2.begin();
  sensor3.begin();
  sensor4.begin();
  sensor5.begin();
  sensor6.begin();
  
  timer = millis();               // Timer für das Display
  timerSwitchZustand = millis();  // Timer seit wann der aktuelle Zustand läuft.
  timerSensorRefresh = millis();  // Timer zur Auslesen der Temperaturwerte 
  timerCalc = millis();           // Timer zur Neuberechnung der aktuellen Solltemperaturen aus der Heizkurve
  timerTaktsperre = millis();     // Timer für die Taktsperre
  timerAnf = millis();            // Timer zur Neuberechnung ob eine Anforderung zur Warmwasserbereitung vorliegt
  timerZykluszeit = millis();     // Timer zur Abfrage der aktuellen Arduino Programm Zykluszeit (war zur Fehlersuche)
  

  pinMode(hdPin, INPUT_PULLUP);  // Lege den Interruptpin als Inputpin mit Pullupwiderstand fest
  pinMode(ndPin, INPUT_PULLUP);

  //attachInterrupt(digitalPinToInterrupt(hdpInterruptPin), hdFehler, RISING);
  //attachInterrupt(digitalPinToInterrupt(ndpInterruptPin), ndFehler, RISING);
   
  pinMode(durchflussPin, INPUT);
  pinMode(evuPin, INPUT_PULLUP);
  
  pinMode(brunnenPin, OUTPUT);
  pinMode(hzPin, OUTPUT);
  pinMode(wwPin, OUTPUT);      
  pinMode(kompPin, OUTPUT);  

  digitalWrite(brunnenPin, LOW);  //active high
  digitalWrite(hzPin, HIGH);      //active low 
  digitalWrite(wwPin, HIGH);      //active low
  digitalWrite(kompPin, HIGH);    //active low

  if(sommer == true){ // Wenn Sommer dann setze die standbyzeit auf 4h
    standbyTime = 14400000;
  }
  else{
    standbyTime = 600000; // im Winter 10 Minuten
  }

/*  if(digitalRead(hdpInterruptPin) == HIGH){  // Interuppt kommt nur bei RISING, daher muss einmal überprüft werden.
    zustand = 9;
    fehlercode = 3;
    fehlerspeichern();
    }  
  if(digitalRead(ndpInterruptPin) == HIGH){
    zustand = 9;
    fehlercode = 2;
    fehlerspeichern();
    }  */
}

void loop(){


  myNex.NextionListen(); // Auf Display-Events hören. Trigger...

  
//---------Hauptprogramm-----------------------------------------------------------------------------------------------

  switch(zustand){
    case 0:  // Pumpentest nach neustart    
      if((millis() - timerSwitchZustand) > 30000){ // nach 30 sec auf "Werte ermitteln" schalten 
        zustand = 1; 
        timerSwitchZustand = millis(); // Timer für nächsten Case zurücksetzen
        digitalWrite(brunnenPin, LOW); // Bunnenpumpe ausschalten
        break;       
      }
      if(digitalRead(brunnenPin) == LOW){ //Wenn die Brunnenpumpe aus ist (erster Durchlauf), Brunnenpumpe einschalten
        digitalWrite(brunnenPin, HIGH);                              
      }
      if((millis() - timerSwitchZustand) > 20000){ // nach 20 sec Durchfluss prüfen
        if(paddelwaechter != 1){
          digitalWrite(brunnenPin, LOW); // Bunnenpumpe ausschalten
          zustand = 9; // Fehler
          fehlercode =  1; // kein Durchfluss
          fehlerspeichern();
          break;       
        }
        break;
      }  
      break;
      
    case 1:  // Werte ermitteln,
      if(evuSperre == true){ // Wenn EVU Sperre, dann Zustand EVU Sperre;
        digitalWrite(hzPin, HIGH);
        zustand = 11;
        break;
      }      
      if(digitalRead(hzPin) == HIGH){  //Heizungspumpe einschalten
        digitalWrite(hzPin, LOW);                              
      }
      if((millis() - timerSwitchZustand) > 90000){ // nach 90 sec RL bewerten
        if(hdpresostat == true){ // Prüfen ob HD oder ND Fehler
          hdFehler();
          break;
          }
        if(ndpresostat == true){
          ndFehler();
          break;
          }  
                 
        if(TistRL < TsollRLmO - hzHyst){ // Anforderung Heizung ermitteln
          anfHZ = true;
        }
        
        if(anfWW == true){  // wenn Warmwasseranforderung dann gleich weiter in Spülen vor WW bereitung
          zustand = 3;
          digitalWrite(brunnenPin, HIGH); //Brunnenpumpe einschalten
          timerSwitchZustand = millis(); // Timer für nächsten Case zurücksetzen
          digitalWrite(hzPin, HIGH);   //Heizungspumpe ausschalten
          break;
          }
        
        if(anfHZ == true){
          zustand = 2;
          digitalWrite(brunnenPin, HIGH); // Brunnenpumpe einschalten
          timerSwitchZustand = millis(); // Timer für nächsten Case zurücksetzen
          break;  
        }
        else{ // sonst case 15, Primärkreis Spülen zur Vermeidung von Luftblasen und dann weiter in Standby
          zustand = 15;
          timerSwitchZustand = millis(); // Timer für nächsten Case zurücksetzen
          digitalWrite(brunnenPin, HIGH); // Brunnenpumpe einschalten
          if(TistRL > TsollRLmO){  // Wenn Rücklauf höher als RL soll, dann wird die Umwälzpumpe während dem Standby nicht benötigt.
            digitalWrite(hzPin, HIGH);    // Heizungspumpe ausschalten
          }

          break;     
        }
      }
      
      if((millis() - timerSwitchZustand) > 15000){ // nach 15 sec RL prüfen ob Paddelwächter wieder auf 0 steht und nicht klemmt
        if(paddelwaechter == true){
          zustand = 9; //Fehler
          fehlercode = 6;
          fehlerspeichern();
           digitalWrite(hzPin, HIGH);    // Heizungspumpe ausschalten 
           digitalWrite(brunnenPin, LOW); // Brunnenpumpe auschalten 
           break;         
        }
      }            
      break;

    case 2:  // Spülen v. Kompressorstart Heizen, Brunnenpumpe und HeizungsPumpe sind bereits eingeschalten.
      if(evuSperre == true){ //Wenn EVU Sperre, dann Zustand EVU Sperre;
        digitalWrite(hzPin, HIGH); // Heizungspumpe ausschalten
        digitalWrite(brunnenPin, LOW);  // Heizungspumpe ausschalten
        zustand = 11;
        break;    
      } 
      if((millis() - timerSwitchZustand) > 30000){ // wenn Timer abgelaufen springen in heizen
        if(hdpresostat == true){ // Prüfen ob HD oder ND Fehler
          hdFehler();
          break;
          }
        if(ndpresostat == true){
          ndFehler();
          break;
          }         
        zustand = 5;
        digitalWrite(kompPin, LOW); // Kompressor Einschalten
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
        break;
      }
      if((millis() - timerSwitchZustand) > 20000){ // nach 20 sec Durchfluss prüfen
        if(paddelwaechter != 1){
          digitalWrite(brunnenPin, LOW); // Bunnenpumpe ausschalten
          digitalWrite(hzPin, HIGH); // Heizungspumpe ausschalten
          zustand = 9; // Fehler
          fehlercode =  1; // kein Durchfluss
          fehlerspeichern();
          timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
          break;                 
        }
      }
      if((millis() - timerSwitchZustand) > 25000){ // nach 25 sec. Heizungsanforderung prüfen
        if(anfHZ == false){
          zustand = 1;
          digitalWrite(brunnenPin, LOW); // Bunnenpumpe ausschalten          
          timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
          break;
        }
      }
       
      break;  

    case 3:  // Spülen v. Kompressorstart Warmwasserbereiten, Brunnen Pumpe ist bereits eingeschalten
      if(evuSperre == true){ //Wenn EVU Sperre, dann Zustand EVU Sperre;
        digitalWrite(hzPin, HIGH); // Heizungspumpe ausschalten
        digitalWrite(brunnenPin, LOW);  // Heizungspumpe ausschalten
        zustand = 11;
        break;    
      }       
      if((millis() - timerSwitchZustand) > 30000){ // wenn Timer abgelaufen springen in WW
        if(hdpresostat == true){ //Prüfen ob HD oder ND Fehler
          hdFehler();
          break;
          }
        if(ndpresostat == true){
          ndFehler();
          break;
          }         
        zustand = 6;
        digitalWrite(kompPin, LOW); //Kompressor einschalten
        digitalWrite(wwPin, LOW); // WW Pumpe einschalten
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
        break;
      }
      if((millis() - timerSwitchZustand) > 20000){ // nach 20 sec Durchfluss prüfen
        if(paddelwaechter != 1){
          digitalWrite(brunnenPin, LOW); // Bunnenpumpe ausschalten
          zustand = 9; // Fehler
          fehlercode =  1; // kein Durchfluss
          fehlerspeichern();
          timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
          break;                 
        }
      }
      if((millis() - timerSwitchZustand) > 25000){ // nach 25 sec. WW Anforderung prüfen
        if(anfWW == false){
          zustand = 1;
          timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
          break;
        }

      }

      break;  
      
    case 4:  // Spülen n. Kompressostop, Brunnenpumpe und HeizungsPumpe laufen bereits.
      if(evuSperre == true){ //Wenn EVU Sperre, dann Zustand EVU Sperre spülen;
        zustand = 12;
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
        break;    
      }   
      if((millis() - timerSwitchZustand) > 60000){
        digitalWrite(brunnenPin, LOW); // BrunnenPumpe ausschalten
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen 
        zustand = 7; //taktsperre
      }      
      break;  
      
    case 5:  // Heizen,
      if(evuSperre == true){
        anfHZ = false;
        zustand = 12; //EVU Sperre Spülen
        digitalWrite(kompPin, HIGH); // Kompressor ausschalten
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
        break;       
      }      
      if(TistRL > (TsollRLmO + hzHyst) && (millis() - timerSwitchZustand) > 120000){ // Ermitteln ob RL Sollwert pus Hysterese erreicht hat, Wartezeit das nach WW Bereitung RL wieder richtige Werte misst.
        anfHZ = false;  // anforderung Heizung zurücksetzen
        }
      if(anfWW == true){ // WW Bereitung hat Priorität daher sofort umschalten auf WW bereiten  
        zustand = 6;
        digitalWrite(hzPin, HIGH);    // Heizungspumpe ausschalten          
        digitalWrite(wwPin, LOW);    // WW Pumpe einschalten     
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen    
        break;
      }
      if(anfHZ == false){   // Wenn Anforderung Heizung false
        if(anfPreWW == true){ // Wenn Anforderung WW mit halber Hysterese vorliegt, dann wechseln in WW Bereitung, da Kompressor eh schon läuft 
          anfWW = true;  
          zustand = 6;
          digitalWrite(hzPin, HIGH);    // Heizungspumpe ausschalten          
          digitalWrite(wwPin, LOW);    // WW Pumpe einschalten     
          timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen  
          break;                   
        }
        zustand = 4;
        digitalWrite(kompPin, HIGH); // Kompressor ausschalten
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
        timerTaktsperre = millis();  // Timer für Taktserre zurückssetzen
      }
      if(paddelwaechter != 1){
        digitalWrite(brunnenPin, LOW); // Bunnenpumpe ausschalten
        digitalWrite(kompPin, HIGH); // Kompressor ausschalten
        digitalWrite(hzPin, HIGH); // Heizungspumpe ausschalten
        zustand = 9; // Fehler
        fehlercode =  1; // kein Durchfluss
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
        fehlerspeichern();
        break;                 
      }
      if(hdpresostat == true){
        hdFehler();
        break;
        }
      if(ndpresostat == true){
        ndFehler();
        break;
        } 
      break;  
            
    case 6:  // Warmwasser
      if(evuSperre == true){
       anfHZ = false;
       zustand = 12; //EVU Sperre Spülen
       digitalWrite(kompPin, HIGH); // Kompressor ausschalten
       digitalWrite(wwPin, HIGH); // WW Pumpe ausschalten  
       digitalWrite(hzPin, LOW); // Heizungspumpe einschalten          
       timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
       break;       
      }   
      if(anfWW == false){
        if(anfHZ == true){
          zustand = 5;
          digitalWrite(wwPin, HIGH); // WW Pumpe ausschalten
          digitalWrite(hzPin, LOW); // HZ Pumpe einschalten  
          timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen   
          break;                       
          }  
        else{
          zustand = 4;
          digitalWrite(kompPin, HIGH); // Kompressor ausschalten
          digitalWrite(wwPin, HIGH); // WW Pumpe ausschalten
          digitalWrite(hzPin, LOW); // Heizungspumpe einschalten
          timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
          timerTaktsperre = millis();  // Timer für Taktserre zurückssetzen
          break;
          }
      }
      if(paddelwaechter != 1){
        digitalWrite(brunnenPin, LOW); // Bunnenpumpe ausschalten
        digitalWrite(kompPin, HIGH); // Kompressor ausschalten
        digitalWrite(wwPin, HIGH); // WW Pumpe ausschalten
        zustand = 9; // Fehler
        fehlercode =  1; // kein Durchfluss
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
        fehlerspeichern();
        break;                 
      }
      if(hdpresostat == true){
        hdFehler();
        break;
        }
      if(ndpresostat == true){
        ndFehler();
        break;
        } 
      break;  

    case 7: // taktsperre
      if((millis() - timerTaktsperre) > 3600000){   // eine h Taktsperre
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
        zustand = 1; // Werte ermitteln
      }
      if(evuSperre == true){
        zustand = 11;
      }
      break;    

    case 8:  // Fehlerspülen
      if(firstloop == true){
        fehlerspeichern();  // Fehler mit Zeit im EPROM speichern
        firstloop = false;
      }
  
      if((millis() - timerSwitchZustand) > 60000){
        digitalWrite(brunnenPin, LOW); // BrunnenPumpe ausschalten
        digitalWrite(hzPin, HIGH); // Heizungspumpe ausschalten
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
        zustand = 9; //Fehler
      }
      break;  

    case 9:  // Fehler

      break;  

    case 10: // 10 min warten
      if(evuSperre == true){
        zustand = 11;
      }    
      if((millis() - timerSwitchZustand) > standbyTime){ // nach 10 min in case 1, Werte ermitteln springen. im Sommer 4h
          zustand = 1;
          digitalWrite(hzPin, LOW);    // Heizungspumpe Pumpe einschalten   
          timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
          break;    
      }
      if(anfWW == true){ // WW Bereitung hat Priorität daher sofort umschalten auf WW bereiten  
        zustand = 3;         
        digitalWrite(brunnenPin, HIGH);    // Brunnen Pumpe einschalten   
        digitalWrite(hzPin, HIGH);    // Umwälzpumpe ausschalten              
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
      }
        break;  
        
    case 11: // EVU Sperre 
      if(evuSperre == false){  // Wenn das EVU Sperre Signal nicht mehr anliegt...
        zustand = 1;   
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen                         
        }
    case 12:  // EVU Sperre Spülen
      if((millis() - timerSwitchZustand) > 60000){
        digitalWrite(brunnenPin, LOW); // BrunnenPumpe ausschalten
        digitalWrite(hzPin, HIGH); //Heizungspumpe ausschalten
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen 
        zustand = 11; //EVU Sperre
      }      
      break;  
    case 13:  // Spülen Systemkontrolle
      if((millis() - timerSwitchZustand) > 60000){
        digitalWrite(brunnenPin, LOW); // BrunnenPumpe ausschalten
        digitalWrite(hzPin, HIGH); //Heizungspumpe ausschalten
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen 
        zustand = 14; // Sytemkontrolle
      }      
      break;     
    case 14:  ///Systemkontrolle            
      break;  
      
    case 15:  // Spülen Primärkreis 
      if((millis() - timerSwitchZustand) > 120000){
        digitalWrite(brunnenPin, LOW); // BrunnenPumpe ausschalten
        timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen 
        zustand = 10; // Standby     
      }   
      break;                  
}  

// Displaykommunikation  -----------------------------------------------------------------------------------------------------    
  aktuelleSeite = myNex.currentPageId; // Sorgt dafür, dass der Arduino weis auf welcher Seite das Display ist
  

  if((millis() - timer) > 1000){
    if(aktuelleSeite == 0){ // Home Inhalsverzeichnis

 
      }
      
    else if(aktuelleSeite == 1){
      myNex.writeNum("x0.val", TistA);     
      myNex.writeNum("x1.val", TistVL);
      myNex.writeNum("x2.val", TistRL);
      myNex.writeNum("x3.val", TsollRLmO);
      myNex.writeNum("x4.val", TistBrRL);
      myNex.writeNum("x5.val", TistWW);   
      myNex.writeNum("x6.val", freeMemory()); 
      myNex.writeNum("x7.val", TistBrVL);   
      myNex.writeNum("x8.val", TsollWW);          

      if(anfHZ == false){ // Wenn keine Anforderung Heizung dann mach die Anzeige grün
        myNex.writeNum("t13.bco", 2016);    
      }
      if(anfHZ == true){ // Wenn Anforderung Heizung dann mach die Anzeige rot
        myNex.writeNum("t13.bco", 63488);    
      }
      if(anfWW == false){ // Wenn keine Anforderung WW dann mach die Anzeige grün
        myNex.writeNum("t14.bco", 2016);    
      }
      if(anfWW == true){ // Wenn Anforderung Heizung dann mach die Anzeige rot
        myNex.writeNum("t14.bco", 63488);    
      }      
      if(evuSperre == false){
        myNex.writeNum("t15.bco", 2016);    
      }
      if(evuSperre == true){
        myNex.writeNum("t15.bco", 63488);    
      } 
      if(sommer == true){ // Wenn "sommer" dann mach die Anzeige gelb und schreibe Sommer
        myNex.writeNum("t22.bco", 65504);   
        myNex.writeStr("t22.txt", "Sommer"); 
        }
      if(sommer == false){  // Wenn nicht "sommer" dann mach die Anzeige blau und schreibe Winter
        myNex.writeNum("t22.bco", 31);  
        myNex.writeStr("t22.txt", "Winter");
        }
        
      if(digitalRead(brunnenPin) == HIGH){ // Wenn Brunnenpumpe läuft --> Anzeige rot
        myNex.writeNum("t23.bco", 63488);    
        }  
      else{
        myNex.writeNum("t23.bco", 2016);                 
      }
        
      if(digitalRead(hzPin) == LOW){ // // Wenn Heizungspumpepumpe läuft --> Anzeige rot
        myNex.writeNum("t24.bco", 63488);    
        }
      else{
        myNex.writeNum("t24.bco", 2016);                 
      }                  
        
      if(digitalRead(wwPin) == LOW){ // // Wenn Warmwasserpumpepumpe läuft --> Anzeige rot
        myNex.writeNum("t25.bco", 63488);    
        } 
      else{
        myNex.writeNum("t25.bco", 2016);                 
      }        
         
      if(digitalRead(kompPin) == LOW){ // // Wenn Kompressor läuft --> Anzeige rot
        myNex.writeNum("t26.bco", 63488);    
        }    
      else{
        myNex.writeNum("t26.bco", 2016);                 
      }

      if(zustand == 8 || zustand == 9){ // Wenn Fehler, dann mache das Zustandsfeld rot
        myNex.writeNum("t12.bco", 63488);
      }
      else{
        myNex.writeNum("t12.bco", 2016);                 
      }    
                                      
      switch(zustand){
        case 0:
          myNex.writeStr("t12.txt", "Selbsttest");  
          break;
        case 1:
          myNex.writeStr("t12.txt", "Werte ermitteln");  
          break;
        case 2:
          myNex.writeStr("t12.txt", "Spuelen v. Heizen");  
          break;
        case 3:
          myNex.writeStr("t12.txt", "Spuelen v. Warmwasser");  
          break;
        case 4:
          myNex.writeStr("t12.txt", "Spuelen n. Kompressorstop");  
          break;
        case 5:
          myNex.writeStr("t12.txt", "Heizen");  
          break; 
        case 6:
          myNex.writeStr("t12.txt", "Warmwasser");  
          break;
        case 7:
          myNex.writeStr("t12.txt", "Taktsperre");  
          break;          
        case 8:
          myNex.writeStr("t12.txt", "Fehler spuelen gesperrt");  
          break; 
        case 9:
          myNex.writeStr("t12.txt", "Fehler gesperrt");  
          break; 
        case 10:
          myNex.writeStr("t12.txt", "Standby ");  
          break; 
        case 11:
          myNex.writeStr("t12.txt", "EVU Sperre");  
          break;     
        case 12:
          myNex.writeStr("t12.txt", "EVU Sperre spuelen");  
          break;  
        case 13:
          myNex.writeStr("t12.txt", "Spuelen Systemkontrolle");  
          break;     
        case 14:
          myNex.writeStr("t12.txt", "Systemkontrolle");  
          break;           
        case 15:
          myNex.writeStr("t12.txt", "Spuelen Primaerkreis");  
          break;                                                                                                           
        } 
      }      
  
    else if(aktuelleSeite == 2){ // Heizung
      
      myNex.writeNum("x0.val", offset);      
      myNex.writeNum("x1.val", hzHyst);

      if(sommer == true){ // Wenn "sommer" dann mach die Anzeige gelb und schreibe Sommer
        myNex.writeNum("b7.bco", 65504);   
        myNex.writeStr("b7.txt", "Sommer"); 
        }
      if(sommer == false){  // Wenn nicht "sommer" dann mach die Anzeige blau und schreibe Winter
        myNex.writeNum("b7.bco", 31);  
        myNex.writeStr("b7.txt", "Winter");
        } 
      }
    else if(aktuelleSeite == 3){ // Uhr         

      }
    else if(aktuelleSeite == 4){ // Warmwasser
      myNex.writeNum("x0.val", TsollWW);            
      myNex.writeNum("x1.val", wwHyst);
      }
    else if(aktuelleSeite == 5){    // Fehlerübersicht
      
      switch(fehlerspeicher1){
        case 0:
          myNex.writeStr("t2.txt", "kein Fehler");
          break;
        case 1:
          myNex.writeStr("t2.txt", "kein Durchfluss primaer");
          break;   
        case 2:
          myNex.writeStr("t2.txt", "Niederdruck ausgeloest");
          break;  
        case 3:
          myNex.writeStr("t2.txt", "Hochduck ausgeloest");
          break;
        case 4:
          myNex.writeStr("t2.txt", "Eingefrierschutz ausgeloest");
          break;  
        case 5:
          myNex.writeStr("t2.txt", "max VL ueberschritten");
          break;  
        case 6:
          myNex.writeStr("t2.txt", "Durchfluss erkannt ohne Pumpe");
          break;  
        case 7:
          myNex.writeStr("t2.txt", "Temperatur Quelle zu gering");
          break; 
        case 8:
          myNex.writeStr("t2.txt", "Temperatur unplausibel");
          break; 
        case 9:
          myNex.writeStr("t2.txt", "Fehlmessung Sensor def.");
          break;                     

      }
      myNex.writeNum("n0.val", fehlerspeicher1);         
      myNex.writeNum("n1.val", fJahr1);  
      myNex.writeNum("n2.val", fMonat1); 
      myNex.writeNum("n3.val", fTag1); 
      myNex.writeNum("n4.val", fStunde1); 
      myNex.writeNum("n5.val", fMinute1); 
      myNex.writeNum("n6.val", fSekunde1);    
      
      switch(fehlerspeicher2){
        case 0:
          myNex.writeStr("t4.txt", "kein Fehler");
          break;
        case 1:
          myNex.writeStr("t4.txt", "kein Durchfluss primaer");
          break;   
        case 2:
          myNex.writeStr("t4.txt", "Niederdruck ausgeloest");
          break;  
        case 3:
          myNex.writeStr("t4.txt", "Hochduck ausgeloest");
          break;
        case 4:
          myNex.writeStr("t4.txt", "Eingefrierschutz ausgeloest");
          break;  
        case 5:
          myNex.writeStr("t4.txt", "max VL ueberschritten");
          break;  
        case 6:
          myNex.writeStr("t4.txt", "Durchfluss erkannt ohne Pumpe");
          break;  
        case 7:
          myNex.writeStr("t4.txt", "Temperatur Quelle zu gering");
          break; 
        case 8:
          myNex.writeStr("t4.txt", "Temperatur unplausibel");
          break; 
        case 9:
          myNex.writeStr("t4.txt", "Fehlmessung Sensor def.");
          break;                                
      }
      myNex.writeNum("n7.val", fehlerspeicher2);         
      myNex.writeNum("n8.val", fJahr2);  
      myNex.writeNum("n9.val", fMonat2); 
      myNex.writeNum("n10.val", fTag2); 
      myNex.writeNum("n11.val", fStunde2); 
      myNex.writeNum("n12.val", fMinute2); 
      myNex.writeNum("n13.val", fSekunde2);           
    }
        else if(aktuelleSeite == 6){
      myNex.writeNum("x0.val", fTistA);     
      myNex.writeNum("x1.val", fTistVL);
      myNex.writeNum("x2.val", fTistRL);
      myNex.writeNum("x3.val", fTsollRLmO);
      myNex.writeNum("x4.val", fTistBrRL);
      myNex.writeNum("x5.val", fTistWW);   
      myNex.writeNum("x6.val", freeMemory()); 
      myNex.writeNum("x7.val", fTistBrVL);   
      myNex.writeNum("x8.val", fTsollWW);          

      if(fanfHZ == false){ // Wenn keine Anforderung Heizung dann mach die Anzeige grün
        myNex.writeNum("t13.bco", 2016);    
      }
      if(fanfHZ == true){ // Wenn Anforderung Heizung dann mach die Anzeige rot
        myNex.writeNum("t13.bco", 63488);    
      }
      if(fanfWW == false){ // Wenn keine Anforderung WW dann mach die Anzeige grün
        myNex.writeNum("t14.bco", 2016);    
      }
      if(fanfWW == true){ // Wenn Anforderung Heizung dann mach die Anzeige rot
        myNex.writeNum("t14.bco", 63488);    
      }      
      if(fevuSperre == false){
        myNex.writeNum("t15.bco", 2016);    
      }
      if(fevuSperre == true){
        myNex.writeNum("t15.bco", 63488);    
      } 
      if(fsommer == true){ // Wenn "sommer" dann mach die Anzeige gelb und schreibe Sommer
        myNex.writeNum("t22.bco", 65504);   
        myNex.writeStr("t22.txt", "Sommer"); 
        }
      if(fsommer == false){  // Wenn nicht "sommer" dann mach die Anzeige blau und schreibe Winter
        myNex.writeNum("t22.bco", 31);  
        myNex.writeStr("t22.txt", "Winter");
        }
        
      if(digitalRead(brunnenPin) == HIGH){ // Wenn Brunnenpumpe läuft --> Anzeige rot
        myNex.writeNum("t23.bco", 63488);    
        }  
      else{
        myNex.writeNum("t23.bco", 2016);                 
      }
        
      if(digitalRead(hzPin) == LOW){ // // Wenn Heizungspumpepumpe läuft --> Anzeige rot
        myNex.writeNum("t24.bco", 63488);    
        }
      else{
        myNex.writeNum("t24.bco", 2016);                 
      }                  
        
      if(digitalRead(wwPin) == LOW){ // // Wenn Warmwasserpumpepumpe läuft --> Anzeige rot
        myNex.writeNum("t25.bco", 63488);    
        } 
      else{
        myNex.writeNum("t25.bco", 2016);                 
      }        
         
      if(digitalRead(kompPin) == LOW){ // // Wenn Kompressor läuft --> Anzeige rot
        myNex.writeNum("t26.bco", 63488);    
        }    
      else{
        myNex.writeNum("t26.bco", 2016);                 
      }  
    }
    else if(aktuelleSeite == 7){  // Systemcontrolle
      if(digitalRead(brunnenPin) == HIGH){ // Wenn Brunnenpumpe läuft --> Anzeige rot
        myNex.writeNum("t0.bco", 63488);    
        }  
      else{
        myNex.writeNum("t0.bco", 2016);                 
      }
        
      if(digitalRead(hzPin) == LOW){ // // Wenn Heizungspumpepumpe läuft --> Anzeige rot
        myNex.writeNum("t1.bco", 63488);    
        }
      else{
        myNex.writeNum("t1.bco", 2016);                 
      }                  
        
      if(digitalRead(wwPin) == LOW){ // // Wenn Warmwasserpumpepumpe läuft --> Anzeige rot
        myNex.writeNum("t2.bco", 63488);    
        } 
      else{
        myNex.writeNum("t2.bco", 2016);                 
      }        
      switch(zustand){
        case 0:
          myNex.writeStr("t5.txt", "Selbsttest");  
          break;
        case 1:
          myNex.writeStr("t5.txt", "Werte ermitteln");  
          break;
        case 2:
          myNex.writeStr("t5.txt", "Spuelen v. Heizen");  
          break;
        case 3:
          myNex.writeStr("t5.txt", "Spuelen v. Warmwasser");  
          break;
        case 4:
          myNex.writeStr("t5.txt", "Spuelen n. Kompressorstop");  
          break;
        case 5:
          myNex.writeStr("t5.txt", "Heizen");  
          break; 
        case 6:
          myNex.writeStr("t5.txt", "Warmwasser");  
          break;
        case 7:
          myNex.writeStr("t5.txt", "Taktsperre");  
          break;          
        case 8:
          myNex.writeStr("t5.txt", "Fehler spuelen gesperrt");  
          break; 
        case 9:
          myNex.writeStr("t5.txt", "Fehler gesperrt");  
          break; 
        case 10:
          myNex.writeStr("t5.txt", "Standby ");  
          break; 
        case 11:
          myNex.writeStr("t5.txt", "EVU Sperre");  
          break;     
        case 12:
          myNex.writeStr("52.txt", "EVU Sperre spuelen");  
          break;  
        case 13:
          myNex.writeStr("t5.txt", "Spuelen Systemkontrolle");  
          break;     
        case 14:
          myNex.writeStr("t5.txt", "Systemkontrolle");  
          break;      
        case 15:
          myNex.writeStr("t12.txt", "Spuelen Primaerkreis");  
          break;                                                                                                               
        }                         
    }
    else if(aktuelleSeite == 8){  //  Zähler Fehlmessungen
      myNex.writeNum("x0.val", TistAdsFcount);     
      myNex.writeNum("x1.val", TistVLdsFcount);
      myNex.writeNum("x2.val", TistRLdsFcount);
      myNex.writeNum("x4.val", TistBrRLdsFcount);
      myNex.writeNum("x5.val", TistWWdsFcount);    
      myNex.writeNum("x7.val", TistBrVLdsFcount);   
    }
    timer = millis();
               
  }
// regelmäßige Berechnungen ------------------------------------------------------------------------------------------------------------------


 
  if((millis() - timerSensorRefresh) > 200){ // springt in die Funktion zum anstoßen der aktuallisierung der Sensorwerte
    sensorRefresh1();
  }
  if((millis() - timerSensorRefresh) > 1000){ // springt in die Funktion zum holen der neuen Sensorwerte
    sensorRefresh2();
    timerSensorRefresh = millis();
  }  
  
  if((millis() - timerCalc) > 10000){ // ruft die Berechnung für die Heizkurve auf
    heizkurveBerechnen();   
    pruefungen(); // führt verschiedene Prüfungen durch
    timerCalc = millis();
  } 
  
  if((millis() - timerAnf) > 1500){ // berechnet ob Anforderung Warmwasser vorliegt
    anforderungWWberechnen();
    timerAnf = millis();
  }
  zykluszeit = millis() - timerZykluszeit;
  timerZykluszeit = millis(); 

  paddelwaechter = digitalRead(durchflussPin);
  evuSperre = digitalRead(evuPin);

  hdpresostat = digitalRead(hdPin);
  ndpresostat = digitalRead(ndPin);

  wdt_reset();   // Setze Watchdog Zähler zurück
}



//Ende Loop ---------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------

void fehlerspeichern(){ // wird bei jedem neuen Fehler 1x aufgerufen um die Fehlermeldung im EEPROM abzuspeichern
  
  EEPROM.put(eeAddrFehler2, fehlerspeicher1); // Fehlerspeicher 1 nach Fehlerspeicher 2 im EEPROM kopieren
  EEPROM.put(eeAddrFjahr2, fJahr1);
  EEPROM.put(eeAddrFmonat2, fMonat1);
  EEPROM.put(eeAddrFtag2, fTag1);
  EEPROM.put(eeAddrFstunde2, fStunde1);
  EEPROM.put(eeAddrFminute2, fMinute1);
  EEPROM.put(eeAddrFsekunde2, fSekunde1);

  
  EEPROM.get(eeAddrFehler2, fehlerspeicher2); //Fehlerspeicher 2 vom Fehlerspeicher 2 des EEPROM holen
  EEPROM.get(eeAddrFjahr2, fJahr2);
  EEPROM.get(eeAddrFmonat2, fMonat2);
  EEPROM.get(eeAddrFtag2, fTag2);
  EEPROM.get(eeAddrFstunde2, fStunde2);
  EEPROM.get(eeAddrFminute2, fMinute2);
  EEPROM.get(eeAddrFsekunde2, fSekunde2);

  fehlerspeicher1 = fehlercode; //aktuellen Fehler auf Fehlerspeicher 1 abspeichern
  fJahr1 = myNex.readNumber("rtc0"); // aktuelle Uhrzeit speichern
  fMonat1 = myNex.readNumber("rtc1");
  fTag1 = myNex.readNumber("rtc2");
  fStunde1 = myNex.readNumber("rtc3");
  fMinute1 = myNex.readNumber("rtc4");
  fSekunde1 = myNex.readNumber("rtc5");

  EEPROM.put(eeAddrFehler1, fehlerspeicher1);// Fehlerspeicher 1 auf EEPROM sichern
  EEPROM.put(eeAddrFjahr1, fJahr1);
  EEPROM.put(eeAddrFmonat1, fMonat1);
  EEPROM.put(eeAddrFtag1, fTag1);
  EEPROM.put(eeAddrFstunde1, fStunde1);
  EEPROM.put(eeAddrFminute1, fMinute1);
  EEPROM.put(eeAddrFsekunde1, fSekunde1);

  fTistA = TistA;  // Temperatur IST Aussen   
  fTistVL = TistVL; // Temperatur IST Vorlauf
  fTistRL = TistRL; // Temperatur IST Rücklauf
  fTsollRL = TsollRL;  // Temperatur Soll Rücklauf
  fTsollRLmO = TsollRLmO;  // Temperatur Soll Rücklauf mit Offset
  fTistBrVL = TistBrVL;   // Temperatur IST vor PrimärWärmetauscher
  fTistBrRL = TistBrRL;  // Temperatur IST nach PrimärWärmetauscher 
  fTistWW = TistWW;  // Temperatur IST Warmwasserspeicher
  fTsollWW = TsollWW; 

  fanfHZ = anfHZ; // Anforderung Heizung
  fanfWW = anfWW; // Anforderung Warmwasser
  fsommer = sommer;
  fevuSperre = evuSperre;
      
}

void heizkurveBerechnen(){  // berechnet die Heizkurve 
  if(sommer == true){ //wenn Sommer, dann setze TsollRLmO auf 10°C 
    TsollRLmO = 100;
  }
  else{  
    TsollRL = -0.00025 * TistA * TistA - 0.225 * TistA + 255;
    TsollRLmO = TsollRL + offset;
    if(TsollRLmO < 200){
      TsollRLmO = 200;
    }
    if(TsollRLmO > 300){
      TsollRLmO = 300;
    }
  }
}

void anforderungWWberechnen(){
  if(TistWW < TsollWW - wwHyst){
    anfWW = true; 
  }
  if(TistWW > TsollWW){
    anfWW = false; 
  }
  if(TistWW < TsollWW - (wwHyst/2)){
    anfPreWW = true; 
  }
  if(TistWW > TsollWW){
    anfPreWW = false; 
  }  
}

void pruefungen(){
  if(unresetfailure == false){
    if(TistBrRL < 50){  //  Brunnenwasser nach WT kleiner 5°C, Kompressor aus Fehler / Fehlerspülen
      if(zustand == 5 || zustand == 6){
        digitalWrite(kompPin, HIGH);
       digitalWrite(wwPin, HIGH);
        digitalWrite(hzPin, LOW);
        zustand = 8;
        fehlercode = 7;
       firstloop = true;
     }
     else{
       digitalWrite(hzPin, HIGH);
       digitalWrite(wwPin, HIGH);
       digitalWrite(brunnenPin, LOW);                  
       zustand = 9;
       fehlercode = 7;
       fehlerspeichern();
     }
     unresetfailure = true;
     timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
    }
    if(TistVL > 650 || TistVL < 50 || TistRL > 650 || TistRL < 50 || TistA > 800 || TistA < -500 || TistWW > 1000 || TistWW < 70){ // prüfen auf unplausieble Temperaturwerte.
      if(zustand == 5 || zustand == 6){
        digitalWrite(kompPin, HIGH);
        zustand = 8;
        fehlercode = 8;
        firstloop = true;
      }
      else{      
        digitalWrite(hzPin, HIGH);
        digitalWrite(wwPin, HIGH);
        digitalWrite(brunnenPin, LOW); 
        zustand = 9;
        fehlercode = 8;
        fehlerspeichern();
      }
      unresetfailure = true;
      timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen     
    }
  
    if(TistAdsFcount > maxFehlmessungen || TistVLdsFcount > maxFehlmessungen || TistRLdsFcount > maxFehlmessungen || TistWWdsFcount > maxFehlmessungen || TistBrVLdsFcount > maxFehlmessungen || TistBrRLdsFcount > maxFehlmessungen){
      if(zustand == 5 || zustand == 6){
        digitalWrite(kompPin, HIGH);
        zustand = 8;
        fehlercode = 9;
        firstloop = true;
      }
      else{      
        digitalWrite(hzPin, HIGH);
        digitalWrite(wwPin, HIGH);
        digitalWrite(brunnenPin, LOW); 
        zustand = 9;
        fehlercode = 9;
        fehlerspeichern();
      }
      unresetfailure = true;
      timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen  
    }
  }  
} 

void sensorRefresh1(){  //stößt aktuallisierte Sensorwerte aus den OneWire Sensoren an
    sensor1.requestTemperatures(); // Send the command to get temperatures
    sensor2.requestTemperatures(); // Send the command to get temperatures
    sensor3.requestTemperatures(); // Send the command to get temperatures
    sensor4.requestTemperatures(); // Send the command to get temperatures
    sensor5.requestTemperatures(); // Send the command to get temperatures
    sensor6.requestTemperatures(); // Send the command to get temperatures 
}
void sensorRefresh2(){  //holt die aktuallisierten Sensorwerte aus den OneWire Sensoren
    mTistVL = sensor1.getTempCByIndex(0) * 10; // jeder Wert wird x 10 genommen, weil das Display keine Gleitkommazahlen verarbeiten kann.
    mTistA = sensor2.getTempCByIndex(0) * 10;
    mTistBrRL = sensor3.getTempCByIndex(0) * 10;
    mTistRL = sensor4.getTempCByIndex(0) * 10;
    mTistWW = sensor5.getTempCByIndex(0) * 10;
    mTistBrVL = sensor6.getTempCByIndex(0) * 10; 

    if(mTistVL < -1240){ // Prüfen ob der Wert richtig angekommen ist -1250 (-125°) ist eine Fehlermeldung
      ++TistVLdsFcount;
    }
    else{
      TistVL = mTistVL; 
    }
    if(mTistA < -1240){ // Prüfen ob der Wert richtig angekommen ist. -1250 (-125°) ist eine Fehlermeldung
      ++TistAdsFcount;
    }
    else{
      TistA = mTistA; 
    }
    if(mTistBrRL < -1240){ // Prüfen ob der Wert richtig angekommen ist -1250 (-125°) ist eine Fehlermeldung
      ++TistBrRLdsFcount;
    }
    else{
      TistBrRL = mTistBrRL; 
    }
    if(mTistRL < -1240){ // Prüfen ob der Wert richtig angekommen ist -1250 (-125°) ist eine Fehlermeldung
      ++TistRLdsFcount;
    }
    else{
      TistRL = mTistRL; 
    }
    if(mTistWW < -1240){ // Prüfen ob der Wert richtig angekommen ist -1250 (-125°) ist eine Fehlermeldung
      ++TistWWdsFcount;
    }
    else{
      TistWW = mTistWW; 
    }
    if(mTistBrVL < -1240){ // Prüfen ob der Wert richtig angekommen ist -1250 (-125°) ist eine Fehlermeldung
      ++TistBrVLdsFcount;
    }
    else{
      TistBrVL = mTistBrVL; 
    }          
            
}

//  Funktionen für die touch buttons am Nextion
void trigger1(){ //offset -0,5 °C
  offset = offset-5;
  EEPROM.put(eeAddrOffset, offset);
}

void trigger2(){  //offset +0,5 °C
  offset = offset+5;
  EEPROM.put(eeAddrOffset, offset);
}

void trigger3(){  //offset und Heizungshyserese zurücksetzen
  offset = 0;
  EEPROM.put(eeAddrOffset, offset);
  hzHyst =15;
  EEPROM.put(eeAddrHZhyst, hzHyst);
}


void trigger5(){ //HZ Hysterese -
  hzHyst = hzHyst-1;
  EEPROM.put(eeAddrHZhyst, hzHyst);  
}

void trigger6(){ //HZ Hysterese +
  hzHyst = hzHyst+1;
  EEPROM.put(eeAddrHZhyst, hzHyst);    
}

void trigger7(){ //WW Soll -
  TsollWW = TsollWW-5;
  EEPROM.put(eeAddrWWsoll, TsollWW);   
}

void trigger8(){ //WW Soll +
  TsollWW = TsollWW+5;
  EEPROM.put(eeAddrWWsoll, TsollWW);   
}

void trigger9(){ //WW Hysterese -
  wwHyst = wwHyst-5;
  EEPROM.put(eeAddrWWhyst, wwHyst);   
}

void trigger10(){ //WW Hysterese +
  wwHyst = wwHyst+5;
  EEPROM.put(eeAddrWWhyst, wwHyst);    
}

void trigger11(){ //WW zurücksetzen 
  TsollWW = 470;
  EEPROM.put(eeAddrWWsoll, TsollWW);
  wwHyst = 50;
  EEPROM.put(eeAddrWWhyst, wwHyst);  
}

void trigger12(){ //Fehler qutieren
  zustand = 0;
  timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
  unresetfailure = false;
  
}

void trigger13(){ // Toggle Sommer  / Winter
  if(sommer == true){
    sommer = false;
    standbyTime = 600000;
  }
  else{
    sommer = true;
    standbyTime = 14400000;
  }
    EEPROM.put(eeAddrSommer, sommer);  
}

void trigger14(){ // Systemkontrolle an

  if(zustand == 5 || zustand ==6){
    digitalWrite(kompPin, HIGH); // Kompressor ausschalten
    digitalWrite(wwPin, HIGH); // WW Pumpe ausschalten
    digitalWrite(hzPin, LOW); // umwälzpumpe einschalten   
    timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen   
    zustand = 13; // Spülen Systemkontrolle
  }
  else if(zustand == 13){
    
  }
  else{
    digitalWrite(brunnenPin, LOW); // Brunnenpumpe ausschalten
    digitalWrite(wwPin, HIGH); // WW Pumpe ausschalten
    digitalWrite(hzPin, HIGH); // Umwälzpumpe ausschalten    
    zustand = 14; // Systemkontrolle
    timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
  }
 
}
void trigger15(){ // Systemkontrolle aus

  if(zustand == 14){
    digitalWrite(brunnenPin, LOW); // Brunnenpumpe ausschalten
    digitalWrite(wwPin, HIGH); // WW Pumpe ausschalten
    digitalWrite(hzPin, HIGH); // Umwälzpumpe ausschalten    
    zustand = 0; // 
    timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
  } 
}
void trigger16(){ // Brunnenpumpe an

  if(zustand == 14){
    digitalWrite(brunnenPin, HIGH); // Brunnenpumpe einschalten
  } 
}
void trigger17(){ // Brunnenpumpe aus

  if(zustand == 14){
    digitalWrite(brunnenPin, LOW); // Brunnenpumpe ausschalten
  } 
}
void trigger18(){ // Umwälzpumpe an

  if(zustand == 14){
    digitalWrite(hzPin, LOW); // Umwälzpumpe einschalten
  } 
}
void trigger19(){ // Umwälzpumpe aus

  if(zustand == 14){
    digitalWrite(hzPin, HIGH); //  ausschalten Umwälzpumpe
  } 
}
void trigger20(){ // WW Pumpe an

  if(zustand == 14){
    digitalWrite(wwPin, LOW); // WW pumpe einschalten
  } 
}
void trigger21(){ // WW aus

  if(zustand == 14){
    digitalWrite(wwPin, HIGH); // WW ausschalten
  } 
}
//-----------------------interrupts--------------------------------------------------------------

void hdFehler(){
    if(unresetfailure == false){
      digitalWrite(kompPin, HIGH);    // Kompressor ausschalten
      if(digitalRead(wwPin) == LOW){ //Wenn WW Pumpe eingeschalten
        digitalWrite(wwPin, HIGH);    // WW Pumpe ausschalten
        digitalWrite(hzPin, LOW);    // Heizungspumpe einschalten
      }  
      zustand = 8; // Fehler Spülen 
      fehlercode = 3; // HD Fehler 
      firstloop = true;
      timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
      unresetfailure = true;
    }
}


void ndFehler(){ 
    if(unresetfailure == false){
      digitalWrite(kompPin, HIGH);    // Kompressor ausschalten
      if(digitalRead(wwPin) == LOW){ //Wenn WW Pumpe eingeschalten
        digitalWrite(wwPin, HIGH);    // WW Pumpe ausschalten
        digitalWrite(hzPin, LOW);    // Heizungspumpe einschalten
      }  
      zustand = 8; // Fehler Spülen 
      fehlercode = 2; // ND Fehler 
      firstloop = true;
      timerSwitchZustand = millis();  // Timer für nächsten Case zurücksetzen
      unresetfailure = true;
    }
}
