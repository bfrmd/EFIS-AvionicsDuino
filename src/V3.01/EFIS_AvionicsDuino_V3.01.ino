/****************************************************************************************************************************************************
                                                      EFIS_AvionicsDuino_V3.0
******************************************************************************************************************************************************
    
  EFIS_Avionicsduino_V3.0 is free software    
  MIT License (MIT)
  
  Copyright (c) 2024 AvionicsDuino - benjamin.fremond@avionicsduino.com
  Contributors : Benjamin Frémond - Gabriel Consigny - Michel Riazuelo
  https://avionicsduino.com/index.php/en/efis-2/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
 *****************************************************************************************************************************/

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                Connexions physiques avec la carte Teensy 4.1
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// Ecran : module TFT LCD RIVERDI (ref. RVT43HLTFWN00), avec contrôleur RA8875 Adafruit(ref. Product ID: 1590)
// ---------------- RA8875 --------------------- Teensy 4.1 ------------------------
//                   GND           --------->      GND 
//                   VIN           --------->   + 5 Volts
//                    CS           --------->   Pin 0 CS1
//                   MISO          --------->   Pin 1 MISO1
//                   MOSI          --------->   Pin 26 MOSI1
//                   SCK           --------->   Pin 27 SCK1
//             Tout le reste       --------->   Non connecté
//---------------------------------------------------------------------------
// Encodeur rotatif optique GrayHill :
// ---------------GrayHill------------------------Teensy 4.1------------------------
//                  1    ---------------------->   GND
//                  2    ---------------------->   GND
//                6 Vcc  ---------------------->   3.3 v 
//                3 SW   ---------------------->   Pin 35 Bouton central
//                 4 B   ---------------------->   Pin 37 Phase B
//                 5 A   ---------------------->   Pin 36 Phase A
//---------------------------------------------------------------------------
// Capteur de température intérieure (oneWire DS18B20) : 
// -------------DS18B20------------------------Teensy 4.1------------------------
//                GND     ------------------->   GND
//             Alim +3.3v ------------------->   + 3.3 volts (courant : 7 mA)
//              Signal   -------------------->   pin 2 + résistance pull up de 4,7k vers 3.3 volts
//---------------------------------------------------------------------------
// Capteurs de pression AMS5915 (1500A pour l'altitude, 050D pour la vitesse air, il sont exploités par une adaptation de cette library : https://github.com/bolderflight/ams5915-arduino
// -------------1500A------------------------Teensy 4.1------------------------
//                GND     ------------------->   GND
//            Alim +3.3v  ------------------->   + 3.3 volts (courant : 4 mA)
//                SDA   --------------------->   SDA (pin 18) + résistance pull up de 4,7k vers 3.3 volts
//                SCL   --------------------->   SCL (pin 19) + résistance pull up de 4,7k vers 3.3 volts
//           Adresse : 0x28

// -------------0050D------------------------Teensy 4.1------------------------
//                GND     ------------------->   GND
//            Alim +3.3v   ------------------>   + 3.3 volts (courant : 4 mA)
//                SDA   --------------------->   SDA1 (pin 17) + résistance pull up de 4,7k vers 3.3 volts
//                SCL   --------------------->   SCL1 (pin 16) + résistance pull up de 4,7k vers 3.3 volts
//           Adresse : 0x28
//---------------------------------------------------------------------------
// Le transceiver CAN MCP 2562 EP (CAN 2.0) :
//-------------- MCP 2562 ------------------------ Teensy 4.1 ------------------------
//             Pin1 TXD ------------------------> Pin 31 CTX3
//             Pin2 GND ------------------------> GND
//             Pin3 VDD ------------------------> + 5V
//             Pin4 RXD ------------------------> Pin 30 CRX3
//             Pin5 Vio ------------------------> + 3.3V
//             Pin6 CAN LOW --------------------> sortie CanBus
//             Pin7 CAN HIGH -------------------> Sortie CanBus
//             Pin8 Standby --------------------> GND

//---------------------------------------------------------------------------
// Le module AHRS Avionics Duino est connecté au CAN Bus
// --------------------------------------------------------------------------
               
//----------------------------------------------------------------------------------------------------------------
// L'heure locale est mémorisée dans la SRTC de la carte Teensy 4.1 par une pile lithium 3v CR2032 entre GND et VBAT.
//----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
// Une carte micro SD formatée ExFAT et contenant le fichier MTBLANC.out doit être insérée dans le lecteur de carte
//-----------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  Inclusions des bibliothèques et fichiers externes
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <EEPROM.h>
#include <SPI.h> 
#include <RA8875.h> 
#include <OneWire.h> 
#include <TimeLib.h>
#include <SD.h> 
#include <QuadEncoder.h>
#include <Adafruit_GFX.h>  // Pour les fontes, il faut télécharger cette library https://github.com/mjs513/ILI9341_fonts et l'installer dans le dossier des libraries Teensy. 
#include <font_Arial.h>
#include <font_ArialBold.h>
#include <TeensyTimerTool.h> 
using namespace TeensyTimerTool; 
#include "HorizArt.h"
#include "HorizArtParamGen.h"
#include "AMS5915_simplified.h"
#include <FlexCAN_T4.h>

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Création des objets
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
RA8875 tft = RA8875(RA8875_CS1, 255, RA8875_MOSI1, RA8875_SCLK1, RA8875_MISO1); // Crée l'objet RA8875 "tft", de classe RA8875, initialisé dans le setup
OneWire  ds(2);  
AMS5915_simplified AMS5915_050D(Wire, 0x28, AMS5915_simplified::AMS5915_0050_D); 
AMS5915_simplified AMS5915_1500A(Wire1, 0x28, AMS5915_simplified::AMS5915_1500_A); 
HorizArt Horizon(&tft); 
QuadEncoder encoder(1, 36, 37, 1);  // Canal 1, Phase A (pin36), PhaseB(pin37), pullups nécessaire avec l'encodeur GrayHill(1.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CAN_Module_EFIS;
File file;
PeriodicTimer TimerClock(TCK);

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Déclarations des variables et constantes globales
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// ******************************************* Déclaration du typedef d'une structure, puis d'un tableau d'éléments de cette structure, définissant l'arborescence des menus ********************************************************
/* La structure des options de menus et l'algorithme de traitement de ces options ont été conçus de telle façon qu'il soit très simple de faire évoluer ces menus, de modifier, de supprimer ou de rajouter des options.
   Voir le schéma du système de menus : https://i.imgur.com/oD18FKH.jpg.
   Voir le schéma de l'algorithme : https://i.imgur.com/eA0QNdb.jpg
   Voir le rôle des index : https://i.imgur.com/c1LsPR6.jpg
*/

typedef struct
{
  char* menuItemLabel;
  uint32_t menuItemNumber;
  char menuItemAction;
  uint8_t numberOfSisterOptions;
  uint8_t indexOfParentOption;
} OptionMenu;

OptionMenu menu[] = 
{
  {"FLIGHT", 10, 'S'},
    { " BACK", 100, 'B'},
    { "  QNH", 101, 'P'},
    { "  QFE", 102, 'P'},
    {"ResetG", 103, 'A'},
    {"  QUIT", 104, 'Q'},

  {"   GEN", 11, 'S'},

    {" BACK",  110, 'B'},
    {" Units",  111, 'S'},

        {  " BACK", 1110, 'B'},
        {  " Km/h", 1111, 'A'},
        {  "Knots", 1112, 'A'},
        {  "  mph", 1113, 'A'},
        {  "  hPa", 1114, 'A'},
        { "  inHg", 1115, 'A'},
        {"Celsius", 1116, 'A'},
        {"Farenht", 1117, 'A'},
        { "  QUIT", 1118, 'Q'},

    {  "MgDev", 112, 'P'},
    { "PrsCor", 113, 'P'},
    {"Backlit", 114, 'P'},
    {"Loc-utc", 115, 'P'},
    { "  QUIT", 116, 'Q'},

  {"Timers", 12, 'S'},

    { " BACK", 120, 'B'},
    {" Start", 121, 'A'},
   { " T. Up", 122, 'A'},
   { "T.Down", 123, 'P'},
   { "  QUIT", 124, 'Q'},

  {"  QUIT", 13, 'Q'},

  {" ", 1000000, '\0'} 
};

// ********************************************************************* Autres variables Menus **************************************************************************
bool Menu_open = false;
bool SetVal = false;
uint8_t indexCurrentMenuOption = 0;
int16_t * ptrGen_int16_t = NULL;
float * ptrGen_float = NULL;
uint16_t menuXCoordinate = 0, menuYCoordinate = 254;
uint8_t boxHeight = 18, boxWidth = 53;

// ******************************************************************** Variables Encodeur ***********************************************************************
int EncoderPositionValue;
int EncoderPositionValueOld = 0;
int EncoderPositionValueOldSetVal = 0;
byte Button_Pin = 35;
const uint16_t debounceTime = 500;
volatile unsigned long lastButtonPress;
bool Button_Click = false;

// ************************************************************** Variables EFIS - horizon artificiel ***************************************************************
int16_t QNH, QFE; // QNH et QFE sont exprimés en centièmes de pouce de mercure, donc atmosphère ISA -> 2992 centièmes de inHg (soit 1013.25 hPa)
uint8_t displayBaroSettingUnit = 0; // 0 -> hPa    1 -> In Hg
uint8_t displayTemperatureUnit = 0; // 0 -> °C     1 -> °F
int16_t pressureCorrection; // Correction d'un offset de pression absolue, variable, exprimée en déciPa. Peut être ajustée dans les menus pour corriger l'erreur altimétrique.
float altitudeCorrection = 0.995; // Idem, mais pour "rattraper" une erreur proportionnelle à l'altidude.
float pressure = 101300;
float nonFilteredPressure;
float previousFilteredPressureValue;
float differentialPressure = 0;
float nonFilteredDifferentialPressure;
float previousFilteredDiffPressureValue = 0;
float qnhAltitude = 0;
float qfeAltitude = 0;
float pressureAltitude = 0;
float densityAltitude = 0.0;
float iat = 5.0;
volatile float oat = 5.0;
volatile float relativeHumidity = 60.0;
volatile uint32_t magneticHeading = 0;
float dewPoint = 0;
int16_t indicatedAirSpeed, trueAirSpeed;
int16_t ballOffset = 0;
float altitudeVario, altitudeVarioOld;
float baroVario;
uint32_t baroVarioIntegrationStartTime;
uint16_t baroVarioIntegrationTimeOut = 250;
float previousFilteredBaroVarioValue;
float accG, accGmax = 1.0, accGmin = 1.0;
float previousFilteredAccGValue = 1;
float speedConversionFactor = 1.0;
char strSpeedUnit[] = "Km/h";
int16_t magDev = 0;
float windSpeed, windDirection;

// ******************************************************* Déclaration de 2 variables pour la gestion de l'EEPROM **********************************************
#define EFISeepromAdresse 22
uint16_t  OffsetEFISeepromAdresse = 0;

// ************************************************************ Variables utilisées pour le chronométrage de la boucle principale **********************************************************
int loopCounter = 0;
int numLoopIterationDuration = 0;
float numLoopIteration = 1000.0;
float loopDuration;
unsigned long loopDurationStartTime;

// ****************************************************************** Variables utilisées pour l'exploitation des paramètres reçus de l'AHRS **************************************************
float roll = 0, pitch = 0, accY, AccZ, Vz, trk, satInView; // pour stocker les variables float de l'AHRS
int16_t roundedIntVz, roundedIntTrk;                       // pour stocker les variables entières de l'AHRS

// ************************************************************************* Variables GNSS *******************************************************************************************************************
float altitudeGNSS, altitudeGNSSOld;
uint16_t yearGNSS;
uint8_t monthGNSS;
uint8_t dayGNSS;
uint8_t hourGNSS;
uint8_t minuteGNSS;
uint8_t secondGNSS;

int32_t nanoSecGNSS;
uint32_t timeAccuracyGNSS;


float groundSpeedGNSS, groundSpeedGNSSOld;
bool fixOK = false;
bool flagTimeSet = false;

// ******************************************************* Variables et constantes liées à l'horloge et à la gestion de l'heure *****************************************************************************
#define Chy 33
#define Chx 131
#define Rh 28
byte hourHandLength, minuteHandLength, secondHandLength;
time_t zt;
time_t t_chrono1;
time_t t_chrono2;
time_t t_timer;
bool flag_chrono1 = false;
bool flag_chrono2 = false;
bool flagTimer = false;
int16_t timerDuration = 0;
bool flagDisplayTime = true;
uint16_t hs, ms;
uint8_t ss;
#define numberOfSecondsIn12h 43200
uint16_t angleH, angleM, angleS;
uint16_t angleHold = 0;
uint16_t angleMold = 0;
uint16_t angleSold = 0;
int16_t localTimeCorrection = 1;

// ************************************************************** Variables nécessaires au fonctionnement de la sonde de température intérieure********************************************************************************************
byte dataDS18B20[12];
byte addrDS18B20[8];
float temperatureDS18B20;
uint32_t DS18B20AcquisitionStartTime = 0;
bool flagDS18B20AcquisitionDelayElapsed = true;

// ************************************************************** Variables CAN Bus ***************************************************************
CAN_message_t msgQNHaltitudesVario;
CAN_message_t msgSpeeds_Wind;

// ************************************************ Variables utilisées pour les envois périodiques de données au CAN Bus **********************************
uint32_t periode200msStartTime;      // Variable utilisée pour les envois à 5 Hz
uint16_t periode200msTimeOut = 200;

// ************************************************************** Variables diverses **********************************************************************************************
int16_t backLightBrightness;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           SETUP
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  // *************************************************************************************** Initialisation de la carte microSD ****************************************************************************************************
  bool result = (SD.begin(BUILTIN_SDCARD));

  // *********************************************************************************** Initialisation du bus SPI1 et de l'écran  *******************************************************************************************
  SPI1.setMOSI(26);
  SPI1.setSCK(27);
  SPI1.setMISO(1);
  SPI1.setCS(0);
  SPI1.begin();
  tft.begin(Adafruit_480x272);
  tft.useLayers(true);
  tft.layerEffect(LAYER1);
  tft.writeTo(L1);
  tft.fillRect(0, 0, 800, 480, RA8875_BLACK);
  drawHourGlass(tft.width() / 2, tft.height() / 2);
  tft.writeTo(L2);
  tft.fillRect(0, 0, 480, 272, RA8875_BLACK);
  displayPhoto("MTBLANC2.out", 470,262); // affiche une photo sur la layer 2
  tft.layerEffect(LAYER2);
  tft.setCursor(41, 8); 
  tft.setFont(Arial_24_Bold); 
  tft.setTextColor(RA8875_WHITE); 
  tft.println("EFIS AvionicsDuino v 3.01");
  tft.setCursor(40, 7); 
  tft.setFont(Arial_24_Bold); 
  tft.setTextColor(RA8875_BLACK); 
  tft.println("EFIS AvionicsDuino v 3.01");
  tft.setFontDefault();
  tft.setFontScale (0);
  delay(199);
  tft.setTextColor(RA8875_RED);
  if (result) tft.println("Carte micro SD        : OK");
  else 
  {
    tft.setTextColor(RA8875_GREEN);
    tft.println("Carte micro SD non disponible");
  }
  delay(199);
  tft.setTextColor(RA8875_RED);
  tft.println("LCD                   : OK");
  delay(199);

  // ************************************************************************ Initialisation du CAN bus  *****************************************************************************************************
  CAN_Module_EFIS.begin();
  CAN_Module_EFIS.setBaudRate(500000);
  CAN_Module_EFIS.setMaxMB(16);
  CAN_Module_EFIS.enableFIFO();
  CAN_Module_EFIS.enableFIFOInterrupt();
  CAN_Module_EFIS.onReceive(FIFO, canSniff);
  CAN_Module_EFIS.mailboxStatus();
  delay(199);

  // ********************************************************** Initialisation des messages CAN Bus ****************************************************************************
  msgQNHaltitudesVario.id = 46;
  msgQNHaltitudesVario.len = 8;
  msgSpeeds_Wind.id = 48;
  msgSpeeds_Wind.len = 8;

  // ********************************************************************************* Initialisation des deux capteurs de pression  *******************************************************************************************
  if (AMS5915_050D.begin() < 0) tft.println("Erreur de communication avec le capteur AMS5915-50D");
  else tft.println("Capteur AMS5915-50D   : OK");
  delay(199);
  if (AMS5915_1500A.begin() < 0) tft.println("Erreur de communication avec le capteur AMS5915-1500A");
  else tft.println("Capteur AMS5915-1500A : OK");
  delay(199);
  
  // ****************************************************************************** Initialisation des filtres des capteurs de pression *************************************************************************************
  AMS5915_1500A.readSensor('A');
  previousFilteredPressureValue = AMS5915_1500A.getPressure_Pa(); 
  altitudeVarioOld = int((1 - pow((((previousFilteredPressureValue + pressureCorrection * 10) / 100) / 1013.25), 0.190284)) * 145366.45);
  AMS5915_050D.readSensor('D');
  previousFilteredDiffPressureValue = AMS5915_050D.getPressure_Pa();
  tft.println("Filtres               : OK");
  delay(199);
  
  // ********************************************************************************* Initialisation des voies série  *******************************************************************************************
  Serial.begin(115200);
  tft.println("Serial                : OK");
  delay(199);
  
  // ********************************************************************************* Initialisations des timers *******************************************************************************************
  TimerClock.begin (setflagDisplayTime, 500ms);
  tft.println("Timers                : OK");
  delay(199);
  
  // *************************************************************************** Initialisation de l'encodeur rotatif ********************************************************************************************
  pinMode(Button_Pin, INPUT_PULLUP);
  attachInterrupt(Button_Pin, encoderButtonPress_ISR, RISING);
  encoder.setInitConfig(); 
  encoder.EncConfig.IndexTrigger = ENABLE;
  encoder.EncConfig.INDEXTriggerMode = RISING_EDGE;
  encoder.init();
  tft.println("Encodeur              : OK");
  delay(199);
  Button_Click = false;
  
  // *************************************************************************** Initialisation du capteur de température intérieure ********************************************************************************************
  if ( !ds.search(addrDS18B20))
  {
    tft.println("Capteur de temperature intérieure non disponible");
    ds.reset_search();
    delay(199);
  }
  if (OneWire::crc8(addrDS18B20, 7) != addrDS18B20[7]) tft.println("Le CRC de la ROM du capteur de température intérieure n'est pas valide !");
  else tft.println("CRC ROM DS18B20       : OK");
  delay(199);

  // ********************************************************************************* Initialisation des variables stockées en EEPROM *************************************************************
  EEPROM.get(EFISeepromAdresse + 0, QNH);  
  if ((QNH < 2750) || (QNH > 3150)) // Si cette condition est vraie (le QNH n'est pas dans l'intervalle 2750-3150, soit 931-1067 hPa), c'est a priori que l'EEPROM n'a pas été utilisée à cette adresse.
                                   // Il faut donc initialiser toutes les variables, puis les inscrire ensuite dans l'EEPROM
                                   // Si par la suite on règle volontairement le QNH à une valeur inférieure à 2750 (environ 930 hPa), toutes les valeurs stockées en EEPROM seront réinitialisées au prochain cycle.
  {
    QNH = 2992; 
    magDev = 0;  
    localTimeCorrection = 1;
    QFE = 2992; 
    pressureCorrection = -20; 
    backLightBrightness = 4;  
    accGmax = 1.0;
    accGmin = 1.0;
    speedConversionFactor = 1.0;
    strcpy(strSpeedUnit, "Km/h");
    displayBaroSettingUnit = 0;
    displayTemperatureUnit = 0;

    EEPROM.put(EFISeepromAdresse + 0, QNH);                    // variable codée sur 2 octets
    EEPROM.put(EFISeepromAdresse + 2, magDev);                 // variable codée sur 2 octets
    EEPROM.put(EFISeepromAdresse + 4, localTimeCorrection);    // variable codée sur 2 octets
    EEPROM.put(EFISeepromAdresse + 6, QFE);                    // variable codée sur 2 octets
    // Place ici pour une variable codée sur 2 octets, à l'adresse EFISeepromAdresse + 8
    EEPROM.put(EFISeepromAdresse + 10, pressureCorrection);    // variable codée sur 2 octets
    EEPROM.put(EFISeepromAdresse + 12, backLightBrightness);   // variable codée sur 2 octets
    EEPROM.put(EFISeepromAdresse + 14, accGmax);               // variable codée sur 4 octets
    EEPROM.put(EFISeepromAdresse + 18, accGmin);               // variable codée sur 4 octets
    EEPROM.put(EFISeepromAdresse + 22, speedConversionFactor); // variable codée sur 4 octets
    EEPROM.put(EFISeepromAdresse + 30, strSpeedUnit);          // variable codée sur 5 octets
    EEPROM.put(EFISeepromAdresse + 35, displayBaroSettingUnit);// variable codée sur 1 octet
    EEPROM.put(EFISeepromAdresse + 36, displayTemperatureUnit);// variable codée sur 1 octet
  }
  else 
  {
    EEPROM.get(EFISeepromAdresse + 0, QNH);                    // variable codée sur 2 octets
    EEPROM.get(EFISeepromAdresse + 2, magDev);                 // variable codée sur 2 octets
    EEPROM.get(EFISeepromAdresse + 4, localTimeCorrection);    // variable codée sur 2 octets
    EEPROM.get(EFISeepromAdresse + 6, QFE);                    // variable codée sur 2 octets
    // Place ici pour une variable codée sur 2 octets, à l'adresse EFISeepromAdresse + 8
    EEPROM.get(EFISeepromAdresse + 10, pressureCorrection);    // variable codée sur 2 octets
    EEPROM.get(EFISeepromAdresse + 12, backLightBrightness);   // variable codée sur 2 octets
    EEPROM.get(EFISeepromAdresse + 14, accGmax);               // variable codée sur 4 octets
    EEPROM.get(EFISeepromAdresse + 18, accGmin);               // variable codée sur 4 octets
    EEPROM.get(EFISeepromAdresse + 22, speedConversionFactor); // variable codée sur 4 octets
    EEPROM.get(EFISeepromAdresse + 30, strSpeedUnit);          // variable codée sur 5 octets
    EEPROM.get(EFISeepromAdresse + 35, displayBaroSettingUnit);// variable codée sur 1 octet
    EEPROM.get(EFISeepromAdresse + 36, displayTemperatureUnit);// variable codée sur 1 octet
  }
  tft.println("EEPROM                : OK");
  delay(199);
  
  // *************************************************** Démarrage de l'horloge à partir de l'heure conservée dans le SRTC par la pile *********************************************************************************************
  // Si la pile est morte, il faudra attendre le fix GPS pour avoir une heure.
  setSyncProvider(getTeensy3Time);
  setSyncInterval(900);
  tft.println("SRTC                  : OK");
  delay(199);
  
  // **************************************************************************************** Initialisation et tri des options des menus  **********************************************************************************************************************
  // Pour faciliter la gestion des options de menu (ajouts, suppressions, modifications), ces options sont affichées pendant le setup sur le terminal série, avant et après tri.
  uint8_t nb = numberOfMenuItems();
  Serial.println();
  Serial.println("EFIS AvionicsDuino version 3.01");
  Serial.print ("Nombre de lignes utiles du tableau : "); 
  Serial.println(nb); 
  Serial.println("********Tableau avant tri et initialisation des index**********");
  Serial.println("Index----Label--------------Numero---Action---NbOptS--indexOfParentOption");
  for (byte n = 1; n <= nb + 1; n++)
  {
    Serial.printf("%2d\t", (n - 1));
    Serial.printf("%-8s\t", (menu[n - 1].menuItemLabel));
    Serial.printf("%7d\t\t", (menu[n - 1].menuItemNumber));
    Serial.print(menu[n - 1].menuItemAction); 
    Serial.printf("\t%2d\t", (menu[n - 1].numberOfSisterOptions));
    Serial.printf("%2d", (menu[n - 1].indexOfParentOption));
    Serial.println();
  }
  bubbleSortTable();
  initTable();       // Initialisation des champs "numberOfSisterOptions" et indexOfParentOption"

  Serial.println();
  Serial.println("********Tableau après tri et initialisation des index**********");
  Serial.println("Index----Label--------------Numero---Action---NbOptS--indexOfParentOption");
  for (byte n = 1; n <= nb + 1; n++)
  {
    Serial.printf("%2d\t", (n - 1));
    Serial.printf("%-8s\t", (menu[n - 1].menuItemLabel));
    Serial.printf("%7d\t\t", (menu[n - 1].menuItemNumber));
    Serial.print(menu[n - 1].menuItemAction); 
    Serial.printf("\t%2d\t", (menu[n - 1].numberOfSisterOptions));
    Serial.printf("%2d", (menu[n - 1].indexOfParentOption));
    Serial.println();
  }
  tft.println("Menus                 : OK");
  delay(199);
  
  // ********************************************************************************************** Démarrage de l'objet horizon ********************************************************************************************
  delay(1000);
  Horizon.begin();

  // ******************************************************* Dessin de l'horloge sur le premier plan (layer 1) et initialisations de quelques variables liées à cette horloge **************************************************
  tft.writeTo(L1);
  tft.setTextColor(RA8875_WHITE, RA8875_PINK);
  for (uint16_t i = 0; i < 360; i += 30) tft.drawLineAngle (Chx, Chy, i, Rh + 1, RA8875_WHITE);
  tft.fillCircle (Chx, Chy, Rh - 4, RA8875_PINK);
  hourHandLength = Rh * 2 / 3;
  minuteHandLength = Rh * 8 / 10;
  secondHandLength = Rh * 9 / 10;

  // ********************************************************************************* On arrive presqu'à la fin de SETUP ***************************************************************************************************
  loopDurationStartTime = millis();
  baroVarioIntegrationStartTime = millis();

}// ******************************************************************************************************************FIN DU SETUP***************************************************************************************************

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                             LOOP
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{
  tft.brightness(backLightBrightness * 16 - 1);
  tft.writeTo(L1);

  // ********************************************** Réglage de l'heure sur celle du GNSS de l'AHRS, une fois qu'il diffuse une heure exacte **********************************************************
  if (!flagTimeSet)  
  {
    if ((timeAccuracyGNSS < 1000) && (yearGNSS>0)) // Cold Start : le GNSS commence à diffuser la date et l'heure à partir de 8 satellites en vue,
                                                   // mais il faut attendre environ 14 ou 15 satellites pour que la précision de l'heure soit < 1000 ns, ce qui peut être long...
                                                   // Hot start : la précision peut être d'emblée < 1000 ns, mais la diffusion de la date et et de l'heure peut prendre ensuite plusieurs longues secondes.
                                                   // Il faut donc attendre impérativement que les deux conditions (timeAccuracyGNSS < 1000) && (yearGNSS>0)soient réunies pour régler l'heure.
    {
      setTime(hourGNSS, minuteGNSS, secondGNSS, dayGNSS, monthGNSS, yearGNSS); // L'heure de la bibliothèque Time est désormais l'heure UTC
      Teensy3Clock.set(now()); // Et cette même heure est desormais aussi celle du RTC/SRTC, sauvegardée par la pile.
      flagTimeSet = true; 
    }   
  }
 
  // ****************************************************************************************** Lecture des capteurs de pression ****************************************************************************************************
  AMS5915_050D.readSensor('D');
  AMS5915_1500A.readSensor('A');
  pressure = AMS5915_1500A.getPressure_Pa();
  nonFilteredPressure = pressure;
  pressure = iirFilter (previousFilteredPressureValue, pressure , 0.005);
  previousFilteredPressureValue = pressure;
  differentialPressure = AMS5915_050D.getPressure_Pa();
  nonFilteredDifferentialPressure = differentialPressure;
  differentialPressure = iirFilter (previousFilteredDiffPressureValue, differentialPressure, 0.005);
  previousFilteredDiffPressureValue = differentialPressure;

  // ****************************************************************************************** Lecture du capteur de température intérieure *********************************************************************************************
  if (flagDS18B20AcquisitionDelayElapsed == true)
  {
    ds.reset();
    ds.select(addrDS18B20);
    ds.write(0x44);
    DS18B20AcquisitionStartTime = millis();
    flagDS18B20AcquisitionDelayElapsed = false;
  }
  if ((millis() - DS18B20AcquisitionStartTime) > 1000)
  {
    ds.reset();
    ds.select(addrDS18B20);
    ds.write(0xBE);
    for (byte i = 0; i < 9; i++) dataDS18B20[i] = ds.read();
    temperatureDS18B20 = (float)((dataDS18B20[1] << 8) | dataDS18B20[0]) / 16.0;
    flagDS18B20AcquisitionDelayElapsed = true;
  }

  // ************************************************************************************ Calculs des affichages textuels *********************************************************************************************************

  // --------- calcul des altitudes --------- (Les pressions sont exprimées en Pascals, et filtrées dès l'étape de lecture des capteurs, mais les QNH/QFE sont exprimés en centièmes de pouces de mercure)
  qnhAltitude = (1 - pow((((pressure + pressureCorrection * 10) / 100) / (QNH*0.338646)), 0.190284)) * 145366.45 * altitudeCorrection; // 0.01 inHg = 0.338646 hPa
  qnhAltitude = int((qnhAltitude + 5) / 10.0) * 10;
  qfeAltitude =  (1 - pow((((pressure + pressureCorrection * 10) / 100) / (QFE*0.338646)), 0.190284)) * 145366.45 * altitudeCorrection;
  qfeAltitude = int((qfeAltitude + 5) / 10.0) * 10;
  pressureAltitude = (1 - pow((((pressure + pressureCorrection * 10) / 100) / 1013.25), 0.190284)) * 145366.45 * altitudeCorrection;
  altitudeVario = pressureAltitude;
  pressureAltitude = int((pressureAltitude + 5) / 10.0) * 10;
  iat = temperatureDS18B20;
  if(iat>1000) iat=20.0;
  float pressionSatH20 = 6.1078 * pow(10, ((7.5 * oat) / (237.3 + oat))) * 100;
  float pressionPartielleH2O = pressionSatH20 * relativeHumidity / 100;
  float densiteAir = ((pressure + pressureCorrection * 10) / (287.05 * (oat + 273.15)) * (1 - (0.378 * pressionPartielleH2O / (pressure + pressureCorrection * 10))));
  densityAltitude = (44.3308 - (42.2665 * (pow(densiteAir, 0.234969)))) * 1000 * 3.28084;
  densityAltitude = int((densityAltitude + 5) / 10.0) * 10;
  dewPoint = 243.12 * (log(relativeHumidity / 100) + 17.62 * oat / (243.12 + oat)) / (17.62 - (log(relativeHumidity / 100) + 17.62 * oat / (243.12 + oat)));

  // ----------- Calcul du taux de montée/descente à partir du capteur de pression statique -------------------
  if (millis() >= baroVarioIntegrationStartTime + baroVarioIntegrationTimeOut)
  {
    baroVario =  (altitudeVario - altitudeVarioOld) * 60 * (1000 / baroVarioIntegrationTimeOut) ;
    baroVarioIntegrationStartTime = millis();
    altitudeVarioOld = altitudeVario;
    baroVario = iirFilter (previousFilteredBaroVarioValue, baroVario , 0.05);
    previousFilteredBaroVarioValue = baroVario; 
    if (baroVario >= 0) baroVario = (int(baroVario + 5) / 10) * 10;
    else baroVario = (int(baroVario - 5) / 10) * 10;
  }

  // ----------------------- calcul des vitesses de l'avion, résultat dans l'unité sélectionnée par l'utilisateur --------------------------------------------
  if (differentialPressure > 4.5)
  {
    indicatedAirSpeed = int((sqrtf(21.159183 * differentialPressure) / speedConversionFactor) + 0.5);
    trueAirSpeed = int((86.257392 * sqrtf(differentialPressure * (273.15 + oat) / pressure) / speedConversionFactor) + 0.5);
  }
  else 
  {
    indicatedAirSpeed = 0;
    trueAirSpeed = 0;
  }

  // ----------------------- calcul direction et vitesse du vent (nécessite un magnétomètre PARFAITEMENT calibré et aligné !!!) --------------------------------------------
  float GSE = sin(PI / 180 * trk) * groundSpeedGNSS; // groundSpeedGNSS est exprimée en Km/h
  float GSN = cos(PI / 180 * trk) * groundSpeedGNSS;
  float TASE = sin(PI / 180 * (magneticHeading + magDev)) * trueAirSpeed;
  float TASN = cos(PI / 180 * (magneticHeading + magDev)) * trueAirSpeed;
  float deltaE = TASE - GSE;
  float deltaN = TASN - GSN;
  windSpeed = sqrt(pow(deltaE, 2) + pow(deltaN, 2)); // résultat en km/h
  windDirection = atan2(deltaE, deltaN) * 180 / PI;
  if (windDirection < 0)windDirection = 360 + windDirection;

  // **************************************************************************************** Mise à l'heure de la pendule et des chronos *********************************************************************************************
  if (flagDisplayTime)
  {
    flagDisplayTime = false;
    displayTime();
    time_t i;
    if (flag_chrono1)
    {
      i = now() - t_chrono1;
      tft.setCursor (80, 222); 
      tft.printf("%02u:%02u:%02u",hour(i),minute(i),second(i));
    }
    if (flag_chrono2)
    {
      i = now() - t_chrono2;
      tft.setCursor (180, 222); 
      tft.setTextColor (RA8875_GREEN, RA8875_BLACK); 
      tft.printf("%02u:%02u",minute(i),second(i));
      tft.setTextColor (RA8875_WHITE, RA8875_PINK);
    }
    if (flagTimer)
    {
      i = t_timer - now();
      if (i >= 0) 
      {
        tft.setCursor (260, 222);
        tft.setTextColor (RA8875_WHITE, RA8875_RED);
        tft.printf("%02u:%02u",minute(i),second(i));
        tft.setTextColor (RA8875_WHITE, RA8875_PINK);
      }
      else 
      {
        flagTimer = false;
        tft.setCursor (260, 222);
        tft.print ("     ");
      }
    }
  }

  // ************************************************************************************ Mise à jour des affichages textuels et du vario ****************************************************************************************
  displayTexts();
  displayVario();

  //  ***************************************************************************** Traitement d'un clic sur le bouton de l'encodeur *******************************************************************************************

  if (Button_Click == true)
  {
    if (Menu_open == false)
    {
      Menu_open = true;
      openMenu(0);
    }
    else
    {
      if (SetVal == false)
      {
        switch (menu[indexCurrentMenuOption].menuItemAction)
        {
          case 'Q':
            Menu_open = false;
            closeMenu ();
            break;
          case 'S':
            openMenu(menu[indexCurrentMenuOption].indexOfParentOption);
            break;
          case 'B':
            openMenu(menu[indexCurrentMenuOption].indexOfParentOption);;
            break;
          case 'P':
            SetVal = true;
            encoder.write(0);
            EncoderPositionValueOld = EncoderPositionValue;
            EncoderPositionValueOldSetVal = 0;
            Button_Click = false;
            switch (menu[indexCurrentMenuOption].menuItemNumber)
            { 
              case 101  : ptrGen_int16_t = &QNH;
                          OffsetEFISeepromAdresse =  0;   
                          break;
              case 102  : ptrGen_int16_t = &QFE;  
                          OffsetEFISeepromAdresse =  6;   
                          break;          
              case 112  : ptrGen_int16_t = &magDev;                    
                          OffsetEFISeepromAdresse =  2;   
                          break;
              case 113  : ptrGen_int16_t = &pressureCorrection;        
                          OffsetEFISeepromAdresse = 10;   
                          break;
              case 114  : ptrGen_int16_t = &backLightBrightness;                
                          OffsetEFISeepromAdresse = 12;   
                          break;
              case 115  : ptrGen_int16_t = &localTimeCorrection;     
                          OffsetEFISeepromAdresse =  4; 
                          break;  
              case 123  : ptrGen_int16_t = &timerDuration;             
                          OffsetEFISeepromAdresse = 1000; 
                          flagTimer = false; 
                          tft.setCursor (320, 222); 
                          tft.print ("     "); 
                          break;
              default   : break;
            }
            tft.fillRect((((menu[indexCurrentMenuOption].menuItemNumber) % 10)*boxWidth), (menuYCoordinate - boxHeight + 3), boxWidth, boxHeight - 3, RA8875_RED);
            tft.setCursor(((menu[indexCurrentMenuOption].menuItemNumber) % 10)*boxWidth + (boxWidth / 4), menuYCoordinate + 4 - boxHeight);
            tft.setTextColor(RA8875_WHITE, RA8875_RED);
            tft.setFont(Arial_10);
            if (ptrGen_int16_t == &QNH || ptrGen_int16_t == &QFE)
            {
              if (displayBaroSettingUnit == 1) // alors on va afficher le QNH/QFE en pouces de mercure
              {
                tft.printf("%02u.%02u",(*ptrGen_int16_t/100),(*ptrGen_int16_t%100));
              }
              else                             // sinon on va afficher le QNH/QFE en hPa
              {
               float i    = *ptrGen_int16_t * 0.338646;
               uint16_t j = int (i + 0.5);
               tft.printf("% 5d", j);
              }
            }
            else tft.print(*ptrGen_int16_t);
            tft.setFont(Arial_11);
            break; // break du case 'P'

          case 'A' :
            switch (menu[indexCurrentMenuOption].menuItemNumber)
            {
              case 103  : accGmax = 1.0; accGmin = 1.0; 
                          EEPROM.put(EFISeepromAdresse + 14, accGmax); 
                          EEPROM.put(EFISeepromAdresse + 18, accGmin); 
                          openMenu(indexFirstSisterOption(indexCurrentMenuOption)); 
                          break;
              case 121  : if (flag_chrono1 == false) 
                          {
                            t_chrono1 = now();
                            flag_chrono1 = true;
                          }
                          else 
                          {
                            tft.setCursor(80, 222);
                            tft.print("        ");
                            flag_chrono1 = false;
                          }
                          break;

              case 122  : if (flag_chrono2 == false) 
                          {
                            t_chrono2 = now();  // Si le chrono2 ne tourne pas, on le démarre
                            flag_chrono2 = true;
                          }
                          else 
                          {
                            tft.setCursor(220, 222);  // S'il tourne, on l'efface et on l'arrête
                            tft.print("        ");
                            flag_chrono2 = false;
                          }
                          break;
              case 1111 : speedConversionFactor = 1.0; 
                          strcpy(strSpeedUnit, "Km/h");
                          EEPROM.put(EFISeepromAdresse + 22, speedConversionFactor);
                          EEPROM.put(EFISeepromAdresse + 30, strSpeedUnit);                         
                          openMenu(indexFirstSisterOption(indexCurrentMenuOption));   
                          break;
              case 1112 : speedConversionFactor = 1.852;
                          strcpy(strSpeedUnit, "Kts ");
                          EEPROM.put(EFISeepromAdresse + 22, speedConversionFactor);
                          EEPROM.put(EFISeepromAdresse + 30, strSpeedUnit);   
                          openMenu(indexFirstSisterOption(indexCurrentMenuOption)); 
                          break;
              case 1113 : speedConversionFactor = 1.609;
                          strcpy(strSpeedUnit, "mph ");
                          EEPROM.put(EFISeepromAdresse + 22, speedConversionFactor);
                          EEPROM.put(EFISeepromAdresse + 30, strSpeedUnit);  
                          openMenu(indexFirstSisterOption(indexCurrentMenuOption)); 
                          break;
              case 1114 : displayBaroSettingUnit = 0;
                          EEPROM.put(EFISeepromAdresse + 35, displayBaroSettingUnit);  
                          openMenu(indexFirstSisterOption(indexCurrentMenuOption)); 
                          break;
              case 1115 : displayBaroSettingUnit = 1;
                          EEPROM.put(EFISeepromAdresse + 35, displayBaroSettingUnit);  
                          openMenu(indexFirstSisterOption(indexCurrentMenuOption)); 
                          break;  
              case 1116 : displayTemperatureUnit = 0;
                          EEPROM.put(EFISeepromAdresse + 36, displayBaroSettingUnit);  
                          openMenu(indexFirstSisterOption(indexCurrentMenuOption)); 
                          break; 
              case 1117 : displayTemperatureUnit = 1;
                          EEPROM.put(EFISeepromAdresse + 36, displayBaroSettingUnit);  
                          openMenu(indexFirstSisterOption(indexCurrentMenuOption)); 
                          break;                                                                                       
            }
            break;
        }
      }

      else // On est dans le cas où le clic vient valider la modification d'une valeur numérique
      {
        if (OffsetEFISeepromAdresse != 1000)
        {
          EEPROM.put(EFISeepromAdresse + OffsetEFISeepromAdresse, *ptrGen_int16_t);
        }
        if (timerDuration <= 0) 
        {
          flagTimer = false;
          tft.setCursor (320, 222);
          tft.print ("     ");
        }
        else if (!flagTimer) 
             {
              t_timer = now() + timerDuration * 60;
              flagTimer = true;
             }
        SetVal = false;
        ptrGen_int16_t = NULL; 
        openMenu(indexFirstSisterOption(indexCurrentMenuOption)); // et on revient à la première option du menu en cours
        //closeMenu(); // Ou, parfois plus ergonomique, on ferme le menu. En fait, un bouton séparé pour la fermeture du menu serait souhaitable.
      }
    }
    Button_Click = false;
  }

  // ************************************************************************* Polling de la position de l'encodeur à chaque passage dans Loop ************************************************************************************
  if (Menu_open == true)
  {
    if (SetVal == false)
    {
      EncoderPositionValue = encoder.read();
      tft.setTextColor (RA8875_WHITE, RA8875_PINK);
      if (EncoderPositionValue != EncoderPositionValueOld)
      {
        uint16_t minimum = menu[indexCurrentMenuOption].menuItemNumber;
        minimum = minimum % 10;
        minimum = indexCurrentMenuOption - minimum;
        uint8_t maximum = minimum + menu[minimum].numberOfSisterOptions - 1;
        if ((EncoderPositionValue > maximum - minimum) || (EncoderPositionValue < 0))
        {
          encoder.write(0);
          EncoderPositionValue = encoder.read();
        }
        indexCurrentMenuOption = EncoderPositionValue + minimum;
        tft.drawRect ((EncoderPositionValueOld * boxWidth), menuYCoordinate, boxWidth, boxHeight, BckgrndMenus);
        tft.drawRect ((EncoderPositionValue * boxWidth), menuYCoordinate, boxWidth, boxHeight, RA8875_WHITE);
      }
      EncoderPositionValueOld = EncoderPositionValue;
    }
    else
    {
      EncoderPositionValue = encoder.read();
      if (EncoderPositionValue != EncoderPositionValueOldSetVal)
      {
        int8_t variation = encoder.getHoldDifference();
        if (displayBaroSettingUnit == 0 && (ptrGen_int16_t == &QNH || ptrGen_int16_t == &QFE)) // si on incrémente QNH/QFE en hPa, alors il faut incrémenter x3
        {
          *ptrGen_int16_t = *ptrGen_int16_t + (3 * variation);
        }
        else *ptrGen_int16_t = *ptrGen_int16_t + variation;
        validateValues();
        tft.fillRect((((menu[indexCurrentMenuOption].menuItemNumber) % 10)*boxWidth), (menuYCoordinate - boxHeight + 3), boxWidth, boxHeight - 3, RA8875_RED);
        tft.setCursor(((menu[indexCurrentMenuOption].menuItemNumber) % 10)*boxWidth + (boxWidth / 4), menuYCoordinate + 4 - boxHeight);
        tft.setTextColor(RA8875_WHITE, RA8875_RED);
        tft.setFont(Arial_10);
        if (ptrGen_int16_t == &QNH || ptrGen_int16_t == &QFE)
        {
          if (displayBaroSettingUnit == 1) // alors on va afficher le QNH/QFE en pouces de mercure
          {
            tft.printf("%02u.%02u",(*ptrGen_int16_t/100),(*ptrGen_int16_t%100));
          }
          else                             // sinon on va afficher le QNH/QFE en hPa
          {
           float i    = *ptrGen_int16_t * 0.338646;
           uint16_t j = int (i + 0.5);
           tft.printf("% 5d", j);
          }          
        }
        else tft.print(*ptrGen_int16_t);
        tft.setFont(Arial_11);
      }
      EncoderPositionValueOldSetVal = EncoderPositionValue;
    }
  }

  // ************************************************************************************ Mise à jour de l'horizon artificiel****************************************************************************************  
  ballOffset = (accY * -1 * ballRadius) / 0.27;
  if (ballOffset > ballRadius * 8) ballOffset = ballRadius * 8;
  if (ballOffset < -ballRadius * 8) ballOffset = -ballRadius * 8;  
  Horizon.draw(roll, pitch, 0, ballOffset, Menu_open); // roll en radians, pitch en degrés, la valeur 0 est en réserve pour l'avenir, inutile pour le moment.

  // ************************************************************************* Mesure de la durée de la boucle principale Loop ************************************************************************************
  if (loopCounter >= numLoopIteration)
  {
    numLoopIterationDuration = millis() - loopDurationStartTime;
    loopDuration = float(numLoopIterationDuration / numLoopIteration);
    //tft.setCursor(450,257);
    //tft.print(loopDuration,1);
    loopDurationStartTime = millis();
    loopCounter = 0;
  }
  else
  {
    loopCounter++;
  }
}

// ***************************************************************************************************** Fin de la boucle infinie Loop ***********************************************************************************************

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   ISR de récéption du CAN Bus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void canSniff(const CAN_message_t &msg)
{
  switch (msg.id)
  {
    case 42: oat =              *(float*)(msg.buf+0);
             break;
             
    case 44: relativeHumidity = *(float*)(msg.buf+0);
             break;
             
    case 28: magneticHeading =  *(float*)(msg.buf+0);
             break;
             
    case 20: roll =  *(float*)(msg.buf+0);
             pitch = (*(float*)(msg.buf+4));
             break;
             
    case 22: accY =  *(float*)(msg.buf+0);
             accG =  *(float*)(msg.buf+4);
             accG = -1 * accG / 9.80665;
             if ((accG > accGmax) && (differentialPressure >= 380))
             {
               accGmax = accG;
               EEPROM.put(EFISeepromAdresse + 14, accGmax);
             }
              if ((accG < accGmin) && (differentialPressure >= 380))
             {
               accGmin = accG;
               EEPROM.put(EFISeepromAdresse + 18, accGmin);
             }
             accG = iirFilter (previousFilteredAccGValue, accG, 0.1);
             previousFilteredAccGValue = accG;    
             break;
             
    case 24: roundedIntVz  =   *(uint16_t*)(msg.buf+0);
             trk =   *(float*)(msg.buf+2);
             break;
             
    case 34: yearGNSS =   *(uint16_t*)(msg.buf+0);
             monthGNSS =  *(uint8_t*) (msg.buf+2);
             dayGNSS =    *(uint8_t*) (msg.buf+3);
             hourGNSS =   *(uint8_t*) (msg.buf+4);
             minuteGNSS = *(uint8_t*) (msg.buf+5);
             secondGNSS = *(uint8_t*) (msg.buf+6);
             satInView =  *(uint8_t*) (msg.buf+7);
             break;

    case 38: nanoSecGNSS = *(int32_t*)(msg.buf+0);
             timeAccuracyGNSS = *(uint32_t*)(msg.buf+4);
             break;
             
    case 30: altitudeGNSS =    *(float*)(msg.buf+0); // altitude MSL passée en pieds par l'AHRS.                
             groundSpeedGNSS = *(float*)(msg.buf+4);  // vitesse sol passée en km/h par l'AHRS
             break;
             
    default: break;
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                          Fonctions de mise à jour des affichages textuels et du vario, et envoi de données au CAN bus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void displayTexts()
{
  // ------------- Affichage direction et vitesse du vent -------------------
  tft.setCursor (35, 15); 
  int16_t windDirectionInt = int(windDirection + 0.5);
  tft.printf("%3d", windDirectionInt);
  tft.setCursor (35, 30); 
  int16_t windSpeedInt = int(windSpeed + 0.5);
  tft.printf ("%3d", windSpeedInt); 
  tft.setCursor (65, 30); 
  tft.print (strSpeedUnit); 

  // ------------- Affichage IAT, OAT et Dew Point---------------------------
  if (displayTemperatureUnit == 0)
  {
    tft.setCursor (35, 157); 
    tft.printf("%5.1f%s",iat," C");
    tft.setCursor (35, 172);
    tft.printf("%5.1f%s",oat," C");
    tft.setCursor (35, 202); 
    tft.printf("%5.1f%s",dewPoint," C");
  }
  else
  {
    tft.setCursor (35, 157); 
    tft.printf("%5.1f%s",(iat*1.8+32)," F");
    tft.setCursor (35, 172);
    tft.printf("%5.1f%s",(oat*1.8+32)," F");
    tft.setCursor (35, 202); 
    tft.printf("%5.1f%s",(dewPoint*1.8+32)," F");
  }

  // --------------Affichage Humidité ---------------------
  tft.setCursor (35, 187); 
  tft.printf("%3d%s",int(relativeHumidity+0.5)," %");

  // ------------- Affichage Vitesse indiquée ---------------------------
  tft.setFontScale (1); 
  tft.setTextColor (RA8875_WHITE, RA8875_BLACK);
  tft.setCursor (7, 118);    
  tft.printf ("%3d", indicatedAirSpeed); // elle a été calculée dans l'unité choisie par l'utilisateur
  tft.setCursor (65, 127); 
  tft.setFontScale (0); 
  tft.setTextColor (RA8875_WHITE, RA8875_PINK); 
  tft.print (strSpeedUnit); 

  // ------------- Affichage Route GPS ---------------------------
  if (groundSpeedGNSS < 5.0) tft.setTextColor (RA8875_WHITE, RA8875_RED);
  else tft.setTextColor (RA8875_WHITE, RA8875_BLACK);     // Si la vitesse GPS est trop faible (moins de 5 km/h), roundedIntTrk n'est pas fiable, et va donc s'afficher sur fond rouge au lieu de noir.
  tft.setFontScale (1, 0);
  tft.setCursor (248, 3);  
  roundedIntTrk = int(trk + 0.5);
  tft.printf ("%3d", roundedIntTrk); 

  // --------------Affichage cap magnétique ---------------------
  tft.setTextColor (RA8875_WHITE, RA8875_BLACK);
  tft.setCursor (185, 3);  
  tft.printf ("%3d", magneticHeading); 

  // ------------- Affichage Vitesse propre (TAS) ---------------------------
  tft.setFontScale (0);
  tft.setTextColor (RA8875_WHITE, RA8875_PINK);
  tft.setCursor (30, 97); 
  tft.printf ("%3d", trueAirSpeed);  // elle a été claculée dans l'unité choisie par l'utilisateur
  tft.setCursor (65, 97); 
  tft.print (strSpeedUnit); 

  // --------------- Affichage vitesse GNSS dans l'unité choisie par l'utilisateur ----------------------------------
  tft.setCursor (30, 82); 
  tft.printf ("%3d", int((groundSpeedGNSS / speedConversionFactor)+0.5)); 
  tft.setCursor (65, 82); 
  tft.print (strSpeedUnit); 

  // ------------- Affichage Vz AHRS et Vario barométrique ---------------------------
  tft.setCursor (415, 15);  
  tft.printf ("%5d", roundedIntVz); 
  tft.setCursor (415, 30);
  int16_t baroVarioInt = int(baroVario);
  tft.printf ("%5d", baroVarioInt);

  // ------------- Affichage nombre de satellites ---------------------------
  tft.setCursor (3, 48);   
  tft.print("Nb SAT : "); 
  tft.printf ("%2d", int(satInView)); 

  // ------------- Affichage QNH ---------------------------
  tft.setCursor (415, 205); 
  if (displayBaroSettingUnit == 1)  // alors on va afficher le QNH en pouces de mercure
  {
    tft.printf("%02u.%02u",(QNH/100),(QNH%100));
  }
  else                             // sinon on va afficher le QNH en hPa
  {
    float i    = QNH * 0.338646;
    uint16_t j = int (i + 0.5);
    tft.printf("% 5d", j);
  }

  // ------------- Affichage QFE ---------------------------
  tft.setCursor (415, 220); 
  if (displayBaroSettingUnit == 1) // alors on va afficher le QFE en pouces de mercure
  {
    tft.printf("%02u.%02u",(QFE/100),(QFE%100));
  }
  else                             // sinon on va afficher le QFE en hPa
  {
    float i    = QFE * 0.338646;
    uint16_t j = int (i + 0.5);
    tft.printf("% 5d", j);
  }

  // ------------- Affichage des 5 altitudes ---------------------------
  tft.setFontScale (0, 1); 
  tft.setTextColor (RA8875_WHITE, RA8875_BLACK);
  tft.setCursor (411, 118);
  int16_t qnhAltitudeInt = int(qnhAltitude);
  tft.printf ("%5d", qnhAltitudeInt); 
  tft.setFontScale (0); 
  tft.setCursor (411, 98); 
  tft.printf ("%5d", int(qfeAltitude));
  tft.setTextColor (RA8875_WHITE, RA8875_PINK);
  tft.setCursor (415, 157); 
  tft.printf ("%5d", int(pressureAltitude));
  tft.setCursor (415, 172);   
  if(densityAltitude>99999) densityAltitude=99999;// pour prévenir un bug d'affichage 
  int16_t densityAltitudeInt = int(densityAltitude);
  tft.printf ("%5d", densityAltitudeInt); 
  tft.setCursor (415, 187);
  uint16_t altGps = int((altitudeGNSS + 5) / 10.0) * 10;
  tft.printf ("%5d", altGps); 

  // ------------- Affichage G, Gmin et Gmax ---------------------------
  tft.setCursor (378, 63); 
  tft.print ("G    ");
  tft.printf("%4.1f",accG);
  tft.setCursor (378, 48); 
  tft.print ("max  ");
  tft.printf("%4.1f",accGmax);
  tft.setCursor (378, 78); 
  tft.print ("min  ");
  tft.printf("%4.1f",accGmin);

  // ------------- Envois des données au CAN Bus à 5 Hz
  if ((millis()- periode200msStartTime)>= periode200msTimeOut)
  {
    periode200msStartTime = millis();
    for (uint8_t i = 0; i < 2; i++ )
    {
      msgQNHaltitudesVario.buf[i]  = ((byte*) &QNH)[i];
      msgQNHaltitudesVario.buf[i+2]  = ((byte*) &qnhAltitudeInt)[i];
      msgQNHaltitudesVario.buf[i+4]  = ((byte*) &densityAltitudeInt)[i];
      msgQNHaltitudesVario.buf[i+6]  = ((byte*) &baroVarioInt)[i];
      msgSpeeds_Wind.buf[i]  = ((byte*) &indicatedAirSpeed)[i];
      msgSpeeds_Wind.buf[i+2]  = ((byte*) &trueAirSpeed)[i];
      msgSpeeds_Wind.buf[i+4]  = ((byte*) &windSpeedInt)[i];
      msgSpeeds_Wind.buf[i+6]  = ((byte*) &windDirectionInt)[i];
    }
    CAN_Module_EFIS.write(msgQNHaltitudesVario);
    CAN_Module_EFIS.write(msgSpeeds_Wind);
  } 
  
}

void displayVario()
{
  // affichage des barres en premier 
  int8_t barHeight;
  if (abs(baroVario) > 2000) barHeight = 116;
  else barHeight = abs(int8_t((baroVario * 58) / 1000));
  if (baroVario < 0)
  {
    tft.fillRect(461, 20-14, 9, 116, RA8875_PINK);
    tft.fillRect(461, 136-14, 9, barHeight, RA8875_PURPLE);
    tft.fillRect(461, 136-14 + barHeight, 9, 252 - 136 - barHeight, RA8875_PINK);
    tft.drawRect(460, 136-14, 10, barHeight + 1, RA8875_WHITE);
  }
  else
  {
    tft.fillRect(461, 20-14, 9, 116 - barHeight, RA8875_PINK);
    tft.fillRect(461, 136-14 - barHeight, 9, barHeight, 0b1111111001000000);
    tft.fillRect(461, 136-14, 9, 116, RA8875_PINK);
    tft.drawRect(460, 136-14 - barHeight, 10, barHeight + 1, RA8875_WHITE);
  }
  // puis affichage des graduations
  for (uint16_t i = 49; i <= 272; i = i + 58) tft.drawFastHLine (460, i-14, 6, RA8875_WHITE);
  for (uint16_t i = 20; i <= 272; i = i + 58) tft.drawFastHLine (460, i-14, 10, RA8875_WHITE);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                          Fonctions liées à l'encodeur rotatif et aux menus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void validateValues() // Cette fonction vérifie si certains paramètres modifiés dans les menus restent dans les limites autorisées
{
  if (backLightBrightness > 16) backLightBrightness = 16;       
  if (backLightBrightness < 1) backLightBrightness = 1;
  if (timerDuration > 60) timerDuration = 60; 
  if (timerDuration < 0) timerDuration = 0;
  if(localTimeCorrection < -12) localTimeCorrection = -12;
  if(localTimeCorrection > 12) localTimeCorrection = 12;
}

// ************************************************************************* Routine d'interruption liée au traitement d'une pression sur le bouton *******************************************************************************
void encoderButtonPress_ISR() 
{
  if ((millis() - lastButtonPress) >= debounceTime)
  {
    lastButtonPress = millis ();
    Button_Click = true;
  }
}
// ***************************************************************************************** Procédure d'ouverture du menu ******************************************************************************************************

void openMenu (uint8_t ndx)
{
  indexCurrentMenuOption = ndx;
  tft.fillRect (menuXCoordinate, menuYCoordinate - boxHeight + 2, tft.width() - menuXCoordinate, boxHeight * 2 - 2, BckgrndMenus);
  tft.drawLine(menuXCoordinate + 1, menuYCoordinate - boxHeight + 2, tft.width() - menuXCoordinate, menuYCoordinate - boxHeight + 2, RA8875_WHITE);
  encoder.write(0);
  EncoderPositionValue = 0;
  EncoderPositionValueOld = 0;
  tft.setFont(Arial_11);
  tft.setTextColor (RA8875_WHITE, BckgrndMenus);
  tft.setCursor(menuXCoordinate + 1, menuYCoordinate + 5 - boxHeight);
  if (indexCurrentMenuOption == 0)
  {
    tft.print("GENERAL MENU");
  }
  else
  {
    int8_t i = 0;
    tft.setTextColor (RA8875_WHITE, BckgrndMenus);
    while (menu[indexCurrentMenuOption + i].menuItemNumber != (menu[indexCurrentMenuOption].menuItemNumber) / 10) i--;
    tft.print(menu[indexCurrentMenuOption + i].menuItemLabel);
  }
  for (byte n = ndx; n < ndx + menu[ndx].numberOfSisterOptions; n++)
  {
    tft.setCursor ((boxWidth * (n - ndx)) + 2, menuYCoordinate + 3);
    tft.print(menu[n].menuItemLabel);
  }
  tft.drawRect ((EncoderPositionValue * boxWidth), menuYCoordinate, boxWidth, boxHeight, 0xFFFF); 
}
// ********************************************************************************************* Procédure de fermeture du menu **************************************************************************************************
void closeMenu()
{
  Menu_open = false;
  Button_Click = false;   
  tft.fillRect (menuXCoordinate, menuYCoordinate - boxHeight, tft.width() - menuXCoordinate, boxHeight * 2, RA8875_PINK);
  Horizon.reDraw();
  tft.setFontDefault();
  tft.setFontScale (0);
}
// ************************************************ Cette fonction retourne, à partir d'un index donné quelconque, l'index de la première option soeur (numéro terminant par un zéro) **********************************************
uint8_t indexFirstSisterOption(uint8_t arbitraryIndex)
{
  int8_t i = 0;
  while (menu[arbitraryIndex + i].menuItemNumber > ((menu[arbitraryIndex].menuItemNumber) / 10) * 10) i--;
  return (arbitraryIndex + i);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonctions utilisées lors de l'initialisation des options des menus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void initTable() // Cette fonction calcule et initialise les champs "numberOfSisterOptions" et "indexOfParentOption" de chaque enregistrement, lors du setup.
{
  for (uint8_t i = 0; i < numberOfMenuItems(); i++)
  {
    uint16_t n = menu[i].menuItemNumber;
    int8_t j = 0;
    if ((n / 10) * 10 == n)
    {
      while (menu[i + j].menuItemNumber < n + 10) j++;
      for (uint8_t k = 0; k < j; k++) menu[i + k].numberOfSisterOptions = j;
      j = 0;
    }
    if (menu[i].menuItemAction == 'S')
    {
      while (menu[i + j].menuItemNumber != n * 10) j++;
      menu[i].indexOfParentOption = i + j;
      j = 0;
    }
    if (menu[i].menuItemAction == 'B')
    {
      while (menu[i + j].menuItemNumber != (n / 100) * 10) j--;
      menu[i].indexOfParentOption = i + j;
    }
  }
}

void bubbleSortTable() 
{
  char * text; uint32_t number; char letter;
  for (byte i = numberOfMenuItems() - 1; i > 0; i--)
  {
    for (byte j = 1; j <= i; j++)
    {
      if (menu[j - 1].menuItemNumber > menu[j].menuItemNumber)
      {
        text  = menu[j].menuItemLabel;
        number = menu[j].menuItemNumber;
        letter = menu[j].menuItemAction;
        menu[j].menuItemLabel        = menu[j - 1].menuItemLabel;
        menu[j].menuItemNumber       = menu[j - 1].menuItemNumber;
        menu[j].menuItemAction       = menu[j - 1].menuItemAction;
        menu[j - 1].menuItemLabel    = text;
        menu[j - 1].menuItemNumber   = number;
        menu[j - 1].menuItemAction   = letter;
      }
    }
  }
}

uint8_t numberOfMenuItems()
{
  uint8_t i = 0;
  while (menu[i].menuItemNumber != 1000000) {
    i = i + 1;
  }
  return i;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonction utilisée pour le filtrage des données brutes issues des capteurs
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Fonction de filtre à Réponse Impulsionnelle Infinie (RII)
float iirFilter (float previousFilteredValue, float currentValue , float iirFilterCoefficient)
{
  return previousFilteredValue  * (1 - iirFilterCoefficient) + currentValue * iirFilterCoefficient ;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                       Fonctions liées à la gestion et à l'affichage de l'heure sur l'écran
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
time_t getTeensy3Time() // retourne l'heure du RTC dans une variable de type time_t
{
  time_t n;
  n = Teensy3Clock.get();
  n++;  // Correction d'un bug qui fait perdre une seconde sur l'heure RTC
  return n;
}

void displayTime()
{
  // affiche l'heure locale digitale sur l'horloge
  tft.setFontDefault();
  tft.setFontScale(0);
  tft.setCursor (Chx - 31, Chy + 30);
  zt = now() + (3600 * localTimeCorrection); //TimeLib et le RTC sont réglés sur l'heure UTC, donc zt contient maintenant l'heure locale
  tft.printf("%02u:%02u:%02u",hour(zt),minute(zt),second(zt));
  // Calcul des angles des 3 aiguilles
  hs = zt % numberOfSecondsIn12h;
  ms = zt % 3600;
  ss = zt % 60;
  angleH = 360 * hs / numberOfSecondsIn12h;
  angleM = 360 * ms / 3600;
  angleS = 360 * ss / 60;
  // On efface les anciennes aiguilles
  tft.drawLineAngle (Chx, Chy, angleHold, hourHandLength, RA8875_PINK);
  tft.drawLineAngle (Chx, Chy, angleMold, minuteHandLength, RA8875_PINK);
  tft.drawLineAngle (Chx, Chy, angleSold, secondHandLength, RA8875_PINK);
  // On affiche les nouvelles aiguilles
  tft.drawLineAngle (Chx, Chy, angleH, hourHandLength, 0xFFFF);
  tft.drawLineAngle (Chx, Chy, angleM, minuteHandLength, 0xFFFF);
  tft.drawLineAngle (Chx, Chy, angleS, secondHandLength, RA8875_GREEN);
  // Sans oublier le petit cercle au centre
  tft.fillCircle (Chx, Chy, 3, RA8875_RED);
  //  Et on mémorise les positions des aiguilles
  angleHold = angleH;
  angleMold = angleM;
  angleSold = angleS;
}


void setflagDisplayTime()
{
  flagDisplayTime = true;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonctions utilitaires diverses
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void drawHourGlass(uint8_t x, uint8_t y) // affiche un sablier
{
  tft.fillRect (x - 20, y - 26, 40, 2, RA8875_WHITE);
  tft.fillRect (x - 20, y + 25, 40, 2, RA8875_WHITE);
  tft.drawLine (x - 18, y - 21, x - 18, y + 20, RA8875_WHITE);
  tft.drawLine (x + 18, y - 21, x + 18, y + 20, RA8875_WHITE);
  tft.drawArc (x, y - 14, 14, 1, 195, 270, RA8875_WHITE);
  tft.drawArc (x, y - 14, 14, 1, 90, 165, RA8875_WHITE);
  tft.drawArc (x, y + 14, 14, 1, 270, 345, RA8875_WHITE);
  tft.drawArc (x, y + 14, 14, 1, 15, 90, RA8875_WHITE);
  tft.drawArc (x, y + 9, 32, 1, 335, 25, RA8875_WHITE);
  tft.drawArc (x, y - 9, 32, 1, 155, 205, RA8875_WHITE);
  tft.drawLine(x - 13, y - 14, x - 13, y - 19, RA8875_WHITE);
  tft.drawLine(x - 13, y + 14, x - 13, y + 19, RA8875_WHITE);
  tft.drawLine(x + 13, y - 14, x + 13, y - 19, RA8875_WHITE);
  tft.drawLine(x + 13, y + 14, x + 13, y + 19, RA8875_WHITE);
  tft.drawArc (x, y - 14, 12, 13, 90, 270, RA8875_WHITE);
  tft.drawLine (x - 1, y - 14, x + 1, y - 14, RA8875_WHITE);
  tft.fillRect (x - 1, y - 3, 3, 12, RA8875_WHITE);
  tft.fillTriangle(x, y + 7, x + 8, y + 20, x - 8, y + 20, RA8875_WHITE);
}

void displayPhoto(char str[], uint16_t photoWidth, uint16_t photoHeight) 
{
  file = SD.open(str, FILE_READ);
  uint16_t xOrig, yOrig;
  xOrig = (tft.width()- photoWidth)/2;
  yOrig = (tft.height()- photoHeight)/2;
  if(file)
    {
      unsigned char LSB, MSB;
      uint16_t couleur;
      for (uint16_t j=1; j<=photoHeight; j++)
        {
          for (uint16_t i = 1; i<=photoWidth; i++)
            {
              LSB = file.read();
              MSB = file.read();
              couleur = LSB + (MSB<<8);
              tft.drawPixel(i+xOrig, j+yOrig, couleur);
            }
        }
      file.close();
    } 
  else 
  {
    tft.print("Erreur a l'ouverture du fichier "); 
    tft.println(str); 
    delay(2000);
  }
}
