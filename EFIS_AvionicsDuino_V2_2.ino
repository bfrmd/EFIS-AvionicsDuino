/****************************************************************************************************************************************************
                                                      EFIS_AvionicsDuino_V2.2
******************************************************************************************************************************************************

  Copyright (c) 2023 AvionicsDuino - benjamin.fremond@avionicsduino.com
  Contributors : Benjamin Frémond - Gabriel Consigny - Michel Riazuelo

    EFIS_Avionicsduino_V2.2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
// Module AHRS Avionics Duino :
// ---------------AHRS------------------------Teensy 4.1------------------------
//                GND     ------------------->   GND
//             Alim +5v   ------------------->   + 5 volts
//                TX   ---------------------->   RX5 (pin 21)
//                RX   ---------------------->   TX5 (pin 20)
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

//----------------------------------------------------------------------------------------------------------------
// L'heure UTC est mémorisée dans la SRTC de la carte Teensy 4.1 par une pile lithium 3v CR2032 entre GND et VBAT.
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
QuadEncoder encodeur(1, 36, 37, 1);  // Canal 1, Phase A (pin36), PhaseB(pin37), pullups nécessaire avec l'encodeur GrayHill(1.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CAN_Module_EFIS;
File fichier;
PeriodicTimer TimerSendF; 
OneShotTimer  TimerStopReading;
PeriodicTimer TimerHorloge(TCK);
PeriodicTimer Timer200ms(TCK);

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
  char* label;
  uint32_t numero;
  char action;
  uint8_t nbOptionsSoeurs;
  uint8_t indexOfOm;
} OptionMenu;

OptionMenu menu[] = 
{
  {"FLIGHT", 10, 'S'},
    {" BACK",  100, 'B'},
    {"  QNH",  101, 'P'},
    {"  ------", 102, 'A'},
    {"ResetG", 103, 'A'},
    {"  QUIT", 104, 'Q'},

  {"   GEN", 11, 'S'},

    {" BACK",  110, 'B'},
    {"  Uvit",  111, 'S'},

        {" BACK", 1110, 'B'},
        {" Km/h", 1111, 'A'},
        {"Noeuds", 1112, 'A'},
        {"  QUIT", 1113, 'Q'},

    {"MgDev", 112, 'P'},
    {"  Time", 113, 'S'},

      {" BACK", 1130, 'B'},
      {" Year", 1131, 'P'},
      {"Month", 1132, 'P'},
      {" Day",  1133, 'P'},
      {"Hours", 1134, 'P'},
      {" Min",  1135, 'P'},
      {" Sec",  1136, 'P'},
      {"Valid", 1137, 'A'},
      {"  QUIT", 1138, 'Q'},

    {"PrsCor", 114, 'P'},
    {"  Lum",  115, 'P'},
    {"DatLog", 116, 'S'},

      {" BACK",  1160, 'B'},
      {"   Del", 1161, 'A'},
      {"  Start", 1162, 'A'},
      {"  Stop", 1163, 'A'},
      {" Export", 1164, 'A'},
      {"  QUIT", 1165, 'Q'},
      
    {"Loc-utc",117, 'P'},
    {"  QUIT", 118, 'Q'},

  {"Timers", 12, 'S'},

    {" BACK",  120, 'B'},
    {"Depart", 121, 'A'},
   {"Chrono", 122, 'A'},
   {" Down",  123, 'P'},
   {"  QUIT", 124, 'Q'},

  {"  QUIT", 13, 'Q'},

  {" ", 1000000, '\0'} 
};

//********************************************************************* Autres variables Menus **************************************************************************
bool Menu_Ouvert = false;
bool SetVal = false;
uint8_t indexOptionMenuEnCours = 0;
int16_t * ptrGen = NULL;
uint16_t abscisseMenu = 0, ordonneeMenu = 254;
uint8_t hauteurCase = 18, largeurCase = 53;

//******************************************************************** Variables Encodeur ***********************************************************************
int ValeurPositionEncodeur;
int ValeurPositionEncodeurOld = 0;
int ValeurPositionEncodeurOldSetVal = 0;
byte Pin_Bouton = 35;
const uint16_t debounceTime = 500;
volatile unsigned long lastButtonPress;
bool Bouton_Clic = false;

// ************************************************************** Variables EFIS - horizon artificiel ***************************************************************
int16_t QNH;
int16_t YY, MM, DD, HH, Mn, Sec;
int16_t correctionPression; // Correction d'un offset de pression absolue, variable, exprimée en déciPa. Peut être ajustée dans les menus pour corriger l'erreur altimétrique.
float correctionAltitude = 0.995; // Idem, mais pour "rattraper" une erreur proportionnelle à l'altidude.
float pression = 101300;
float pressionNonF;
float valPressionFiltreePrecedente;
float pressionDiff = 0;
float pressionDiffNonF;
float valPressionDiffFiltreePrecedente = 0;
float tempAMS5915_050D = 0;
float tempAMS5915_1500A = 0;
float altitudeQNH = 0;
float altitudePression = 0;
float altitudeDensite = 0.0;
float iat = 5.0;
volatile float oat = 5.0;
volatile float humiditeRelative = 0.5;
volatile uint32_t capMagnetique = 0;
float dewPoint = 0;
int16_t vitesseIndiquee, vitessePropre;
int16_t offsetBille = 0;
float altitudeVario, altitudeVarioOld;
float vario = 0;
uint32_t topchronoVario;
uint16_t tempsIntegrationVario = 250;
float valVarioFiltreePrecedente = 0;
float accG, accGmax = 1.0, accGmin = 1.0;
float valAccGFiltreePrecedente = 1;
float facteurConversionVitesse = 1.0;
int16_t magDev = 0;
float windSpeed, windDirection;

//******************************************************* Déclaration de 2 variables pour la gestion de l'EEPROM **********************************************
#define EFISeepromAdresse 22
uint16_t  OffsetEFISeepromAdresse = 0;

// ************************************************************ Variables utilisées pour le chronométrage de la boucle principale **********************************************************
int nombreBoucleLoop = 0;
int dureeNdbBoucles = 0;
float Ndb = 1000.0;
float dureeLoop;
unsigned long topHoraire;

// ****************************************************************** Variables utilisées pour l'exploitation des trames reçues de l'AHRS **************************************************
bool dataReady = false;
byte tabTrame[74];
float roll = 0, pitch = 0, psi = 0, AccY, AccZ, Vz, constante, trk, nbSat; // variables float de la trame AHRS
char a; byte b;
float valRollFiltreePrecedente = 0, valPitchFiltreePrecedente = 0, valAccYFiltreePrecedente = 0, valVzFiltreePrecedente = 0, valTrkFiltreePrecedente;
int16_t Vzint;
uint16_t trkEntierArrondi;
bool flagPsiFiable = false;
//bool flagDataLogging = false;
byte ndex = 0;

// *********************************Variables gérant les enregistrements de données (flight data recorder externe/microSD) et l'envoi de données sur le CAN bus ***********************************************************
bool okSendDataToRecord = false;
bool recordStarted = false;
uint8_t cptBlink = 0;
CAN_message_t msg;
CAN_message_t msgRoll;
CAN_message_t msgPitch;
CAN_message_t msgAltiDAltiP;
CAN_message_t msgDateHeureTAS;
boolean okSendCAMmsg = false;

// ************************************************************************* Variables GNSS *******************************************************************************************************************
float altitudeGNSS;

uint16_t anneeGNSS;
uint8_t moisGNSS;
uint8_t jourGNSS;
uint8_t heureGNSS;
uint8_t minuteGNSS;
uint8_t secondeGNSS;

float groundSpeedGNSS, groundSpeedGNSSOld;

bool fixOK = false;
bool heureReglee = false;

//******************************************************* Variables et constantes liées à l'horloge et à la gestion de l'heure *****************************************************************************
#define Chy 192
#define Chx 35
#define Rh 28
byte LaigH, LaigM, LaigS;
time_t zt;
time_t t_chrono1;
time_t t_chrono2;
time_t t_minuteur;
bool flag_chrono1 = false;
bool flag_chrono2 = false;
bool flag_minuteur = false;
int16_t dureeMinuteur = 0;
bool flagAfficheHeure = true;
uint16_t hs, ms;
uint8_t ss;
#define nbs12h 43200
uint16_t angleH, angleM, angleS;
uint16_t angleHold = 0;
uint16_t angleMold = 0;
uint16_t angleSold = 0;
int16_t correctionHeureLocale=0;

// ************************************************************** Variables nécessaires au fonctionnement de la sonde de température intérieure********************************************************************************************
byte dataDS18B20[12];
byte addrDS18B20[8];
float temperatureDS18B20;
uint32_t chronoDS18B20 = 0;
bool flagDelaiAcquisitionDS18B20Termine = true;

// ************************************************************** Variables diverses **********************************************************************************************
int16_t luminosite;
char buf[25];

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
  sablier(tft.width() / 2, tft.height() / 2);
  tft.writeTo(L2);
  tft.fillRect(0, 0, 480, 272, RA8875_BLACK);
  photo("MTBLANC2.out", 470,262); // affiche une photo sur la layer 2
  tft.layerEffect(LAYER2);
  tft.setCursor(151, 8); 
  tft.setFont(Arial_24_Bold); 
  tft.setTextColor(RA8875_WHITE); 
  tft.println("EFIS-DUINO v 2.1");
  tft.setCursor(150, 7); 
  tft.setFont(Arial_24_Bold); 
  tft.setTextColor(RA8875_BLACK); 
  tft.println("EFIS-DUINO v 2.1");
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

  // ***************************************************************************************** Initialisation du CAN bus  *****************************************************************************************************
  CAN_Module_EFIS.begin();
  CAN_Module_EFIS.setBaudRate(500000);
  CAN_Module_EFIS.setMaxMB(16);
  CAN_Module_EFIS.enableFIFO();
  CAN_Module_EFIS.enableFIFOInterrupt();
  CAN_Module_EFIS.onReceive(FIFO, canSniff);
  CAN_Module_EFIS.mailboxStatus();
  delay(199);

  // ********************************************************************************* Initialisation des deux capteurs de pression  *******************************************************************************************
  if (AMS5915_050D.begin() < 0) tft.println("Erreur de communication avec le capteur AMS5915-50D");
  else tft.println("Capteur AMS5915-50D   : OK");
  delay(199);
  if (AMS5915_1500A.begin() < 0) tft.println("Erreur de communication avec le capteur AMS5915-1500A");
  else tft.println("Capteur AMS5915-1500A : OK");
  delay(199);
  
  // ****************************************************************************** Initialisation des filtres des capteurs de pression *************************************************************************************
  AMS5915_1500A.readSensor('A');
  valPressionFiltreePrecedente = AMS5915_1500A.getPressure_Pa(); 
  altitudeVarioOld = int((1 - pow((((valPressionFiltreePrecedente + correctionPression * 10) / 100) / 1013.25), 0.190284)) * 145366.45);
  AMS5915_050D.readSensor('D');
  valPressionDiffFiltreePrecedente = AMS5915_050D.getPressure_Pa();
  tft.println("Filtres               : OK");
  delay(199);
  
  // ********************************************************************************* Initialisation des voies série  *******************************************************************************************
  Serial.begin(115200);
  Serial5.begin(115200);
  tft.println("Serial                : OK");
  delay(199);
  
  // ********************************************************************************* Initialisations des timers *******************************************************************************************
  TimerSendF.begin(SendF, 50000); // Compatibilité avec l'ancien AHRS Naveol : il faut envoyer un 'F' à l'AHRS pour qu'il envoie une trame
  TimerStopReading.begin(StopReading);
  TimerHorloge.begin (setflagAfficheHeure, 500ms);
  Timer200ms.begin(SendData, 200ms);
  tft.println("Timers                : OK");
  delay(199);
  
  // *************************************************************************** Initialisation de l'encodeur rotatif ********************************************************************************************
  pinMode(Pin_Bouton, INPUT_PULLUP);
  attachInterrupt(Pin_Bouton, Bouton_ISR, RISING);
  encodeur.setInitConfig();  //
  encodeur.EncConfig.IndexTrigger = ENABLE;
  encodeur.EncConfig.INDEXTriggerMode = RISING_EDGE;
  encodeur.init();
  tft.println("Encodeur              : OK");
  delay(199);
  Bouton_Clic = false;
  
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
  if ((QNH < 940) || (QNH > 1050)) // Si cette condition est vraie (le QNH n'est pas dans l'intervalle 940-1050), c'est a priori que l'EEPROM n'a pas été utilisée à cette adresse.
                                   // Il faut donc initialiser toutes les variables, puis les inscrire ensuite dans l'EEPROM
  {
    QNH = 1013; 
    magDev = 0;  
    correctionHeureLocale = 1;
    // place pour un int16_t stockée à l'offset adresse 06
    YY = 2022; 
    correctionPression = -20; 
    luminosite = 8;  
    accGmax = 1.0;
    accGmin = 1.0;

    EEPROM.put(EFISeepromAdresse + 0, QNH);
    EEPROM.put(EFISeepromAdresse + 2, magDev);
    EEPROM.put(EFISeepromAdresse + 4, correctionHeureLocale);
    EEPROM.put(EFISeepromAdresse + 8, YY);
    EEPROM.put(EFISeepromAdresse + 10, correctionPression);
    EEPROM.put(EFISeepromAdresse + 12, luminosite);
    EEPROM.put(EFISeepromAdresse + 14, accGmax);
    EEPROM.put(EFISeepromAdresse + 18, accGmin);
  }
  else 
  {
    EEPROM.get(EFISeepromAdresse + 0, QNH);
    EEPROM.get(EFISeepromAdresse + 2, magDev);
    EEPROM.get(EFISeepromAdresse + 4, correctionHeureLocale);
    EEPROM.get(EFISeepromAdresse + 8, YY);
    EEPROM.get(EFISeepromAdresse + 10, correctionPression);
    EEPROM.get(EFISeepromAdresse + 12, luminosite);
    EEPROM.get(EFISeepromAdresse + 14, accGmax);
    EEPROM.get(EFISeepromAdresse + 18, accGmin);
  }
  tft.println("EEPROM                : OK");
  delay(199);
  
  // ************************************************************************************************ Démarrage de l'horloge **************************************************************************************************************************************
  setSyncProvider(getTeensy3Time);
  setSyncInterval(900);
  tft.println("SRTC                  : OK");
  delay(199);
  
  // **************************************************************************************** Initialisation et tri des options des menus  **********************************************************************************************************************
  // Pour faciliter la gestion des options de menu (ajouts, suppressions, modifications), ces options sont affichées pendant le setup sur le terminal série, avant et après tri.
  uint8_t nb = NbOptionsMenu();
  Serial.print ("Nombre de lignes utiles du tableau : "); 
  Serial.println(nb); Serial.println();
  Serial.println("**********Tableau avant tri et initialisation des index***********");
  Serial.println("Index------Label--------------Numero--Action---NbOptS--IndexOfOm--");
  for (byte n = 1; n <= nb + 1; n++)
  {
    sprintf(buf, "%2d", (n - 1));
    Serial.print(buf); 
    Serial.print(char(9));
    sprintf(buf, "%-20s", (menu[n - 1].label));
    Serial.print (buf); 
    Serial.print(char(9));
    sprintf (buf, "%4d", (menu[n - 1].numero));
    Serial.print (buf); 
    Serial.print (char(9));
    Serial.print(menu[n - 1].action); 
    Serial.print (char(9));
    sprintf(buf, "%2d", (menu[n - 1].nbOptionsSoeurs));
    Serial.print(buf); 
    Serial.print(char(9));
    sprintf(buf, "%2d", (menu[n - 1].indexOfOm));
    Serial.print(buf); 
    Serial.println();
  }
  triBulleTableau();
  initTable();       // Initialisation des champs "nbOptionsSoeurs" et indexOfOm"

  Serial.println();
  Serial.println("**********Tableau après tri et initialisation des index***********");
  Serial.println("Index------Label--------------Numero--Action---NbOptS--IndexOfOm--");
  for (byte n = 1; n <= nb + 1; n++)
  {
    indexOptionMenuEnCours = n - 1;
    sprintf(buf, "%2d", (n - 1));
    Serial.print(buf); 
    Serial.print(char(9));
    sprintf(buf, "%-20s", (menu[n - 1].label));
    Serial.print (buf); 
    Serial.print(char(9));
    sprintf (buf, "%4d", (menu[n - 1].numero));
    Serial.print (buf); 
    Serial.print (char(9));
    Serial.print(menu[n - 1].action); 
    Serial.print (char(9));
    sprintf(buf, "%2d", (menu[n - 1].nbOptionsSoeurs));
    Serial.print(buf); 
    Serial.print(char(9));
    sprintf(buf, "%2d", (menu[n - 1].indexOfOm));
    Serial.println(buf);
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
  LaigH = Rh * 2 / 3;
  LaigM = Rh * 8 / 10;
  LaigS = Rh * 9 / 10;

  //********************************************************************************* On arrive presqu'à la fin de SETUP ***************************************************************************************************
  topHoraire = millis();
  topchronoVario = millis();

}// ******************************************************************************************************************FIN DU SETUP***************************************************************************************************

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                             LOOP
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{
  tft.brightness(luminosite * 16 - 1);
  tft.writeTo(L1);
  
  //  ***************************************************************************** Traitement de l'éventuelle disponibilité d'une trame AHRS **************************************************************************************

  if (dataReady)
  {
    dataReady = false;
    if (tabTrame[0] == 70 && tabTrame[1] == 18 && (*(float*)(tabTrame + 58)) == -1 && ndex == 74)
    {
      roll = *(float*)(tabTrame + 2);
      if (roll < 2.8 && roll > -2.8)
      {
        roll = filtrageRII (valRollFiltreePrecedente, roll, 0.1);
      }
      valRollFiltreePrecedente = roll;

      pitch = (*(float*)(tabTrame + 6)) * (180 / PI);
      pitch = filtrageRII (valPitchFiltreePrecedente, pitch, 0.1);
      valPitchFiltreePrecedente = pitch;

      psi = (*(float*)(tabTrame + 10));
      if (psi > PI) // Compatibilité avec l'ancien AHRS Naveol
      {
        flagPsiFiable = false;
        psi = psi - (4 * PI);
      }
      else flagPsiFiable = true;
      psi = psi * (180 / PI);
      if (psi < 0)psi = 360 + psi;

      AccY = *(float*)(tabTrame + 46);
      AccY = filtrageRII (valAccYFiltreePrecedente, AccY , 0.01);
      valAccYFiltreePrecedente = AccY;

      accG = *(float*)(tabTrame + 50);
      accG = -1 * accG / 9.80665;
      if ((accG > accGmax) && (pressionDiff >= 380))
      {
        accGmax = accG;
        EEPROM.put(EFISeepromAdresse + 14, accGmax);
      }
      if ((accG < accGmin) && (pressionDiff >= 380))
      {
        accGmin = accG;
        EEPROM.put(EFISeepromAdresse + 18, accGmin);
      }
      accG = filtrageRII (valAccGFiltreePrecedente, accG, 0.1);
      valAccGFiltreePrecedente = accG;

      offsetBille = (AccY * -1 * Rbille) / 0.27;
      if (offsetBille > Rbille * 8) offsetBille = Rbille * 8;
      if (offsetBille < -Rbille * 8) offsetBille = -Rbille * 8;

      Vz = *(float*)(tabTrame + 54);
      Vz = filtrageRII (valVzFiltreePrecedente, Vz, 0.01);
      valVzFiltreePrecedente = Vz;
      Vz = Vz * 196.8504;
      Vzint = (int(Vz + 5) / 10) * 10;

      trk =      (*(float*)(tabTrame + 62)) * (180 / PI);
      if ((trk < 0) && (valTrkFiltreePrecedente < 90))
      {
        valTrkFiltreePrecedente = 360;
      }
      if ((trk > 0) && (valTrkFiltreePrecedente > 270))
      {
        valTrkFiltreePrecedente = 0;
      }
      if (trk < 0)trk = 360 + trk;
      trk = filtrageRII (valTrkFiltreePrecedente, trk, 0.1);
      valTrkFiltreePrecedente = trk;
      trkEntierArrondi = int(trk + 0.5);
      nbSat =     *(float*)(tabTrame + 70);

      altitudeGNSS = (*(float*)(tabTrame + 14)); // altitude MSL passée en mètre par l'AHRS
      altitudeGNSS = altitudeGNSS * 3.28084;    // conversion en pieds
      
      groundSpeedGNSS = (*(float*)(tabTrame + 18)); // Vitesse sol GPS passée en m/s
      groundSpeedGNSS = filtrageRII (groundSpeedGNSSOld, groundSpeedGNSS, 0.1);
      groundSpeedGNSSOld = groundSpeedGNSS;
      groundSpeedGNSS = groundSpeedGNSS * 3.6; // Vitesse sol convertie en km/h
  
      anneeGNSS   = (*(uint16_t*)(tabTrame + 22));
      moisGNSS    = tabTrame[24];
      jourGNSS    = tabTrame[25];
      heureGNSS   = tabTrame[26];
      minuteGNSS  = tabTrame[27];
      secondeGNSS = tabTrame[28];     
    }
    
    // Envoi des nouvelles données de l'AHRS à l'unité distante (magnétomètre - thermomètre - hygromètre) via le CAN bus
      msgRoll.id =  52;
      msgPitch.id = 53;
      msgRoll.len =  4;
      msgPitch.len = 4;
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgRoll.buf[i] = ((byte*) &roll)[i];
      msgPitch.buf[i] = ((byte*) &pitch)[i];
    }
    CAN_Module_EFIS.write(msgRoll);
    CAN_Module_EFIS.write(msgPitch);
  }
  
  //********************************************** Le cas échéant, après la mise sous tension, réglage unique de l'heure sur celle du GNSS de l'AHRS, une fois le fix obtenu **********************************************************
  if (!fixOK)
  { 
    if (nbSat > 8)
    {
      fixOK = true;
    }
  }
  if (fixOK && !heureReglee)
  { 
    setTime(heureGNSS, minuteGNSS, secondeGNSS, jourGNSS, moisGNSS, anneeGNSS);
    Teensy3Clock.set(now());
    heureReglee = true;
  }
  
  // ****************************************************************************************** Lecture des capteurs de pression ****************************************************************************************************
  AMS5915_050D.readSensor('D');
  AMS5915_1500A.readSensor('A');
  pression = AMS5915_1500A.getPressure_Pa();
  pressionNonF = pression;
  pression = filtrageRII (valPressionFiltreePrecedente, pression , 0.005);
  valPressionFiltreePrecedente = pression;
  pressionDiff = AMS5915_050D.getPressure_Pa();
  pressionDiffNonF = pressionDiff;
  pressionDiff = filtrageRII (valPressionDiffFiltreePrecedente, pressionDiff, 0.005);
  valPressionDiffFiltreePrecedente = pressionDiff;

  // ****************************************************************************************** Lecture du capteur de température intérieure *********************************************************************************************
  if (flagDelaiAcquisitionDS18B20Termine == true)
  {
    ds.reset();
    ds.select(addrDS18B20);
    ds.write(0x44);
    chronoDS18B20 = millis();
    flagDelaiAcquisitionDS18B20Termine = false;
  }
  if ((millis() - chronoDS18B20) > 1000)
  {
    ds.reset();
    ds.select(addrDS18B20);
    ds.write(0xBE);
    for (byte i = 0; i < 9; i++) dataDS18B20[i] = ds.read();
    temperatureDS18B20 = (float)((dataDS18B20[1] << 8) | dataDS18B20[0]) / 16.0;
    flagDelaiAcquisitionDS18B20Termine = true;
  }

  // ************************************************************************************ Calculs des affichages textuels *********************************************************************************************************

  // --------- calcul des altitudes --------- (Les pressiosn sont exprimées en Pascals, et filtrées dès l'étape de lecture des capteurs)
  altitudeQNH = (1 - pow((((pression + correctionPression * 10) / 100) / QNH), 0.190284)) * 145366.45 * correctionAltitude;
  altitudeQNH = int((altitudeQNH + 5) / 10.0) * 10;
  altitudePression = (1 - pow((((pression + correctionPression * 10) / 100) / 1013.25), 0.190284)) * 145366.45 * correctionAltitude;
  altitudeVario = altitudePression;
  altitudePression = int((altitudePression + 5) / 10.0) * 10;
  iat = temperatureDS18B20;
  float pressionSatH20 = 6.1078 * pow(10, ((7.5 * oat) / (237.3 + oat))) * 100;
  float pressionPartielleH2O = pressionSatH20 * humiditeRelative / 100;
  float densiteAir = ((pression + correctionPression * 10) / (287.05 * (oat + 273.15)) * (1 - (0.378 * pressionPartielleH2O / (pression + correctionPression * 10))));
  altitudeDensite = (44.3308 - (42.2665 * (pow(densiteAir, 0.234969)))) * 1000 * 3.28084;
  altitudeDensite = int((altitudeDensite + 5) / 10.0) * 10;
  dewPoint = 243.12 * (log(humiditeRelative / 100) + 17.62 * oat / (243.12 + oat)) / (17.62 - (log(humiditeRelative / 100) + 17.62 * oat / (243.12 + oat)));

  // ----------- Calcul du taux de montée/descente à partir du capteur de pression statique -------------------
  if (millis() >= topchronoVario + tempsIntegrationVario)
  {
    vario =  (altitudeVario - altitudeVarioOld) * 60 * (1000 / tempsIntegrationVario) ;
    topchronoVario = millis();
    altitudeVarioOld = altitudeVario;
    vario = filtrageRII (valVarioFiltreePrecedente, vario , 0.05);
    valVarioFiltreePrecedente = vario; 
    if (vario >= 0) vario = (int(vario + 5) / 10) * 10;
    else vario = (int(vario - 5) / 10) * 10;
  }

  // ----------------------- calcul des vitesses de l'avion --------------------------------------------
  if (pressionDiff > 4.5)
  {
    vitesseIndiquee = int((sqrtf(21.159144 * pressionDiff) / facteurConversionVitesse) + 0.5);
    vitessePropre = int((84.9528 * sqrtf(pressionDiff * (273.15 + oat) / pression) / facteurConversionVitesse) + 0.5);
  }
  else 
  {
    vitesseIndiquee = 0;
    vitessePropre = 0;
  }

  // ----------------------- calcul direction et vitesse du vent (nécessite un magnétomètre PARFAITEMENT calibré !!!) --------------------------------------------
  float GSE = sin(PI / 180 * trk) * groundSpeedGNSS;
  float GSN = cos(PI / 180 * trk) * groundSpeedGNSS;
  float TASE = sin(PI / 180 * (capMagnetique + magDev)) * vitessePropre;
  float TASN = cos(PI / 180 * (capMagnetique + magDev)) * vitessePropre;
  float deltaE = TASE - GSE;
  float deltaN = TASN - GSN;
  windSpeed = sqrt(pow(deltaE, 2) + pow(deltaN, 2));
  windDirection = atan2(deltaE, deltaN) * 180 / PI;
  if (windDirection < 0)windDirection = 360 + windDirection;

  //**************************************************************************************** Mise à l'heure de la pendule et des chronos *********************************************************************************************
  if (flagAfficheHeure)
  {
    flagAfficheHeure = false;
    afficheHeure();
    time_t i;
    if (flag_chrono1)
    {
      i = now() - t_chrono1;
      tft.setCursor (80, 222); 
      printDigits(hour(i)); 
      tft.print(":"); 
      printDigits(minute(i)); 
      tft.print(":"); 
      printDigits(second(i));
    }
    if (flag_chrono2)
    {
      i = now() - t_chrono2;
      tft.setCursor (180, 222); 
      tft.setTextColor (RA8875_GREEN, RA8875_BLACK); 
      printDigits(minute(i)); 
      tft.print(":"); 
      printDigits(second(i)); 
      tft.setTextColor (RA8875_WHITE, RA8875_PINK);
    }
    if (flag_minuteur)
    {
      i = t_minuteur - now();
      if (i >= 0) 
      {
        tft.setCursor (260, 222);
        tft.setTextColor (RA8875_WHITE, RA8875_RED);
        printDigits(minute(i));
        tft.print(":");
        printDigits(second(i));
        tft.setTextColor (RA8875_WHITE, RA8875_PINK);
      }
      else 
      {
        flag_minuteur = false;
        tft.setCursor (260, 222);
        tft.print ("     ");
      }
    }
  }

  // ************************************************************************************ Mise à jour des affichages textuels et du vario ****************************************************************************************
  afficheTextes();
  afficheVario();

  // *********************************************************** Envoi périodique sur le CAN bus de différentes informations à destination de l'EMS *************************************************************
  if (okSendCAMmsg)
  {
    okSendCAMmsg = false;
    msgAltiDAltiP.id =  62;
    msgAltiDAltiP.len =  8;
    msgDateHeureTAS.id = 63;
    msgDateHeureTAS.len = 6;
    zt  = now();
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgDateHeureTAS.buf[i] = ((byte*) &zt)[i];
    }
    for (uint8_t i = 0; i < 2; i++ )
    {
      msgDateHeureTAS.buf[i + 4] = ((byte*) &vitessePropre)[i];
    }
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgAltiDAltiP.buf[i] = ((byte*) &altitudeDensite)[i];
    }
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgAltiDAltiP.buf[i + 4] = ((byte*) &altitudePression)[i];
    }
    CAN_Module_EFIS.write(msgAltiDAltiP);
    CAN_Module_EFIS.write(msgDateHeureTAS);
  }

// **************************************** Envoi éventuel périodique des paramètres EFIS sur la voie série USB et sur la cartte micro SD pour enregistrement *************************************************************
if (okSendDataToRecord && recordStarted) // okSendDataToRecord mis sur true par interruption toutes les 200 ms, recordStarted mis sur true ou false par menu
{
  okSendDataToRecord=false;
  if(cptBlink == 0) tft.fillCircle(350,220,10,RA8875_RED);
  if(cptBlink == 2) tft.fillCircle(350,220,10,RA8875_PINK);

  // TimeLib et le RTC sont réglé sur l'heure UTC. Pour l'horodatage, on souhaite une heure locale
  zt = now() + (3600 * correctionHeureLocale); // zt contient maintenant l'heure locale
  Serial.print(day(zt)); Serial.print('/'); Serial.print(month(zt)); Serial.print('/'); Serial.print (year(zt));  Serial.print (';');
  Serial.print(hour(zt));Serial.print(':'); Serial.print(minute(zt));Serial.print(':'); Serial.print(second(zt)); Serial.print (';');
  Serial.print((*(float*)(tabTrame +  2)), 4); Serial.print (";");  //Angle de roulis "brut" en radians
  Serial.print((*(float*)(tabTrame +  6)), 4); Serial.print (";");  //Angle de tangage "brut" en radians
  Serial.print((*(float*)(tabTrame + 10)), 4); Serial.print (";");  //Angle de lacet "brut" en radians (+ 4 * Pi lorsque invalide, + 0 lorsque valide)
  Serial.print((*(float*)(tabTrame + 46)), 4); Serial.print (";");  //Accélération "brute" Y en m/s²
  Serial.print((*(float*)(tabTrame + 50)), 4); Serial.print (";");  //Accélération "brute" Z en m/s²
  Serial.print((*(float*)(tabTrame + 54)), 3); Serial.print (";");  //Vitesse verticale "brute" en m/s (valeur transmise positive vers le haut)
  Serial.print((*(float*)(tabTrame + 58)), 0); Serial.print (";");  //Constante -1
  Serial.print((*(float*)(tabTrame + 62)), 4); Serial.print (";");  //Cap GPS "brut" (TK, ou Track) en radians (c’est-à-dire la route)
  Serial.print((*(float*)(tabTrame + 70)), 0); Serial.print (";");  //Nombre de satellites GPS utilisés
  Serial.print(roll, 4);                       Serial.print (";");  // roll filtré en radians
  Serial.print(pitch, 3);                      Serial.print (";");  // pitch filtré en degrés
  Serial.print(psi, 1);                        Serial.print (";");  // psi filtré en degrés
  Serial.print(AccY, 3);                       Serial.print (";");  // AccY filtrée en m/s²
  Serial.print(accG, 3);                       Serial.print (";");  // accG filtrée en G
  Serial.print(Vz, 3);                         Serial.print (";");  // Vz filtrée en ft/minute
  Serial.print(trk, 3);                        Serial.print (";");  // trk filtrée en degrés
  Serial.print(pression, 5);                   Serial.print (";");  // pression absolue filtrée en Pa
  Serial.print(pressionNonF, 5);               Serial.print (";");  // pression absolue non filtrée en Pa
  Serial.print(pressionDiff, 5);               Serial.print (";");  // pression différentielle filtrée en Pa
  Serial.print(pressionDiffNonF, 5);           Serial.print (";");  // pression différentielle non filtrée en Pa
  Serial.print(oat, 1);                        Serial.print (";");  // Température intérieure
  Serial.print(altitudeQNH);                   Serial.print (";");  // Altitude QNH
  Serial.print(vitesseIndiquee);               Serial.print (";");  // Vitesse indiquée
  Serial.print(vitessePropre);                 Serial.print (";");  // Vitesse propre
  Serial.print(Vzint);                         Serial.print (";");  // Vzint (vario AHRS) en ft/min
  Serial.print(vario, 0);                      Serial.println();    // Vario barométrique en ft/min

  /*  
  // L'enregistrement de données en vol est beaucoup plus performant avec un enregistreur externe connecté sur la voie série USB qu'avec une carte micro SD
  // Pour mémoire uniquement, on laisse ici ces lignes dédiées micro SD.
  fichier.print(day(now()));        fichier.print(";");
  fichier.print(month(now()));      fichier.print(";");
  fichier.print(hour(now()));       fichier.print(";");
  fichier.print(minute(now()));     fichier.print(";");
  fichier.print(second(now()));     fichier.print(";");
  fichier.print(roll,0);            fichier.print(";");
  fichier.print(pitch,0);           fichier.print(";");
  fichier.print(trkEntierArrondi);  fichier.print(";");
  fichier.print(oat,1);             fichier.print(";");
  fichier.print(altitudeQNH);       fichier.print(";");
  fichier.print(pressionDiff,4);    fichier.print(";");
  fichier.print(pression,4);        fichier.print(";");
  fichier.print(vitesseIndiquee);   fichier.print(";");
  fichier.print(vitessePropre);     fichier.print(";");
  fichier.print(Vzint);             fichier.print(";");
  fichier.print(int(vario));        fichier.println();
  */
  cptBlink ++; if (cptBlink==4) cptBlink = 0;
}  

  //  ***************************************************************************** Traitement d'un clic sur le bouton de l'encodeur *******************************************************************************************

  if (Bouton_Clic == true)
  {
    if (Menu_Ouvert == false)
    {
      Menu_Ouvert = true;
      OuvreMenu(0);
    }
    else
    {
      if (SetVal == false)
      {
        switch (menu[indexOptionMenuEnCours].action)
        {
          case 'Q':
            Menu_Ouvert = false;
            FermeMenu ();
            break;
          case 'S':
            OuvreMenu(menu[indexOptionMenuEnCours].indexOfOm);
            break;
          case 'B':
            OuvreMenu(menu[indexOptionMenuEnCours].indexOfOm);;
            break;
          case 'P':
            SetVal = true;
            encodeur.write(0);
            ValeurPositionEncodeurOld = ValeurPositionEncodeur;
            ValeurPositionEncodeurOldSetVal = 0;
            Bouton_Clic = false;
            switch (menu[indexOptionMenuEnCours].numero)
            { 
              case 101  : ptrGen = &QNH;
                          OffsetEFISeepromAdresse =  0;   
                          break;
              case 112  : ptrGen = &magDev;                    
                          OffsetEFISeepromAdresse =  2;   
                          break;
              case 114  : ptrGen = &correctionPression;        
                          OffsetEFISeepromAdresse = 10;   
                          break;
              case 115  : ptrGen = &luminosite;                
                          OffsetEFISeepromAdresse = 12;   
                          break;
              case 117  : ptrGen = &correctionHeureLocale;     
                          OffsetEFISeepromAdresse =  4;   
                          break;  
              case 123  : ptrGen = &dureeMinuteur;             
                          OffsetEFISeepromAdresse = 1000; 
                          flag_minuteur = false; 
                          tft.setCursor (320, 222); 
                          tft.print ("     "); 
                          break;
              case 1131 : ptrGen = &YY;
                          OffsetEFISeepromAdresse =  8;   
                          break;
              case 1132 : ptrGen = &MM;                        
                          OffsetEFISeepromAdresse = 1000; 
                          break;
              case 1133 : ptrGen = &DD;                        
                          OffsetEFISeepromAdresse = 1000; 
                          break;
              case 1134 : ptrGen = &HH;                        
                          OffsetEFISeepromAdresse = 1000; 
                          break;
              case 1135 : ptrGen = &Mn;                        
                          OffsetEFISeepromAdresse = 1000; 
                          break;
              case 1136 : ptrGen = &Sec;                       
                          OffsetEFISeepromAdresse = 1000; 
                          break;
              default   : break;
            }
            tft.fillRect((((menu[indexOptionMenuEnCours].numero) % 10)*largeurCase), (ordonneeMenu - hauteurCase + 3), largeurCase, hauteurCase - 3, RA8875_RED);
            tft.setCursor(((menu[indexOptionMenuEnCours].numero) % 10)*largeurCase + (largeurCase / 4), ordonneeMenu + 4 - hauteurCase);
            tft.setTextColor(RA8875_WHITE, RA8875_RED);
            tft.setFont(Arial_10);
            tft.print(*ptrGen);
            tft.setFont(Arial_11);
            break; // break du case 'P'

          case 'A' :
            switch (menu[indexOptionMenuEnCours].numero)
            {
              case 102  : OuvreMenu(indexRetour(indexOptionMenuEnCours)); 
                          break;
              case 103  : accGmax = 1.0; accGmin = 1.0; 
                          EEPROM.put(EFISeepromAdresse + 14, accGmax); 
                          EEPROM.put(EFISeepromAdresse + 18, accGmin); 
                          OuvreMenu(indexRetour(indexOptionMenuEnCours)); 
                          break;
              case 112  : OuvreMenu(indexRetour(indexOptionMenuEnCours)); 
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
              case 1111 : facteurConversionVitesse = 1.0; 
                          OuvreMenu(indexRetour(indexOptionMenuEnCours));   
                          break;
              case 1112 : facteurConversionVitesse = 1.852; 
                          OuvreMenu(indexRetour(indexOptionMenuEnCours)); 
                          break;
              case 1137 : setTime(HH, Mn, Sec, DD, MM, YY); 
                          FermeMenu(); 
                          break;
              case 1161 : if (SD.exists("log.txt"))
                          {
                            SD.remove("log.txt");
                            fichier = SD.open("log.txt");
                            fichier.close();
                          } 
                          recordStarted = false; 
                          break; 
              case 1162 : fichier = SD.open("log.txt", FILE_WRITE);
                          serialPrintNomsChamps(); 
                          recordStarted = true; 
                          break; 
              case 1163 : fichier.close();
                          recordStarted = false;
                          tft.fillCircle(350,220,10,RA8875_PINK); 
                          break; 
              case 1164 : fichier.close();
                          recordStarted = false; 
                          fichier = SD.open("log.txt"); 
                          if (fichier) 
                          {
                            while (fichier.available()) 
                            {
                              Serial.write(fichier.read());
                            } 
                            fichier.close();
                          } 
                          break; 
            }
            break;
        }
      }

      else
      {
        if (OffsetEFISeepromAdresse != 1000) EEPROM.put(EFISeepromAdresse + OffsetEFISeepromAdresse, *ptrGen);
        if (dureeMinuteur <= 0) 
        {
          flag_minuteur = false;
          tft.setCursor (320, 222);
          tft.print ("     ");
        }
        else if (flag_minuteur == false) 
             {
              t_minuteur = now() + dureeMinuteur * 60;
              flag_minuteur = true;
             }
        SetVal = false;
        ptrGen = NULL; 
        //OuvreMenu(indexRetour(indexOptionMenuEnCours)); // et on revient à la première option du menu en cours
        FermeMenu(); // Ou, plus ergonomique, on ferme le menu
      }
    }
    Bouton_Clic = false;
  }

  // ************************************************************************* Polling de la position de l'encodeur à chaque passage dans Loop ************************************************************************************
  if (Menu_Ouvert == true)
  {
    if (SetVal == false)
    {
      ValeurPositionEncodeur = encodeur.read();
      tft.setTextColor (RA8875_WHITE, RA8875_PINK);
      if (ValeurPositionEncodeur != ValeurPositionEncodeurOld)
      {
        uint16_t mini = menu[indexOptionMenuEnCours].numero;
        mini = mini % 10;
        mini = indexOptionMenuEnCours - mini;
        uint8_t maxi = mini + menu[mini].nbOptionsSoeurs - 1;
        if ((ValeurPositionEncodeur > maxi - mini) || (ValeurPositionEncodeur < 0))
        {
          encodeur.write(0);
          ValeurPositionEncodeur = encodeur.read();
        }
        indexOptionMenuEnCours = ValeurPositionEncodeur + mini;
        tft.drawRect ((ValeurPositionEncodeurOld * largeurCase), ordonneeMenu, largeurCase, hauteurCase, BckgrndMenus);
        tft.drawRect ((ValeurPositionEncodeur * largeurCase), ordonneeMenu, largeurCase, hauteurCase, RA8875_WHITE);
      }
      ValeurPositionEncodeurOld = ValeurPositionEncodeur;
    }
    else
    {
      ValeurPositionEncodeur = encodeur.read();
      if (ValeurPositionEncodeur != ValeurPositionEncodeurOldSetVal)
      {
        int8_t sens = encodeur.getHoldDifference();
        *ptrGen = *ptrGen + sens;
        verificationValeurs();
        tft.fillRect((((menu[indexOptionMenuEnCours].numero) % 10)*largeurCase), (ordonneeMenu - hauteurCase + 3), largeurCase, hauteurCase - 3, RA8875_RED);
        tft.setCursor(((menu[indexOptionMenuEnCours].numero) % 10)*largeurCase + (largeurCase / 4), ordonneeMenu + 4 - hauteurCase);
        tft.setTextColor(RA8875_WHITE, RA8875_RED);
        tft.setFont(Arial_10);
        tft.print(*ptrGen);
        tft.setFont(Arial_11);
      }
      ValeurPositionEncodeurOldSetVal = ValeurPositionEncodeur;
    }
  }

  // ************************************************************************************ Mise à jour de l'horizon artificiel****************************************************************************************
  Horizon.dessine(roll, pitch, psi, offsetBille, Menu_Ouvert);

  // ************************************************************************* Mesure de la durée de la boucle principale Loop ************************************************************************************
  if (nombreBoucleLoop >= Ndb)
  {
    dureeNdbBoucles = millis() - topHoraire;
    dureeLoop = float(dureeNdbBoucles / Ndb);
    topHoraire = millis();
    nombreBoucleLoop = 0;
  }
  else
  {
    nombreBoucleLoop++;
  }
}

//***************************************************************************************************** Fin de la boucle infinie Loop ***********************************************************************************************

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//           ISR déclenchant l'envoi de données sur la carte micro SD et sur la voie série USB pour le flight recorder (si un enregistrement est en cours) et l'envoi de données sur le CAN Bus
////-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void SendData() // ISR du PeriodicTimer Timer200ms, ce timer déclenche son interruption 5 fois par seconde
{ 
  okSendDataToRecord = true;
  okSendCAMmsg = true;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                               Cette fonction envoie sur la voie série USB à chaque début d'enregistrement les noms des champs à enregistrer par le flight data recorder 
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void serialPrintNomsChamps()
{
  Serial.print ("Date");                Serial.print(';');
  Serial.print ("Heure");               Serial.print(';');
  Serial.print ("RollBrutRad");         Serial.print(';');
  Serial.print ("PitchBrutRad");        Serial.print(';');
  Serial.print ("LacetBrutRad");        Serial.print(';');
  Serial.print ("AccYBrutm/s²");        Serial.print(';');
  Serial.print ("AccZBrutm/s²");        Serial.print(';');
  Serial.print ("VzBrutm/s");           Serial.print(';');
  Serial.print ("Const-1");             Serial.print(';');
  Serial.print ("TrkBrutRad");          Serial.print(';');
  Serial.print ("NbSatAHRS");           Serial.print(';');
  Serial.print ("RollFiltrRad");        Serial.print(';');
  Serial.print ("PitchFiltrDeg");       Serial.print(';');
  Serial.print ("PsiFiltrDeg");         Serial.print(';');
  Serial.print ("AccYFiltrm/s²");       Serial.print(';');
  Serial.print ("AccZFiltrG");          Serial.print(';');
  Serial.print ("VzFiltrFt/min");       Serial.print(';');
  Serial.print ("TrkFiltrDeg");         Serial.print(';');
  Serial.print ("PressAbsFiltrPa");     Serial.print(';');
  Serial.print ("PressAbsNonFiltrPa");  Serial.print(';');
  Serial.print ("PressDifFiltrPa");     Serial.print(';');
  Serial.print ("PressDifNonFiltrPa");  Serial.print(';');
  Serial.print ("OAT");                 Serial.print(';');
  Serial.print ("AltiQNH");             Serial.print(';');
  Serial.print ("VitIndiq");            Serial.print(';');
  Serial.print ("VitPropre");           Serial.print(';');
  Serial.print ("VarioAHRSFt/min");     Serial.print(';');
  Serial.print ("VarioBaroFt/min");     Serial.println();
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   ISR de récéption du CAN Bus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void canSniff(const CAN_message_t &msg)
{
  switch (msg.id)
  {
    case 42: oat = *(float*)(msg.buf);
             break;
    case 43: humiditeRelative = *(float*)(msg.buf);
             break;
    case 44: capMagnetique = *(uint32_t*)(msg.buf);
             break;
    default: break;
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                          Fonctions de mise à jour des affichages textuels et du vario
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void afficheTextes()
{
  // ------------- Affichage direction et vitesse du vent -------------------
  tft.setCursor (50, 30); 
  sprintf (buf, "%3d", int(windDirection + 0.5)); 
  tft.print (buf); tft.print("");
  tft.setCursor (50, 45); 
  sprintf (buf, "%3d", int(windSpeed + 0.5)); 
  tft.print (buf); 
  if (facteurConversionVitesse == 1.0)tft.print (" Km/h"); 
  else tft.print (" Kts ");

  // ------------- Affichage IAT ---------------------------
  tft.setCursor (105, 156); 
  printDecimalInfCent(iat, 1);

  // ------------- Affichage OAT ---------------------------
  tft.setCursor (105, 171); 
  printDecimalInfCent(oat, 1);

  // --------------Affichage Humidité ---------------------
  tft.setCursor (105, 186); 
  printDecimalInfCent(humiditeRelative, 1);

  // --------------Affichage DewPoint ---------------------
  tft.setCursor (105, 201); 
  printDecimalInfCent(dewPoint, 1);

  // ------------- Affichage Vitesse indiquée ---------------------------
  tft.setFontScale (1); 
  tft.setTextColor (RA8875_WHITE, RA8875_BLACK);
  sprintf (buf, "%3d", (vitesseIndiquee)); 
  tft.setCursor (7, 118);  
  tft.print (buf);
  tft.setCursor (65, 127); 
  tft.setFontScale (0); 
  tft.setTextColor (RA8875_WHITE, RA8875_PINK); 
  if (facteurConversionVitesse == 1.0)tft.print ("Km/h"); 
  else tft.print ("Kts ");

  // ------------- Affichage Route GPS ---------------------------
  if (flagPsiFiable == false) tft.setTextColor (RA8875_WHITE, RA8875_RED);
  else tft.setTextColor (RA8875_WHITE, RA8875_BLACK);     // Si la vitesse GPS est trop faible (moins de 6 ou 7 km/h), psi n'est pas fiable, les paramètre trk et psi vont s'afficher sur fond rouge au lieu de noir.
  tft.setFontScale (1, 0); 
  sprintf (buf, "%3d", trkEntierArrondi); 
  tft.setCursor (276, 3); 
  tft.print (buf);

  // --------------Affichage cap magnétique ---------------------
  tft.setTextColor (RA8875_WHITE, RA8875_BLACK);
  tft.setCursor (156, 3);  
  sprintf (buf, "%3d", capMagnetique); 
  tft.print(buf);

  // ------------- Affichage Vitesse propre (TAS) ---------------------------
  tft.setFontScale (0);
  tft.setTextColor (RA8875_WHITE, RA8875_PINK);
  sprintf (buf, "%3d", vitessePropre); 
  tft.setCursor (30, 97); 
  tft.print (buf); 
  tft.setCursor (65, 97); 
  if (facteurConversionVitesse == 1.0)tft.print ("Km/h"); 
  else tft.print ("Kts ");

  // --------------- Affichage vitesse UBLOX ----------------------------------
  sprintf (buf, "%3d", int(groundSpeedGNSS / facteurConversionVitesse)); 
  tft.setCursor (30, 82); 
  tft.print (buf); 
  tft.setCursor (65, 82); 
  if (facteurConversionVitesse == 1.0)tft.print ("Km/h"); 
  else tft.print ("Kts ");

  // ------------- Affichage Vz AHRS et Vario barométrique ---------------------------
  tft.setCursor(363, 3); 
  tft.print("Vert. speed");
  tft.setCursor (363, 33);   
  tft.print("AHRS :"); 
  sprintf (buf, "%5d", Vzint); 
  tft.print(buf);
  tft.setCursor (363, 48);   
  tft.print("BARO :"); 
  sprintf (buf, "%5d", int(vario)); 
  tft.print(buf);

  // ------------- Affichage nombre de satellites ---------------------------
  tft.setCursor (210, 180);   
  tft.print("nSat : "); 
  sprintf (buf, "%2d", int(nbSat)); 
  tft.print(buf); 


  // ------------- Affichage QNH ---------------------------
  tft.setCursor (420, 206); 
  sprintf (buf, "%4d", QNH); 
  tft.print(buf);

  // ------------- Affichage des 3 altitudes ---------------------------
  tft.setFontScale (0, 1); 
  tft.setTextColor (RA8875_WHITE, RA8875_BLACK);
  sprintf (buf, "%5d", int(altitudeQNH)); 
  tft.setCursor (415, 118);  
  tft.print (buf);
  tft.setFontScale (0); 
  tft.setTextColor (RA8875_WHITE, RA8875_PINK);
  sprintf (buf, "%5d", int(altitudePression)); 
  tft.setCursor (415, 157); 
  tft.print (buf);
  sprintf (buf, "%5d", int(altitudeDensite)); 
  tft.setCursor (415, 172); 
  tft.print (buf);
  sprintf (buf, "%5d", int(altitudeGNSS)); 
  tft.setCursor (415, 187); 
  tft.print (buf);

  // ------------- Affichage G, Gmin et Gmax ---------------------------
  tft.setCursor (363, 82); 
  tft.print ("G    :"); printDecimalInfDix(accG, 1);
  tft.setCursor (363, 67); 
  tft.print ("Gmax :"); printDecimalInfDix(accGmax, 1);
  tft.setCursor (363, 97); 
  tft.print ("Gmin :"); printDecimalInfDix(accGmin, 1);

  // ------------- Affichage durée loop() ---------------------------
  tft.setCursor(210, 195);
  tft.print("Loop : ");
  tft.print (dureeLoop, 2);
  tft.print(" ms ");  

}

void afficheVario()
{
  // affichage des barres en premier 
  int8_t hauteurBarre;
  if (abs(vario) > 2000) hauteurBarre = 116;
  else hauteurBarre = abs(int8_t((vario * 58) / 1000));
  if (vario < 0)
  {
    tft.fillRect(461, 20, 9, 116, RA8875_PINK);
    tft.fillRect(461, 136, 9, hauteurBarre, RA8875_PURPLE);
    tft.fillRect(461, 136 + hauteurBarre, 9, 252 - 136 - hauteurBarre, RA8875_PINK);
    tft.drawRect(460, 136, 10, hauteurBarre + 1, RA8875_WHITE);
  }
  else
  {
    tft.fillRect(461, 20, 9, 116 - hauteurBarre, RA8875_PINK);
    tft.fillRect(461, 136 - hauteurBarre, 9, hauteurBarre, 0b1111111001000000);
    tft.fillRect(461, 136, 9, 116, RA8875_PINK);
    tft.drawRect(460, 136 - hauteurBarre, 10, hauteurBarre + 1, RA8875_WHITE);
  }
  // puis affichage des graduations
  for (uint16_t i = 49; i <= 272; i = i + 58) tft.drawFastHLine (460, i, 6, RA8875_WHITE);
  for (uint16_t i = 20; i <= 272; i = i + 58) tft.drawFastHLine (460, i, 10, RA8875_WHITE);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                          Fonctions liées à l'AHRS
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void SendF()
{
  Serial5.clear();
  ndex = 0;
  Serial5.print('F');

  TimerStopReading.trigger(25ms);
}

void StopReading()
{
  dataReady = true;
}

void serialEvent5()
{
  tabTrame[ndex] = Serial5.read();
  ndex++;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                          Fonctions liées à l'encodeur rotatif et aux menus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void verificationValeurs() // Cette fonction vérifie si certains paramètres modifiés dans les menus restent dans les limites autorisées
{
  if (luminosite > 16) luminosite = 16;       
  if (luminosite < 1) luminosite = 1;
  if (dureeMinuteur > 60) dureeMinuteur = 60; 
  if (dureeMinuteur < 0) dureeMinuteur = 0;
  if(correctionHeureLocale < -12) correctionHeureLocale = -12;
  if(correctionHeureLocale > 12) correctionHeureLocale = 12;
}

// ************************************************************************* Routine d'interruption liée au traitement d'une pression sur le bouton *******************************************************************************
void Bouton_ISR() 
{
  if ((millis() - lastButtonPress) >= debounceTime)
  {
    lastButtonPress = millis ();
    Bouton_Clic = true;
  }
}
// ***************************************************************************************** Procédure d'ouverture du menu ******************************************************************************************************

void OuvreMenu (uint8_t ndx)
{
  indexOptionMenuEnCours = ndx;
  tft.fillRect (abscisseMenu, ordonneeMenu - hauteurCase + 2, tft.width() - abscisseMenu, hauteurCase * 2 - 2, BckgrndMenus);
  tft.drawLine(abscisseMenu + 1, ordonneeMenu - hauteurCase + 2, tft.width() - abscisseMenu, ordonneeMenu - hauteurCase + 2, RA8875_WHITE);
  encodeur.write(0);
  ValeurPositionEncodeur = 0;
  ValeurPositionEncodeurOld = 0;
  tft.setFont(Arial_11);
  tft.setTextColor (RA8875_WHITE, BckgrndMenus);
  tft.setCursor(abscisseMenu + 1, ordonneeMenu + 5 - hauteurCase);
  if (indexOptionMenuEnCours == 0)
  {
    tft.print("MENU GENERAL");
  }
  else
  {
    int8_t i = 0;
    tft.setTextColor (RA8875_WHITE, BckgrndMenus);
    while (menu[indexOptionMenuEnCours + i].numero != (menu[indexOptionMenuEnCours].numero) / 10) i--;
    tft.print(menu[indexOptionMenuEnCours + i].label);
  }
  for (byte n = ndx; n < ndx + menu[ndx].nbOptionsSoeurs; n++)
  {
    tft.setCursor ((largeurCase * (n - ndx)) + 2, ordonneeMenu + 3);
    tft.print(menu[n].label);
  }
  tft.drawRect ((ValeurPositionEncodeur * largeurCase), ordonneeMenu, largeurCase, hauteurCase, 0xFFFF); 
}
// ********************************************************************************************* Procédure de fermeture du menu **************************************************************************************************
void FermeMenu()
{
  Menu_Ouvert = false;
  Bouton_Clic = false;   
  tft.fillRect (abscisseMenu, ordonneeMenu - hauteurCase, tft.width() - abscisseMenu, hauteurCase * 2, RA8875_PINK);
  Horizon.redessine();
  tft.setFontDefault();
  tft.setFontScale (0);
}
// ************************************************ Cette fonction retourne, à partir d'un index donné quelconque, l'index de la première option soeur (numéro terminant par un zéro) **********************************************
uint8_t indexRetour(uint8_t indexQuelconque)
{
  int8_t i = 0;
  while (menu[indexQuelconque + i].numero > ((menu[indexQuelconque].numero) / 10) * 10) i--;
  return (indexQuelconque + i);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonctions utilisées lors de l'initialisation des options des menus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void initTable() // Cette fonction calcule et initialise les champs "nbOptionsSoeurs" et "indexOfOm" de chaque enregistrement, lors du setup.
{
  for (uint8_t i = 0; i < NbOptionsMenu(); i++)
  {
    uint16_t n = menu[i].numero;
    int8_t j = 0;
    if ((n / 10) * 10 == n)
    {
      while (menu[i + j].numero < n + 10) j++;
      for (uint8_t k = 0; k < j; k++) menu[i + k].nbOptionsSoeurs = j;
      j = 0;
    }
    if (menu[i].action == 'S')
    {
      while (menu[i + j].numero != n * 10) j++;
      menu[i].indexOfOm = i + j;
      j = 0;
    }
    if (menu[i].action == 'B')
    {
      while (menu[i + j].numero != (n / 100) * 10) j--;
      menu[i].indexOfOm = i + j;
    }
  }
}

void triBulleTableau() 
{
  char * texte; uint32_t nombre; char lettre;
  for (byte i = NbOptionsMenu() - 1; i > 0; i--)
  {
    for (byte j = 1; j <= i; j++)
    {
      if (menu[j - 1].numero > menu[j].numero)
      {
        texte  = menu[j].label;
        nombre = menu[j].numero;
        lettre = menu[j].action;
        menu[j].label        = menu[j - 1].label;
        menu[j].numero       = menu[j - 1].numero;
        menu[j].action       = menu[j - 1].action;
        menu[j - 1].label    = texte;
        menu[j - 1].numero   = nombre;
        menu[j - 1].action   = lettre;
      }
    }
  }
}

uint8_t NbOptionsMenu()
{
  uint8_t i = 0;
  while (menu[i].numero != 1000000) {
    i = i + 1;
  }
  return i;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonction utilisée pour le filtrage des données brutes issues des capteurs
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Fonction de filtre à Réponse Impulsionnelle Infinie (RII)
float filtrageRII (float valeurFiltreePrecedente, float valeurCourante , float coeffFiltrageRII)
{
  return valeurFiltreePrecedente  * (1 - coeffFiltrageRII) + valeurCourante * coeffFiltrageRII ;
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

void afficheHeure()
{
  // affiche l'heure locale digitale sur l'horloge
  tft.setFontDefault();
  tft.setFontScale(0);
  tft.setCursor (Chx - 31, Chy + 30);
  zt = now() + (3600 * correctionHeureLocale);
  printDigits(hour(zt));
  tft.print(":");
  printDigits(minute(zt));
  tft.print(":");
  printDigits(second(zt));
  // Calcul des angles des 3 aiguilles
  hs = zt % nbs12h;
  ms = zt % 3600;
  ss = zt % 60;
  angleH = 360 * hs / nbs12h;
  angleM = 360 * ms / 3600;
  angleS = 360 * ss / 60;
  // On efface les anciennes aiguilles
  tft.drawLineAngle (Chx, Chy, angleHold, LaigH, RA8875_PINK);
  tft.drawLineAngle (Chx, Chy, angleMold, LaigM, RA8875_PINK);
  tft.drawLineAngle (Chx, Chy, angleSold, LaigS, RA8875_PINK);
  // On affiche les nouvelles aiguilles
  tft.drawLineAngle (Chx, Chy, angleH, LaigH, 0xFFFF);
  tft.drawLineAngle (Chx, Chy, angleM, LaigM, 0xFFFF);
  tft.drawLineAngle (Chx, Chy, angleS, LaigS, RA8875_GREEN);
  // Sans oublier le petit cercle au centre
  tft.fillCircle (Chx, Chy, 3, RA8875_RED);
  //  Et on mémorise les positions des aiguilles
  angleHold = angleH;
  angleMold = angleM;
  angleSold = angleS;
}

void printDigits(int digits)
{
  if (digits < 10) tft.print('0');
  tft.print(digits);
}

void setflagAfficheHeure()
{
  flagAfficheHeure = true;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonctions utilitaires diverses
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void printDecimalInfCent(float nombre, byte nbDec) // Permet un affichage formaté à minima des nombres décimaux inférieurs à 100 et supérieurs à -100, avec alignement à droite (adapté aux températures))
{
  if (nombre >= 10) 
  {
    tft.print(" ");
    tft.print(nombre, nbDec);
    return;
  }
  if (nombre >= 0)  
  {
    tft.print("  ");
    tft.print(nombre, nbDec);
    return;
  }
  if (nombre > -10) 
  {
    tft.print(" ");
    tft.print(nombre, nbDec);
    return;
  }
  tft.print(nombre, nbDec);
}

void printDecimalInfDix(float nombre, byte nbDec) // Permet un affichage formaté à minima des nombres décimaux inférieurs à 10 et supérieurs à -10, avec alignement à droite (adapté aux G)
{
  if (nombre >= 0) 
  {
    tft.print(" ");
    tft.print(nombre, nbDec);
  }
  else tft.print(nombre, nbDec);
}

void sablier(uint8_t x, uint8_t y) // affiche un sablier
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

void photo(char str[], uint16_t largeur, uint16_t hauteur) 
{
fichier = SD.open(str, FILE_READ);
uint16_t xOrig, yOrig;
xOrig = (tft.width()- largeur)/2;
yOrig = (tft.height()- hauteur)/2;
if(fichier)
  {
    unsigned char LSB, MSB;
    uint16_t couleur;
    for (uint16_t j=1; j<=hauteur; j++)
      {
        for (uint16_t i = 1; i<=largeur; i++)
          {
            LSB = fichier.read();
            MSB = fichier.read();
            couleur = LSB + (MSB<<8);
            tft.drawPixel(i+xOrig, j+yOrig, couleur);
          }
      }
    fichier.close();
  } 
else {tft.print("Erreur a l'ouverture du fichier "); tft.println(str); delay(2000);}
}
