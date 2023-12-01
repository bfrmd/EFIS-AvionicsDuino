/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //                                                 HorizArt.cpp
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  Copyright (c) 2022 AvionicsDuino - benjamin.fremond@avionicsduino.com
  Contributors : Benjamin Frémond - Gabriel Consigny - Michel Riazuelo

    HorizArt.cpp is part of EFIS_Avionicsduino_V2.1

    EFIS_Avionicsduino_V2 is free software: you can redistribute it and/or modify
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
#include "HorizArt.h"

HorizArt::HorizArt(RA8875 * ptr)
{
  ptr_tft = ptr;
  BLUESKY = skyColor;
  OCHRE = groundColor;
  LD = Cx + DL;
  LG = Cx - DL;
  LH = Cy - DH;
  LB = Cy + DH;
  largeur = DL * 2;
  hauteur = DH * 2;
  Ybille = Cy + DH - ballRadius - 2;
  RayonGraduations = (hauteur / 3) + 20;
  racpiai = RayonGraduations - 19;
  demilonggrad = largeur / 14;
  k = 4;
  PasGrad = k * 10;
}

void HorizArt::begin()
{
  ptr_tft->setFontScale (1);
  ptr_tft->clearMemory();
  // Initialise l'utilisation des layers et dessine le ciel et la terre sur la layer2
  ptr_tft->writeTo(L2);
  ptr_tft->fillWindow(RA8875_BLACK);
  ptr_tft->fillRect (Cx - DL, Cy - DH, DL * 2, DH, OCHRE);
  ptr_tft->fillRect (Cx - DL, Cy, DL * 2, DH, BLUESKY);
  ptr_tft->setTransparentColor(RA8875_PINK);
  ptr_tft->writeTo(L1);
  // Et maintenant, on affiche la Layer 1, on lui attribue un fond rose, puis on la superpose sur la Layer 2 par application d'un effet "TRANSPARENT"
  ptr_tft->layerEffect(LAYER1);
  ptr_tft->fillWindow(RA8875_PINK);
  ptr_tft->layerEffect(TRANSPARENT);
  // Traçage de l'arc fixe et des graduations d'inclinaison de l'horizon artificiel, de la maquette, et du cadre de la bille
  reDraw();
  // On repasse en fonte par défaut
  ptr_tft->setFontDefault();
  ptr_tft->setFontScale (0);
}

void HorizArt::draw( float aRoll, float aPitch, uint8_t n, float aOffsetBille, bool menuOpen) // n en réserve pour l'avenir, inutile pour le moment.
{
  // ******************************************************************************************** On affiche d'abord sur le premier plan (layer 1) toutes les parties mobiles **************************************************************************************
  ptr_tft->writeTo(L1);
  ptr_tft->setTextColor(RA8875_WHITE, RA8875_PINK);
  ptr_tft->setFontDefault();
  ptr_tft->setFontScale (0);
  // Affichage des graduations mobiles de l'angle de pitch 
  sinroll = sin(aRoll);
  cosroll = cos(aRoll);
  curseurGx3 = Cx + (sinroll * -racpiai);
  curseurGy3 = Cy + (cosroll * -racpiai);
  // et on va afficher ce triangle
  float dist = 9.0;
  float alpha = 0.4;
  float beta  = PI - aRoll - alpha;
  float betaOld = PI - rollold - alpha;
  float gamma = PI - aRoll + alpha;
  float gammaOld = PI - rollold + alpha;
  curseurGx1 = curseurGx3 + (sin(beta) * dist);
  curseurGy1 = curseurGy3 + (cos(beta) * dist * -1);
  curseurGx2 = curseurGx3 + (sin(gamma) * dist);
  curseurGy2 = curseurGy3 + (cos(gamma) * dist * -1);
  curseurGxold1 = curseurGxold3 + (sin(betaOld) * dist);;
  curseurGyold1 = curseurGyold3 + (cos(betaOld) * dist * -1);
  curseurGxold2 = curseurGxold3 + (sin(gammaOld) * dist);
  curseurGyold2 = curseurGyold3 + (cos(gammaOld) * dist * -1);
  ptr_tft->drawTriangle (curseurGxold3, curseurGyold3, curseurGxold1, curseurGyold1, curseurGxold2, curseurGyold2, RA8875_PINK);
  ptr_tft->drawTriangle (curseurGx3, curseurGy3, curseurGx1, curseurGy1, curseurGx2, curseurGy2, RA8875_WHITE);
  for (float n = 0.5; n <= 2; n += 0.5)
  {
    byte m;
    if (int(n) == n) m = 1;
    else m = 2;
    dist = sqrt(pow(PasGrad * n, 2) + pow(demilonggrad / m, 2));
    alpha = atan((demilonggrad / m) / (PasGrad * n));
    beta  = PI - aRoll - alpha;
    betaOld = PI - rollold - alpha;
    gamma = PI - aRoll + alpha;
    gammaOld = PI - rollold + alpha;
    curseurGx1 = Cx + (sin(beta) * dist);
    curseurGy1 = Cy + (cos(beta) * dist * -1);
    curseurGx2 = Cx + (sin(gamma) * dist);
    curseurGy2 = Cy + (cos(gamma) * dist * -1);
    curseurGxold1 = Cx + (sin(betaOld) * dist);;
    curseurGyold1 = Cy + (cos(betaOld) * dist * -1);
    curseurGxold2 = Cx + (sin(gammaOld) * dist);
    curseurGyold2 = Cy + (cos(gammaOld) * dist * -1);
    ptr_tft->drawLine (curseurGxold1, curseurGyold1, curseurGxold2, curseurGyold2, RA8875_PINK);
    ptr_tft->drawLine (curseurGx1, curseurGy1, curseurGx2, curseurGy2, RA8875_WHITE);
    ptr_tft->drawLine (Cx + (curseurGxold1 - Cx) * -1, Cy + (curseurGyold1 - Cy) * -1, Cx + (curseurGxold2 - Cx) * -1, Cy + (curseurGyold2 - Cy) * -1, RA8875_PINK);
    ptr_tft->drawLine (Cx + (curseurGx1 - Cx) * -1, Cy + (curseurGy1 - Cy) * -1, Cx + (curseurGx2 - Cx) * -1, Cy + (curseurGy2 - Cy) * -1, RA8875_WHITE);
  }

  //****************************************************************************************************** Puis on dessine ensuite la terre et le ciel sur la Layer 2 *****************************************************************************************************
  ptr_tft->writeTo(L2);
  if ((aRoll > PI / 2) | (aRoll < -PI / 2)) // pour le vol inversé, on inverse les couleurs du ciel et de la terre.
  {
    OCHRE         = groundColor; // 0b 0000 0100 0001 1111
    BLUESKY     = skyColor; // 0b 1011 1010 1100 0011
  }
  else
  {
    BLUESKY     = groundColor; // 0b 0000 0100 0001 1111
    OCHRE         = skyColor; // 0b 1011 1010 1100 0011
  }

  tanroll = tan(aRoll);
  if (tanroll > 600) 
  {
    tanroll = 600;
  }
  if (tanroll < -600) 
  {
    tanroll = -600;
  }
  g = DH + (k * aPitch * (cosroll + (sinroll * tanroll))) + (DL * tanroll);
  d = g - (largeur * tanroll);
  // Determination du cas actuel, toujours en coordonnées "cadre"
  if (g < 0) {
    casG = 10;
  }
  else if (g > hauteur) 
  {
    casG = 30;
  }
  else {
    casG = 20;
  }
  if (d < 0) 
  {
    casD = 1;
  }
  else if (d > hauteur) 
  {
    casD = 3;
  }
  else 
  {
    casD = 2;
  }

  switch (casG + casD)
  {
    case 22:
      fillQuadrilatere (LG, g + LH, LG, LH, LD, LH, LD, d + LH, BLUESKY);
      fillQuadrilatere ( LG, LB, LG, g + LH, LD, d + LH, LD, LB, OCHRE);
      break;
    case 12:
      h = min(g / tanroll, 2 * DL);
      fillPentagone (LG, LB, LG, LH, h + LG, LH, LD, d + LH, LD, LB, OCHRE);
      ptr_tft->fillTriangle (h + LG, LH, LD, LH, LD, d + LH, BLUESKY);
      break;
    case 13:
      h = (g / tanroll);
      b = (g - hauteur) / tanroll;
      fillQuadrilatere (LG, LB, LG, LH, h + LG, LH, b + LG, LB, OCHRE);
      fillQuadrilatere (h + LG, LH, LD, LH, LD, LB, b + LG, LB, BLUESKY);
      break;
    case 21:
      h = min(g / tanroll, 2 * DL);
      ptr_tft->fillTriangle (LG, g + LH, LG, LH, h + LG, LH, BLUESKY);
      fillPentagone (LG, LB, LG, g + LH, h + LG, LH, LD, LH, LD, LB, OCHRE);
      break;

    case 23:
      b = min((g - hauteur) / tanroll, 2 * DL); 
      ptr_tft->fillTriangle(LG, g + LH, b + LG, LB, LG, LB, OCHRE);
      fillPentagone (LG, g + LH, LG, LH, LD, LH, LD, LB, b + LG, LB, BLUESKY);
      break;
    case 31:
      h = (g / tanroll);
      b = (g - hauteur) / tanroll;
      fillQuadrilatere (LG, LB, LG, LH, h + LG, LH, b + LG, LB, BLUESKY);
      fillQuadrilatere (h + LG, LH, LD, LH, LD, LB, b + LG, LB, OCHRE);
      break;
    case 32:
      b = min((g - hauteur) / tanroll, 2 * DL); // corr CSY 23/09/20
      ptr_tft->fillTriangle(b + LG, LB, LD, LB, LD, d + LH, OCHRE);
      fillPentagone (b + LG, LB, LG, LB, LG, LH, LD, LH, LD, d + LH, BLUESKY);
      break;
    case 33:
      ptr_tft->fillRect (LG, LH, largeur, hauteur, BLUESKY);
      break;
    case 11:
      ptr_tft->fillRect (LG, LH, largeur, hauteur, OCHRE);
      break;
    default:
      break;
  }

  // ****************************************************** On affiche la bille sur la Layer 1 (premier plan) *******************************************************
  if (!menuOpen)
  {
    ptr_tft->writeTo(L1);
    bille (offsetBilleOld, RA8875_PINK);
    bille(aOffsetBille, RA8875_WHITE);
  }

  // ************************************* Dessin partiel de la maquette dont les ailes et l'empennage peuvent être "écornés" lors des virages par les graduations de pitch ********************************************
  ptr_tft->fillRect (Cx - 27, Cy - 2, 9, 4, RA8875_WHITE);
  ptr_tft->fillRect (Cx + 19, Cy - 2, 10, 4, RA8875_WHITE);
  ptr_tft->drawLine (Cx - 27, Cy - 3, Cx - 19, Cy - 3, RA8875_BLACK);
  ptr_tft->drawLine (Cx - 27, Cy + 2, Cx - 19, Cy + 2, RA8875_BLACK);
  ptr_tft->drawLine (Cx + 19, Cy - 3, Cx + 27, Cy - 3, RA8875_BLACK);
  ptr_tft->drawLine (Cx + 19, Cy + 2, Cx + 27, Cy + 2, RA8875_BLACK);
  ptr_tft->drawRect(Cx - 3, Cy - 23, 6, 5, RA8875_BLACK);
  ptr_tft->fillRect(Cx - 2, Cy - 22, 4, 5, RA8875_WHITE);

  // ********************* On enregistre toutes les positions clés pour pouvoir effaccer les anciennes graduations au prochain passage dans Loop. *******************************************
  curseurGxold3 = curseurGx3;
  curseurGyold3 = curseurGy3;
  rollold = aRoll;
  offsetBilleOld = aOffsetBille;
  Mxold = Mx;
  Myold = My;
}

void HorizArt::reDraw()
{
  ptr_tft->layerEffect(LAYER2); delay(1);
  for (uint8_t n = 0; n <= 5; n++)
  {
    ptr_tft->drawLineAngle  (Cx, Cy, AngleGraduationCourte[n], RayonGraduations - 5, RA8875_WHITE);
  }
  for (uint8_t n = 0; n <= 4; n++)
  {
    ptr_tft->drawLineAngle (Cx, Cy, AngleGraduationLongue[n], RayonGraduations + 2, RA8875_WHITE);
  }
  ptr_tft->fillCircle (Cx, Cy, RayonGraduations - 18, RA8875_PINK);
  // Dessin de l'arc fixe
  ptr_tft->drawArc (Cx, Cy, RayonGraduations - 15, 2, 300, 60, RA8875_WHITE);
  ptr_tft->layerEffect(TRANSPARENT);

  // Dessin de la maquette
  ptr_tft->drawCircle(Cx, Cy, 10, RA8875_BLACK);
  ptr_tft->fillRect (Cx - 38, Cy - 2, 76, 4, RA8875_WHITE);
  ptr_tft->drawRect(Cx - 38, Cy - 3, 76, 6, RA8875_BLACK);
  ptr_tft->fillRect(Cx - 2, Cy - 22, 4, 17, RA8875_WHITE);
  ptr_tft->drawRect(Cx - 3, Cy - 22, 6, 17, RA8875_BLACK);
  ptr_tft->drawArc(Cx, Cy, 9, 7, 0, 360, RA8875_WHITE);
  ptr_tft->fillCircle(Cx, Cy, 5, RA8875_PINK);
  ptr_tft->drawCircle(Cx, Cy, 5, RA8875_BLACK);

  // Dessin du cadre de la bille
  ptr_tft->drawArc (Cx - ballRadius * 8 - 1, Ybille, ballRadius + 2, 1, 180, 360, RA8875_BLACK);
  ptr_tft->drawArc (Cx + ballRadius * 8 + 1, Ybille, ballRadius + 2, 1, 0, 180, RA8875_BLACK);
  ptr_tft->drawLine (Cx - ballRadius * 8 - 1, Ybille - ballRadius - 1, Cx + ballRadius * 8 + 1, Ybille - ballRadius - 1, RA8875_BLACK);
  ptr_tft->drawLine (Cx - ballRadius * 8 - 1, Ybille + ballRadius + 1, Cx + ballRadius * 8 + 1, Ybille + ballRadius + 1, RA8875_BLACK);

  // Dessin partiel du cadre graphique du vario
  ptr_tft->setFontDefault();
  ptr_tft->setFontScale (0);
  ptr_tft->setTextColor (RA8875_WHITE, RA8875_PINK);
  byte j = 0;
  for (uint16_t i = 20; i <= 272; i = i + 58) 
  {
    ptr_tft->setCursor(470, i - 9);
    ptr_tft->print(abs(2 - j));
    j++;
  }
  ptr_tft->drawFastVLine (460, 20, 232, RA8875_WHITE);

  // Dessin du cadre d'affichage de l'altitude QNH
  ptr_tft->fillRect(406, 117, 50, 38, RA8875_BLACK);
  ptr_tft->drawRect(406, 117, 50, 38, RA8875_LIGHT_GREY);

  // Dessin du cadre d'affichage de l'altitude QFE
  ptr_tft->fillRect(406, 97, 50, 20, RA8875_BLACK);
  ptr_tft->drawRect(406, 97, 50, 20, RA8875_LIGHT_GREY);
  
  // Dessin des cadres d'affichage des vitesses
  ptr_tft->fillRect(2, 117, 60, 38, RA8875_BLACK);
  ptr_tft->drawRect(2, 117, 60, 38, RA8875_LIGHT_GREY);
  //ptr_tft->drawRect(2,96,60,20, RA8875_LIGHT_GREY);

  // Affichage des légendes fixes
  ptr_tft->setCursor(318, 3); ptr_tft->print("TRK");
  ptr_tft->setCursor(138, 3); ptr_tft->print("HDG");
  ptr_tft->setCursor(2, 82); ptr_tft->print("GND");
  ptr_tft->setCursor(2, 97); ptr_tft->print("TAS");
  ptr_tft->setCursor(378, 157); ptr_tft->print("AltP");
  ptr_tft->setCursor(378, 172); ptr_tft->print("AltD");
  ptr_tft->setCursor(378, 187); ptr_tft->print("GPS");
  ptr_tft->setCursor(378, 98); ptr_tft->print("QFE");
  ptr_tft->setCursor(378, 127); ptr_tft->print("QNH");
  ptr_tft->setCursor(378, 205); ptr_tft->print("QNH");
  ptr_tft->setCursor(378, 220); ptr_tft->print("QFE");
  ptr_tft->setCursor (71, 160); ptr_tft->print("IAT");
  ptr_tft->setCursor (71, 175); ptr_tft->print("OAT");
  ptr_tft->setCursor (71, 190); ptr_tft->print(" RH");
  ptr_tft->setCursor (71, 205); ptr_tft->print("DPT");
  ptr_tft->setCursor (40,0); ptr_tft->print("WIND");
  ptr_tft->setCursor (3, 15); ptr_tft->print("Dir  ");
  ptr_tft->setCursor (3, 30); ptr_tft->print("Spd");
  ptr_tft->setCursor (385, 0); ptr_tft->print("V. SPEED");
  ptr_tft->setCursor (378, 15); ptr_tft->print("AHRS"); 
  ptr_tft->setCursor (378, 30); ptr_tft->print("BARO");  
  ptr_tft->setCursor (30, 68); ptr_tft->print("SPEEDS");   

  // Dessin des cadres d'affichage de la route et du cap magnétique
  ptr_tft->fillRect(255, 1, 60, 22, RA8875_BLACK);
  ptr_tft->drawRect(255, 1, 60, 22, RA8875_LIGHT_GREY);
  ptr_tft->fillRect(165, 1, 60, 22, RA8875_BLACK);
  ptr_tft->drawRect(165, 1, 60, 22, RA8875_LIGHT_GREY);

  // Dessin des cadres d'affichages températures, G, altP&D, QNH/QFE,varios, wind, nbSat, speeds (dans cet ordre)
  //ptr_tft->drawRect(68, 155, 80, 65, RA8875_WHITE);
  ptr_tft->drawRect(374, 47, 84, 49, RA8875_WHITE);
  ptr_tft->drawRect(374, 95, 84, 110, RA8875_WHITE);
  ptr_tft->drawRect(374, 204, 84, 35, RA8875_WHITE);
  ptr_tft->drawRect(374, 1, 84, 47, RA8875_WHITE);
  ptr_tft->drawRect(1, 1, 100, 47, RA8875_WHITE);
  ptr_tft->drawRect(1, 47, 100, 20, RA8875_WHITE);
  ptr_tft->drawRect(1, 66, 100, 91, RA8875_WHITE);
  
}

// ************************************************************************************** Quelques fonctions primitives graphiques ********************************************************************************************

void HorizArt::fillPentagone( int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4, uint16_t color)
{
  ptr_tft->fillTriangle (x0, y0, x1, y1, x4, y4, color);
  ptr_tft->fillTriangle (x1, y1, x2, y2, x4, y4, color);
  ptr_tft->fillTriangle (x2, y2, x3, y3, x4, y4, color);
}

void HorizArt::fillQuadrilatere ( int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, uint16_t color)
{
  ptr_tft->fillTriangle (x0, y0, x1, y1, x3, y3, color);
  ptr_tft->fillTriangle (x1, y1, x2, y2, x3, y3, color);
}

// ********************************************************************************************************** Fonction liée à la bille ******************************************************************************************************************************
void HorizArt::bille( int8_t ofs, uint16_t couleur)
{
  ptr_tft->drawCircle (Cx + ofs, Ybille, ballRadius, couleur);
  ptr_tft->drawCircle (Cx + ofs, Ybille, 5, couleur);
  ptr_tft->fillRect (Cx - ballRadius - 4, LH + hauteur - ballRadius * 2 - 2, 2, ballRadius * 2 + 1, RA8875_BLACK);
  ptr_tft->fillRect (Cx + ballRadius + 3, LH + hauteur - ballRadius * 2 - 2, 2, ballRadius * 2 + 1, RA8875_BLACK);
}
