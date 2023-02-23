//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                             Bibliothèque HorizArt : création d'un horizon artificiel complet,
//        prêt à être intégré dans un programme d'EFIS utilisant un écran basé sur un contrôleur RA8875
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //                                                 HorizArt.h
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  Copyright (c) 2022 AvionicsDuino - benjamin.fremond@avionicsduino.com
  Contributors : Benjamin Frémond - Gabriel Consigny - Michel Riazuelo

    HorizArt.h is part of EFIS_Avionicsduino_V2.1

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
#ifndef HORIZART_H
#define HORIZART_H

#include <SPI.h>
#include <RA8875.h>
#include "HorizArtParamGen.h"

class HorizArt
{
    //  -------------------------------------------------------------------- Attributs de la classe --------------------------------------------------------------------------------------------
  private:

    RA8875 * ptr_tft = 0;

    // Variables définissant la taille et la position du cadre
    int LD, LG, LH, LB;
    int largeur, hauteur;
    int Ybille;

    // Variables utilisées pour tracer l'arc fixe de l'indicateur d'angle d'inclinaison et ses graduations 
    const int AngleGraduationCourte[6] = {315, 340, 350, 10, 20, 45};
    const int AngleGraduationLongue[5] = {300, 330, 360, 30, 60};
    byte RayonGraduations;
    byte racpiai;

    // Variables utilisées pour tracer les graduations mobiles du pitch ****** les constantes gagneront à devenir des variables initialisées (en fonction de la taille de l'écran) dans le constructeur
    float PasGrad;
    float demilonggrad;
    float k;

    // variables "de service" servant à "enregistrer" une position quelconque de "curseur graphique" en vue d'un tracé (ou d'un effacement) de ligne (ou autre) à partir de ce point.
    int curseurGx1, curseurGy1;
    int curseurGx2, curseurGy2;
    int curseurGx3, curseurGy3;
    int curseurGxold1, curseurGyold1;
    int curseurGxold2, curseurGyold2;
    int curseurGxold3 = 7;
    int curseurGyold3 = 7;
    int Mx, My;
    int Mxold, Myold;

    //  Variables utilisées pour le calcul de la position puis l'affichage de l'horizon
    int g;
    int d;
    int h;
    int b;
    byte casG, casD;

    // Variables relatives à la position de l'avion et à la symétrie du vol
    float roll = 0.0;
    float rollold = 0.0;
    float pitch = 0.0;
    float sinroll, cosroll, tanroll;

    int16_t offsetBille = 0;
    int16_t offsetBilleOld;

    // Variables diverses 
    uint16_t  BLEUCIEL;
    uint16_t  OCRE;

    //  --------------------------------------------------------------------- Méthodes de la classe ---------------------------------------------------------------------------------------------------------------

    void fillPentagone(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4, uint16_t color);
    void fillQuadrilatere (int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, uint16_t color);
    void triangleIso (int xS, int yS, int angle, int longueur, uint16_t couleur);
    void bille(int8_t ofs, uint16_t couleur);


  public:

    HorizArt(RA8875 * ptr);
    void begin();
    void dessine(float aRoll, float aPitch, float aYaw, float aOffsetBille, bool aMenuOUvert);
    void redessine();
};

#endif
