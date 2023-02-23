//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                             Bibliothèque HorizArt : création d'un horizon artificiel complet,
//        prêt à être intégré dans un programme d'EFIS utilisant un écran basé sur un contrôleur RA8875
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //                                                 HorizArt.h
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

HorizArt.h is part of EFIS_Avionicsduino_V2.2
EFIS_Avionicsduino_V2.2 is free software 

MIT License (MIT)

Copyright (c) 2023 AvionicsDuino - benjamin.fremond@avionicsduino.com
Contributors : Benjamin Frémond - Gabriel Consigny - Michel Riazuelo

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
