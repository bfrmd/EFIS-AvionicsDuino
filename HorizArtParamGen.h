//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                         Paramètres généraux de la bibiothèque HorizArt
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //                                                 HorizArtParamGen.h
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

HorizArtParamGen.h is part of EFIS_Avionicsduino_V2.2
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
#ifndef HORIZART_PARAMGEN_H
#define HORIZART_PARAMGEN_H

//Broches pour l'écran sur SPI1
#define RA8875_CS1 0
#define RA8875_MOSI1 26
#define RA8875_MISO1 1
#define RA8875_SCLK1 27

// Couleurs personnalisées ne figurant pas dans la bibliothèque RA8875
#define ciel      0xBAC3    // 0b 1011 1010 1100 0011
#define terre     0x041F    // 0b 0000 0100 0001 1111
#define VERTFONCE 0x0440    // 0b 0000 0100 0100 0000
#define ORANGE    0xFD00    // 0b 1111 1101 0000 0000
#define BckgrndMenus 0b1111

//****************** Taille et position du cadre de l'horizon **********************
#define Cx 240
#define Cy 136
#define DH 135
#define DL 239

//****************** Taille de la bille **********************
#define Rbille 10

#endif
