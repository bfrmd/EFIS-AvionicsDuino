//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                         Paramètres généraux de la bibiothèque HorizArt
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //                                                 HorizArtParamGen.h
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  Copyright (c) 2022 AvionicsDuino - benjamin.fremond@avionicsduino.com
  Contributors : Benjamin Frémond - Gabriel Consigny - Michel Riazuelo

    HorizArtParamGen.h is part of EFIS_Avionicsduino_V2.1

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
#ifndef HORIZART_PARAMGEN_H
#define HORIZART_PARAMGEN_H

//Broches pour l'écran sur SPI1
#define RA8875_CS1 0
#define RA8875_MOSI1 26
#define RA8875_MISO1 1
#define RA8875_SCLK1 27

// Couleurs personnalisées ne figurant pas dans la bibliothèque RA8875
#define skyColor      0xBAC3    // 0b 1011 1010 1100 0011
#define groundColor     0x041F    // 0b 0000 0100 0001 1111
#define BckgrndMenus 0b1111

//****************** Taille et position du cadre de l'horizon **********************
#define Cx 240
#define Cy 136
#define DH 135
#define DL 239

//****************** Rayon de la bille **********************
#define ballRadius 10

#endif
