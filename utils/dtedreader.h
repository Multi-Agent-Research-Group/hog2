/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2017 All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef DTEDREADER_H
#define DTEDREADER_H

#define FILE_HEADER 3428
#define NUM_RECORDS 1201
#define COL_HEADER 4 // 2-byte words
#define COL_FOOTER 2 // 2-byte words
// Reads a DTED1 (3 arc-second resolution) file
inline bool readdted1( const char* filename, float** elevations, unsigned width, unsigned height, unsigned xoffset=0, unsigned yoffset=0, float rescale=0){

  // Sanity check
  if(NUM_RECORDS-width-xoffset < 1)
    return false;

  if(NUM_RECORDS-height-yoffset < 1)
    return false;

  float minVal(9999999);
  float maxVal(0);

  // Open file
  FILE* fin;
  if(( fin = fopen(filename,"rb")) == NULL){
    return false;
  }
  //Throw away 3428 bytes of header
  uint16_t dummy[NUM_RECORDS+COL_HEADER+COL_FOOTER];
  for(int i(0); i<FILE_HEADER; ++i) fread(dummy,1,1,fin);

  // Skip unwanted columns
  for(int i(0); i<xoffset; ++i) fread(dummy,sizeof(uint16_t),NUM_RECORDS+COL_HEADER+COL_FOOTER,fin);

  // Read in records
  for(int lon(0); lon<width; ++lon){
    fread(dummy,sizeof(int16_t),COL_HEADER,fin); // Column header
    fread(dummy,sizeof(uint16_t),yoffset,fin); // Skip unwanted rows
    fread(dummy,sizeof(uint16_t),height,fin); // Read data directly into array
    // Swap bytes...
    for(int i(0); i<height; ++i){
      elevations[lon][i]=(((dummy[i]&0xff00) >> 8) + ((dummy[i]&0x00ff) << 8));
      minVal=std::min(minVal,elevations[lon][i]);
      maxVal=std::max(maxVal,elevations[lon][i]);
    }
    fread(dummy,sizeof(uint16_t),NUM_RECORDS-width-yoffset,fin); // Skip remainder
    fread(dummy,sizeof(int16_t),COL_FOOTER,fin); // Column footer
  }
  fclose(fin);
  if(rescale){
    for(int lon(0); lon<width; ++lon){
      for(int lat(0); lat<height; ++lat){
        elevations[lon][lat]=(elevations[lon][lat]-minVal)*rescale/float(maxVal-minVal);
      }
    }
  }
  return true;
}

#endif
