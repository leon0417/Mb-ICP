#ifndef READFLOAT_H
#define READFLOAT_H

void ReadTxtByLine(std::string pathSrc,std::vector<std::string> &strBufferVec);
void readfloat(std::vector<std::string> strBufferVec, float *vec, int num );
void read( const char* address, float* vec );

#endif