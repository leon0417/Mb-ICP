#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include "readfloat.h"


using namespace::std;

void ReadTxtByLine(string pathSrc,vector<string> &strBufferVec)
{
//	bool bol=false;
	fstream infile(pathSrc.c_str());
	if (!infile)
	{
		cout<<"打开文件失败"<<endl;
		exit(1);
	}
	cout<<"开始读取"<<endl;
	string strLine;
	while (getline(infile,strLine))
	{
		strBufferVec.push_back(strLine);
	}
	cout<<"文件读取完毕"<<endl;
	infile.close();
}

void readfloat( vector<string> strBufferVec, float *vec, int num )
{
	vector<string> stringVec;
	int cnt = 0;
	for( vector<string>::iterator iterLine = strBufferVec.begin(); iterLine != strBufferVec.end(); ++iterLine )
	{
		stringstream str;
		str<<*iterLine;
		str>>vec[cnt];
		cnt++;
	}
}

void read( const char* address, float* vec )
{
	vector<string> strBufferVec;
	ReadTxtByLine(address, strBufferVec);
	readfloat( strBufferVec, vec, 721 );
}