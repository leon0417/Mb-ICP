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
		cout<<"���ļ�ʧ��"<<endl;
		exit(1);
	}
	cout<<"��ʼ��ȡ"<<endl;
	string strLine;
	while (getline(infile,strLine))
	{
		strBufferVec.push_back(strLine);
	}
	cout<<"�ļ���ȡ���"<<endl;
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