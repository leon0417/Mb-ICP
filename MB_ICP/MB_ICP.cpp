// MBICP.cpp : ｶｨﾒ蠢ﾘﾖﾆﾌｨﾓｦﾓﾃｳﾌﾐﾄﾈ�ｿﾚｵ罍｣
//

#include "stdafx.h"
#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
//#include <string.h>
#include <time.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "readfloat.h"

#include "mbicp.h"

using namespace std;

////////Mb-ICP///////////////////
TSMparams myConfig={0.87222222, 1.0, 3, 2, 1, 0.5, 0.85, 0.1, 100, 0.001, 0.001, 0.001, 0.001, 2};
mbicp m_mbicp( myConfig, 721, 30 );
Tpfp laserRef[721], laserNew[721];
ArPose robot(0,0,0), robot1;
matrix cov1;
float xx,yy,tt;
////////Mb-ICP///////////////////
const clock_t PER_SEC = 1000;

enum {
	Timeout = 1000,               // [msec]
	EachTimeout = 2,              // [msec]
	LineLength = 64 + 3 + 1 + 1 + 1 + 16,
};

static HANDLE HCom = INVALID_HANDLE_VALUE;
static int ReadableSize = 0;
static char* ErrorMessage = "no error.";

typedef struct
{
	enum 
	{
		MODL = 0,                   //!< Sensor model information
		DMIN,                       //!< Minimum measurable distance [mm]
		DMAX,                       //!< Maximum measurable distance [mm]
		ARES,                       //!< Angle of resolution
		AMIN,                       //!< Minimum measurable area
		AMAX,                       //!< Maximum measurable area
		AFRT,                       //!< Front direction value
		SCAN,                       //!< Standard angular velocity
	};
	string model;                 //!< Obtained MODL information
	long distance_min;            //!< Obtained DMIN information
	long distance_max;            //!< Obtained DMAX information
	int area_total;               //!< Obtained ARES information
	int area_min;                 //!< Obtained AMIN information
	int area_max;                 //!< Obtained AMAX information
	int area_front;               //!< Obtained AFRT information
	int scan_rpm;                 //!< Obtained SCAN information
	
	int first;                    //!< Starting position of measurement
	int last;                     //!< End position of measurement
	int max_size;                 //!< Maximum size of data
	long last_timestamp;          //!< Time stamp when latest data is obtained
} urg_state_t;

static void delay(int msec)
{
	Sleep(msec);
}

static int com_changeBaudrate(long baudrate)
{
	DCB dcb;
	
	GetCommState(HCom, &dcb);
	dcb.BaudRate = baudrate;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.fParity = FALSE;
	dcb.StopBits = ONESTOPBIT;
	SetCommState(HCom, &dcb);
	
	return 0;
}

static int com_connect(const char* device, long baudrate)
{
#if defined(RAW_OUTPUT)
	Raw_fd_ = fopen("raw_output.txt", "w");
#endif
	
	char adjust_device[16];
	_snprintf(adjust_device, 16, "\\\\.\\%s", device);
	HCom = CreateFileA(adjust_device, 
		GENERIC_READ | GENERIC_WRITE, 
		0,
		NULL, 
		OPEN_EXISTING, 
		FILE_ATTRIBUTE_NORMAL, 
		NULL);
	
	if (HCom == INVALID_HANDLE_VALUE) 
	{
		return -1;
	}
	
	// Baud rate setting
	return com_changeBaudrate(baudrate);
}

static void com_disconnect(void)
{
	if (HCom != INVALID_HANDLE_VALUE) 
	{
		CloseHandle(HCom);
		HCom = INVALID_HANDLE_VALUE;
	}
}

static int com_send(const char* data, int size)
{
	DWORD n;
	WriteFile(HCom, data, size, &n, NULL);
	return n;
}

static int com_recv(char* data, int max_size, int timeout)
{
	if (max_size <= 0)
	{
		return 0;
	}
	
	if (ReadableSize < max_size) 
	{
		DWORD dwErrors;
		COMSTAT ComStat;
		ClearCommError(HCom, &dwErrors, &ComStat);
		ReadableSize = ComStat.cbInQue;
	}
	
	if (max_size > ReadableSize) 
	{
		COMMTIMEOUTS pcto;
		int each_timeout = 2;
		
		if (timeout == 0) 
		{
			max_size = ReadableSize;
		} 
		else 
		{
			if (timeout < 0) 
			{
				/* If timeout is 0, this function wait data infinity */
				timeout = 0;
				each_timeout = 0;
			}
			
			/* set timeout */
			GetCommTimeouts(HCom, &pcto);
			pcto.ReadIntervalTimeout = timeout;
			pcto.ReadTotalTimeoutMultiplier = each_timeout;
			pcto.ReadTotalTimeoutConstant = timeout;
			SetCommTimeouts(HCom, &pcto);
		}
	}
	
	DWORD n;
	ReadFile(HCom, data, (DWORD)max_size, &n, NULL);
#if defined(RAW_OUTPUT)
	if (Raw_fd_) 
	{
		for (int i = 0; i < n; ++i) 
		{
			fprintf(Raw_fd_, "%c", data[i]);
		}
		fflush(Raw_fd_);
	}
#endif
	if (n > 0) 
	{
		ReadableSize -= n;
	}
	
	return n;
}

// The command is transmitted to URG
static int urg_sendTag(const char* tag)
{
	char send_message[LineLength];
	_snprintf(send_message, LineLength, "%s\n", tag);
	int send_size = (int)strlen(send_message);
	com_send(send_message, send_size);
	
	return send_size;
}

// Read one line data from URG
static int urg_readLine(char *buffer)
{
	int i;
	for (i = 0; i < LineLength -1; ++i) 
	{
		char recv_ch;
		int n = com_recv(&recv_ch, 1, Timeout);
		if (n <= 0) 
		{
			if (i == 0) 
			{
				return -1;              // timeout
			}
			break;
		}
		if ((recv_ch == '\r') || (recv_ch == '\n')) 
		{
			break;
		}
		buffer[i] = recv_ch;
	}
	buffer[i] = '\0';
	
	return i;
}

// Trasmit command to URG and wait for response
static int urg_sendMessage(const char* command, int timeout, int* recv_n)
{
	int send_size = urg_sendTag(command);
	int recv_size = send_size + 2 + 1 + 2;
	char buffer[LineLength];
	
	int n = com_recv(buffer, recv_size, timeout);
	*recv_n = n;
	
	if (n < recv_size) 
	{
		// if received data size is incorrect
		return -1;
	}
	
	if (strncmp(buffer, command, send_size -1)) 
	{
		// If there is mismatch in command
		return -1;
	}
	
	// !!! check checksum here
	
	// Convert the response string into hexadecimal number and return that value
	char reply_str[3] = "00";
	reply_str[0] = buffer[send_size];
	reply_str[1] = buffer[send_size + 1];
	return strtol(reply_str, NULL, 16);
}

// Change baudrate
static int urg_changeBaudrate(long baudrate)
{
	char buffer[] = "SSxxxxxx\r";
	_snprintf(buffer, 10, "SS%06d\r", baudrate);
	int dummy = 0;
	int ret = urg_sendMessage(buffer, Timeout, &dummy);
	
	if ((ret == 0) || (ret == 3) || (ret == 4)) 
	{
		return 0;
	} 
	else 
	{
		return -1;
	}
}

// Read out URG parameter
static int urg_getParameters(urg_state_t* state)
{
	// Read parameter
	urg_sendTag("PP");
	char buffer[LineLength];
	int line_index = 0;
	enum 
	{
		TagReply = 0,
		DataReply,
		Other,
	};
	int line_length;
	for (; (line_length = urg_readLine(buffer)) > 0; ++line_index) 
	{
		if (line_index == Other + urg_state_t::MODL) 
		{
			buffer[line_length - 2] = '\0';
			state->model = &buffer[5];
		} 
		else if (line_index == Other + urg_state_t::DMIN) 
		{
			state->distance_min = atoi(&buffer[5]);
		} 
		else if (line_index == Other + urg_state_t::DMAX) 
		{
			state->distance_max = atoi(&buffer[5]);
		}
		else if (line_index == Other + urg_state_t::ARES) 
		{
			state->area_total = atoi(&buffer[5]);
		} 
		else if (line_index == Other + urg_state_t::AMIN) 
		{
			state->area_min = atoi(&buffer[5]);
			state->first = state->area_min;
		} 
		else if (line_index == Other + urg_state_t::AMAX) 
		{
			state->area_max = atoi(&buffer[5]);
			state->last = state->area_max;
		} 
		else if (line_index == Other + urg_state_t::AFRT) 
		{
			state->area_front = atoi(&buffer[5]);
		} 
		else if (line_index == Other + urg_state_t::SCAN) 
		{
			state->scan_rpm = atoi(&buffer[5]);
		}
	}
	
	if (line_index <= Other + urg_state_t::SCAN) 
	{
		return -1;
	}
	// Calculate the data size
	state->max_size = state->area_max +1;
	
	return 0;
}

/*!
  \brief Connection to URG

  \param state [o] Sensor information
  \param port [i] Device
  \param baudrate [i] Baudrate [bps]

  \retval 0 Success
  \retval < 0 Error
*/

static int urg_connect(urg_state_t* state, const char* port, const long baudrate)
{
	static char message_buffer[LineLength];
	
	if (com_connect(port, baudrate) < 0) 
	{
		_snprintf(message_buffer, LineLength, "Cannot connect COM device: %s", port);
		ErrorMessage = message_buffer;
		return -1;
	}
	
	const long try_baudrate[] = { 19200, 115200, 38400 };
	size_t n = sizeof(try_baudrate) / sizeof(try_baudrate[0]);
	for (size_t i = 0; i < n; ++i) 
	{
		// Search for the communicate able baud rate by trying different baud rate
		if (com_changeBaudrate(try_baudrate[i])) 
		{
			ErrorMessage = "change baudrate fail.";
			return -1;
		}
		
		// Change to SCIP2.0 mode
		int recv_n = 0;
		urg_sendMessage("SCIP2.0", Timeout, &recv_n);
		if (recv_n <= 0) 
		{
			// If there is difference in baud rate value,then there will be no
			// response. So if there is no response, try the next baud rate.
			continue;
		}
		
		// If specified baudrate is different, then change the baudrate
		if (try_baudrate[i] != baudrate) 
		{
			urg_changeBaudrate(baudrate);
			
			// Wait for SS command applied.
			delay(100);
			
			com_changeBaudrate(baudrate);
		}
		
		// Get parameter
		if (urg_getParameters(state) < 0) 
		{
			ErrorMessage =
				"PP command fail.\n"
				"This COM device may be not URG, or URG firmware is too old.\n"
				"SCIP 1.1 protocol is not supported. Please update URG firmware.";
			return -1;
		}
		state->last_timestamp = 0;
		
		// success
		return 0;
	}
	
	// fail
	ErrorMessage = "no urg ports.";
	return -1;
}

/*!
  \brief Disconnection
*/

static void urg_disconnect(void)
{
	com_disconnect();
}

/*!
  \brief Receive range data by using GD command

  \param state[i] Sensor information

  \retval 0 Success
  \retval < 0 Error
*/

static int urg_captureByGD(const urg_state_t* state)
{
	char send_message[LineLength];
	_snprintf(send_message, LineLength, "GD%04d%04d%02d", state->first, state->last, 1);
	
	return urg_sendTag(send_message);
}

/*!
  \brief Get range data by using MD command

  \param state [i] Sensor information
  \param capture_times [i] capture times

  \retval 0 Success
  \retval < 0 Error
*/

static int urg_captureByMD(const urg_state_t* state, int capture_times)
{
	// 100 回を超えるデータ取得に対しては、回数に 00 (無限回取得)を指定し、
	// QT or RS コマンドでデータ取得を停止すること
	if (capture_times >= 100) 
	{
		capture_times = 0;
	}
	
	char send_message[LineLength];
	_snprintf(send_message, LineLength, "MD%04d%04d%02d%01d%02d", state->first, state->last, 1, 0, capture_times);
	
	return urg_sendTag(send_message);
}

// Decode 6bit data
static long urg_decode(const char data[], int data_byte)
{
	long value = 0;
	for (int i = 0; i < data_byte; ++i) 
	{
		value <<= 6;
		value &= ~0x3f;
		value |= data[i] - 0x30;
	}
	
	return value;
}

// Receive range data
static int urg_addRecvData(const char buffer[], long data[], int* filled)
{
	static int remain_byte = 0;
	static char remain_data[3];
	const int data_byte = 3;
	
	const char* pre_p = buffer;
	const char* p = pre_p;
	
	if (*filled <= 0) 
	{
		remain_byte = 0;
	}
	
	if (remain_byte > 0) 
	{
		memmove(&remain_data[remain_byte], buffer, data_byte - remain_byte);
		data[*filled] = urg_decode(remain_data, data_byte);
		++(*filled);
		pre_p = &buffer[data_byte - remain_byte];
		p = pre_p;
		remain_byte = 0;
	}
	
	do 
	{
		++p;
		if ((p - pre_p) >= static_cast<int>(data_byte)) 
		{
			data[*filled] = urg_decode(pre_p, data_byte);
			++(*filled);
			pre_p = p;
		}
	} while (*p != '\0');
	remain_byte = (int)(p - pre_p);
	memmove(remain_data, pre_p, remain_byte);
	
	return 0;
}

static int checkSum(char buffer[], int size, char actual_sum)
{
	char expected_sum = 0x00;
	int i;
	
	for (i = 0; i < size; ++i) 
	{
		expected_sum += buffer[i];
	}

	expected_sum = (expected_sum & 0x3f) + 0x30;
	
	return (expected_sum == actual_sum) ? 0 : -1;
}

/*!
  \brief Receive URG data

  測定データを配列に格納し、格納データ数を戻り値で返す。

  \param state [i] Sensor information
  \param data [o] range data
  \param max_size [i] range data buffer size

  \retval >= 0 number of range data
  \retval < 0 Error
*/

static int urg_receiveData(urg_state_t* state, long data[], size_t max_size)
{
	int filled = 0;
	
	// fill -1 from 0 to first
	for (int i = state->first -1; i >= 0; --i) 
	{
		data[filled++] = -1;
	}
	
	char message_type = 'M';
	char buffer[LineLength];
	int line_length;
	for (int line_count = 0; (line_length = urg_readLine(buffer)) >= 0; ++line_count) 
	{		
		// check sum
		if ((line_count > 3) && (line_length >= 3)) 
		{
			if (checkSum(buffer, line_length - 1, buffer[line_length - 1]) < 0) 
			{
				fprintf(stderr, "line_count: %d: %s\n", line_count, buffer);
				return -1;
			}
		}
		
		if ((line_count >= 6) && (line_length == 0)) 
		{
			// データ受信の完了
			for (size_t i = filled; i < max_size; ++i) 
			{
				// fill -1 to last of data buffer
				data[filled++] = -1;
			}
			return filled;
		} 
		else if (line_count == 0) 
		{
			// 送信メッセージの最初の文字でメッセージの判定を行う
			if ((buffer[0] != 'M') && (buffer[0] != 'G'))
			{
				return -1;
			}
			message_type = buffer[0];
		} 
		else if (! strncmp(buffer, "99b", 3))
		{
			// "99b" を検出し、以降を「タイムスタンプ」「データ」とみなす
			line_count = 4;
		} 
		else if ((line_count == 1) && (message_type == 'G')) 
		{
			line_count = 4;
		} 
		else if (line_count == 4) 
		{
			// "99b" 固定
			if (strncmp(buffer, "99b", 3)) 
			{
				return -1;
			}
		} 
		else if (line_count == 5) 
		{
			state->last_timestamp = urg_decode(buffer, 4);
		} 
		else if (line_count >= 6) 
		{
			// 取得データ
			if (line_length > (64 + 1)) 
			{
				line_length = (64 + 1);
			}
			buffer[line_length -1] = '\0';
			int ret = urg_addRecvData(buffer, data, &filled);
			if (ret < 0) 
			{
				return ret;
			}
		}
	}
	return -1;
}

void outputData(long data[], int n, size_t total_index)
{
	char output_file[] = "data_xxxxxxxxxx.csv";
	_snprintf(output_file, sizeof(output_file), "data_%03d.csv", total_index);
	FILE* fd = fopen(output_file, "w");
	if (! fd) 
	{
		perror("fopen");
		return;
	}
	
	for (int i = 0; i < n; ++i) 
	{
		fprintf(fd, "%ld, ", data[i]);
	}
	fprintf(fd, "\n");
	
	fclose(fd);
}

//thread/////////////////////////////////////
void ErrorExit(LPSTR lpszMessage)   
{   
    fprintf(stderr, "%s\n", lpszMessage);   
    ExitProcess(0);   
}



void process_data_one_pair()
{
	float data1[721];
	float data2[721];
	
	ifstream fi;

	clock_t start, end;

	char buf[100];
	string name1 = "D:\\Matlab Project\\useful data\\SceneData\\convergence\\data";
	string name3 = ".txt";

	int num1 = 23;
	int num2 = 24;
	
	string name2_1 = itoa( num1, buf, 10 );

	fi.open( name1 + name2_1 + name3, ios::app );
	if( !fi.is_open() )
	{
		cout<<"open file failed!"<<endl;
		//continue;
	}
	for( int j = 0; j < 721; ++j )
	{
		fi>>data1[j];
	}
	fi.close();

	name2_1 = itoa( num2, buf, 10 );

	fi.open( name1 + name2_1 + name3, ios::app );
	if( !fi.is_open() )
	{
		cout<<"open file failed!"<<endl;
		//continue;
	}
	for( int j = 0; j < 721; ++j )
	{
		fi>>data2[j];
	}
	fi.close();

	for( int i = 0; i < 721; ++i )
	{
		laserNew[i].r = data2[i] / 100;
		laserNew[i].t = i * 0.25 / 180 * M_PI;

		laserRef[i].r = data1[i] / 100;
		laserRef[i].t = i * 0.25 / 180 * M_PI;
	}
		
	////////////////////////////////////////////////////
	int numIteration = 0;
	start = clock();
	m_mbicp.MbICPmatcher(laserRef,laserNew,robot, numIteration, robot1,cov1);
	end = clock();
	cout<<"matching time: "<<(double)(end - start)/PER_SEC<<"S"<<endl;
	xx=robot1.getX();
	yy=robot1.getY();
	tt=robot1.getTh();
	cout<<"IterationTime = "<<numIteration<<endl;
	cout<<"x = "<<xx<<" y = "<<yy<<" tt = "<<tt<<endl;

	ofstream f1;
	f1.open("D:\\Matlab Project\\useful data\\SceneData\\convergence\\track.txt", ios::app);
	if( !f1.is_open() )
	{
		cout<<"open file failed!"<<endl;
		//continue;
	}
	f1<<xx * 100<<"  "<<yy * 100<<"  "<<tt<<"  "<<(double)(end - start)/PER_SEC<<"  "<<numIteration<<endl;
	f1.close();
}


void process_data1()
{
	float data1[721];
	float data2[721];
	
	ifstream fi;

	clock_t start, end;

	char buf[100];
	string name1 = "D:\\Matlab Project\\useful data\\circleline_test\\";
	string name3 = ".txt";
	
	for( int i = 1; i < 115; i += 3)
	{
		string name2_1 = itoa( i, buf, 10 );

		fi.open( name1 + name2_1 + name3, ios::app );
		if( !fi.is_open() )
		{
			cout<<"open file failed!"<<endl;
			//continue;
		}
		for( int j = 0; j < 721; ++j )
		{
			fi>>data1[j];
		}
		fi.close();

		name2_1 = itoa( i + 3, buf, 10 );

		fi.open( name1 + name2_1 + name3, ios::app );
		if( !fi.is_open() )
		{
			cout<<"open file failed!"<<endl;
			//continue;
		}
		for( int j = 0; j < 721; ++j )
		{
			fi>>data2[j];
		}
		fi.close();

		for( int i = 0; i < 721; ++i )
		{
			laserNew[i].r = ((float)data2[i]) / 100;
			laserNew[i].t = i * 0.25 / 180 * M_PI;

			laserRef[i].r = ((float)data1[i]) / 100;
			laserRef[i].t = i * 0.25 / 180 * M_PI;
		}
		
		////////////////////////////////////////////////////
		int numIteration = 0;
		start = clock();
		m_mbicp.MbICPmatcher(laserRef,laserNew,robot, numIteration, robot1,cov1);
		end = clock();
		cout<<"matching time: "<<(double)(end - start)/PER_SEC<<"S"<<endl;
		xx=robot1.getX();
		yy=robot1.getY();
		tt=robot1.getTh();
		cout<<"IterationTime = "<<numIteration<<endl;
		cout<<"x = "<<xx<<" y = "<<yy<<" tt = "<<tt<<endl;

		ofstream f1;
		f1.open("D:\\Matlab Project\\useful data\\circleline_test1\\track.txt", ios::app);
		if( !f1.is_open() )
		{
			cout<<"open file failed!"<<endl;
			//continue;
		}
		f1<<xx * 100<<"  "<<yy * 100<<"  "<<tt<<"  "<<(double)(end - start)/PER_SEC<<"  "<<numIteration<<endl;
		f1.close();
	}
}



void process_data()
{
	long data1[721];
	long data2[721];
	
	ifstream fi;

	clock_t start, end;

	char buf[100];
	string name1 = "D:\\Matlab Project\\useful data\\circleline_test3\\";
	string name3 = ".txt";
	
	for( int i = 1; i < 115; i += 3)
	{
		string name2_1 = itoa( i, buf, 10 );

		fi.open( name1 + name2_1 + name3, ios::app );
		if( !fi.is_open() )
		{
			cout<<"open file failed!"<<endl;
			//continue;
		}
		for( int j = 0; j < 721; ++j )
		{
			fi>>data1[j];
		}
		fi.close();

		name2_1 = itoa( i + 3, buf, 10 );

		fi.open( name1 + name2_1 + name3, ios::app );
		if( !fi.is_open() )
		{
			cout<<"open file failed!"<<endl;
			//continue;
		}
		for( int j = 0; j < 721; ++j )
		{
			fi>>data2[j];
		}
		fi.close();

		for( int i = 0; i < 721; ++i )
		{
			laserNew[i].r = ((float)data2[i]) / 1000;
			laserNew[i].t = i * 0.25 / 180 * M_PI;

			laserRef[i].r = ((float)data1[i]) / 1000;
			laserRef[i].t = i * 0.25 / 180 * M_PI;
		}
		
		////////////////////////////////////////////////////
		int numIteration = 0;
		start = clock();
		m_mbicp.MbICPmatcher(laserRef,laserNew,robot, numIteration, robot1,cov1);
		end = clock();
		cout<<"matching time: "<<(double)(end - start)/PER_SEC<<"S"<<endl;
		xx=robot1.getX();
		yy=robot1.getY();
		tt=robot1.getTh();
		cout<<"IterationTime = "<<numIteration<<endl;
		cout<<"x = "<<xx<<" y = "<<yy<<" tt = "<<tt<<endl;

		ofstream f1;
		f1.open("D:\\Matlab Project\\useful data\\circleline_test1\\track.txt", ios::app);
		if( !f1.is_open() )
		{
			cout<<"open file failed!"<<endl;
			//continue;
		}
		f1<<xx * 100<<"  "<<yy * 100<<"  "<<tt<<"  "<<(double)(end - start)/PER_SEC<<"  "<<numIteration<<endl;
		f1.close();
	}
}



int _tmain(int argc, _TCHAR* argv[])
{
	const char com_port[] = "COM4";
	const long com_baudrate = 115200;
	// URG に接続
	urg_state_t urg_state;
	//int ret = urg_connect( &urg_state, com_port, com_baudrate );
	//if (ret < 0) 
	//{
	//	// エラーメッセージを出力して終了
	//	printf("urg_connect: %s\n", ErrorMessage);
	//	
	//	// 即座に終了しないための処理。不要ならば削除すること
	//	getchar();
	//	exit(1);
	//}

	int max_size = urg_state.max_size;
	//long* data = new long[max_size];
	long data[1081];
	enum { CaptureTimes = 5 };
	size_t total_index = 0;
	int numIteration = 0;
	//////////////////////////////////////////////////////////////////////
	//UTM-30LX GD model 
	/*printf("using GD command\n");
	int recv_n = 0;
	urg_sendMessage("BM", Timeout, &recv_n);
	printf("urg_state.area_min: %d  urg_state.area_max: %d\n", urg_state.area_min, urg_state.area_max);
	printf("urg_state.distance_min: %ld  urg_state.distance_max:%ld\n", urg_state.distance_min, urg_state.distance_max);
	printf("urg_state.area_total: %d\n", urg_state.area_total);
	printf("urg_state.scan_rpm: %d\n", urg_state.scan_rpm);
	printf("urg_state.max_size: %d\n", urg_state.max_size);
	printf("urg_state.first: %d  urg_state.last: %d\n", urg_state.first, urg_state.last);*/
	//cvWaitKey(0);
	urg_state.first = 0;
	urg_state.last = 1080;

	static bool bfirst = 1;

	long saveData[721];
	clock_t start, end;

	int num1 = 1;
	int num2 = 2;

	////////////test//////////////////
	//process_data();
	process_data1();
	//process_data_one_pair();
	////////////test//////////////////
	while(1)
	{
		/*urg_captureByGD(&urg_state);
		int n = urg_receiveData(&urg_state, data, max_size);*/

		for( int i = 0; i < 1081; ++i )
		{
			saveData[i] = data[i + 180];
		}
		
		if( bfirst )
		{
			for( int i = 0; i < 721; ++i )
			{
				laserNew[i].r = (float)saveData[i] / 1000;
				laserNew[i].t = i * 0.25 / 180 * M_PI;
			}
			
			char buf[100];
			ofstream f1;
			string name1 = "E:\\test\\";
			string name2_1 = itoa( num1, buf, 10 );
			string name3 = ".txt";
			f1.open(name1 + name2_1 + name3, ios::app);

			if( !f1.is_open() )
			{
				cout<<"open file failed!"<<endl;
				//continue;
			}
			for( int i = 0; i < 721; ++i )
			{
				f1<<saveData[i]<<endl;
			}

			f1.close();
			bfirst = 0;
		}
		else
		{
			for( int i = 0; i < 721; ++i )
			{
				laserRef[i].r = laserNew[i].r;
				laserRef[i].t = laserNew[i].t;

				laserNew[i].r = (float)saveData[i] / 1000;
				laserNew[i].t = i * 0.25 / 180 * M_PI;
			}

			char buf[100];
			ofstream f2;
			string name1 = "E:\\test\\";
			string name2_1 = itoa( num2, buf, 10 );
			string name3 = ".txt";
			f2.open(name1 + name2_1 + name3, ios::app);

			if( !f2.is_open() )
			{
				cout<<"open file failed!"<<endl;
				//continue;
			}
			for( int i = 0; i < 721; ++i )
			{
				f2<<saveData[i]<<endl;
			}

			f2.close();

			start = clock();
			m_mbicp.MbICPmatcher(laserRef,laserNew,robot, numIteration, robot1,cov1);
			end = clock();
			cout<<"matching time: "<<(double)(end - start)/PER_SEC<<"S"<<endl;
			xx=robot1.getX();
			yy=robot1.getY();
			tt=robot1.getTh();
			cout<<"x = "<<xx<<" y = "<<yy<<" tt = "<<tt<<endl;
			end = 0;
		}
	}

	return 0;
}