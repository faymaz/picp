
#ifndef __SERIAL_H_
#define __SERIAL_H_

#ifdef WIN32
#define	bool	int
#endif

bool ByteWaiting(int theDevice,unsigned int timeOut);
unsigned int ReadBytes(int theDevice,unsigned char *theBytes,unsigned int maxBytes,unsigned int timeOut);
void WriteBytes(int theDevice,unsigned char *theBytes,unsigned int numBytes);
void FlushBytes(int theDevice);
bool ConfigureDevice(int theDevice,unsigned int baudRate,unsigned char dataBits,unsigned char stopBits,unsigned char parity,bool cooked);
void GetDeviceConfiguration(int theDevice,unsigned int *baudRate,unsigned char *dataBits,unsigned char *stopBits,unsigned char *parity);
bool ConfigureFlowControl(int theDevice,bool wantControl);
void GetDeviceStatus(int theDevice,bool *CTS,bool *DCD);
void SetDTR(int theDevice,bool DTR);
bool OpenDevice(char *theName,int *theDevice);
void CloseDevice(int theDevice);

#endif // defined __SERIAL_H_
