
#ifndef __SERIAL_H_
#define __SERIAL_H_

#ifdef WIN32
#define	bool	int
#endif

bool ByteWaiting(int theDevice, int timeOut);
int ReadBytes(int theDevice, byte *theBytes, int maxBytes, int timeOut);
void WriteBytes(int theDevice, byte *theBytes, int numBytes);
void FlushBytes(int theDevice);
bool ConfigureDevice(int theDevice, int baudRate, byte dataBits, byte stopBits, byte parity,bool cooked);
void GetDeviceConfiguration(int theDevice, int *baudRate, byte *dataBits, byte *stopBits, byte *parity);
bool ConfigureFlowControl(int theDevice, bool wantControl);
void GetDeviceStatus(int theDevice, bool *CTS, bool *DCD);
void SetDTR(int theDevice, bool DTR);
bool OpenDevice(char *theName, int *theDevice);
void CloseDevice(int theDevice);

#endif // defined __SERIAL_H_
