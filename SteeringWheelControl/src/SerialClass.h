<<<<<<< HEAD
#define ARDUINO_WAIT_TIME 2000
#ifndef SERIALCLASS_H_INCLUDED
#define SERIALCLASS_H_INCLUDED
#include <stdio.h>
#include <stdlib.h>
#if defined (_MSC_VER)
#include <windows.h>
#elif defined(__linux__)
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>
#include <errno.h>
#include "Migration.h"
#endif

class Serial
{
    private:
        //Serial comm handler
        HANDLE hSerial;
        //Connection status
        bool connected;
#ifdef _MSC_VER
        //Get various information about the connection
        COMSTAT status;
#endif
        //Keep track of last error
        DWORD errors;

    public:
        //Initialize Serial communication with the given COM port
        Serial(char *portName);
        //Close the connection
        ~Serial();
        //Read data in a buffer, if nbChar is greater than the
        //maximum number of bytes available, it will return only the
        //bytes available. The function return -1 when nothing could
        //be read, the number of bytes actually read.
        int ReadData(char *buffer, unsigned int nbChar);
        //Writes data from a buffer through the Serial connection
        //return true on success.
        bool WriteData(char *buffer, unsigned int nbChar);
        //Check if we are actually connected
        bool IsConnected();


};

#endif
=======
#define ARDUINO_WAIT_TIME 2000
#ifndef SERIALCLASS_H_INCLUDED
#define SERIALCLASS_H_INCLUDED
#include <stdio.h>
#include <stdlib.h>
#if defined (_MSC_VER)
#include <windows.h>
#elif defined(__linux__)
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>
#include <errno.h>
#include "Migration.h"
#endif

class Serial
{
    private:
        //Serial comm handler
        HANDLE hSerial;
        //Connection status
        bool connected;
#ifdef _MSC_VER
        //Get various information about the connection
        COMSTAT status;
#endif
        //Keep track of last error
        DWORD errors;

    public:
        //Initialize Serial communication with the given COM port
        Serial(char *portName);
        //Close the connection
        ~Serial();
        //Read data in a buffer, if nbChar is greater than the
        //maximum number of bytes available, it will return only the
        //bytes available. The function return -1 when nothing could
        //be read, the number of bytes actually read.
        int ReadData(char *buffer, unsigned int nbChar);
        //Writes data from a buffer through the Serial connection
        //return true on success.
        bool WriteData(char *buffer, unsigned int nbChar);
        //Check if we are actually connected
        bool IsConnected();


};

#endif
>>>>>>> f22ec6c22c6fa5e43b341e05886f52a49472abd7
