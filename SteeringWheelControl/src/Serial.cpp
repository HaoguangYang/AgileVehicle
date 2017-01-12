<<<<<<< HEAD
#include "SerialClass.h"

#if defined (_MSC_VER)

Serial::Serial(char *portName)
{
    //We're not yet connected
    this->connected = false;

    //Try to connect to the given port throuh CreateFile
    this->hSerial = CreateFile(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

    //Check if the connection was successfull
    if(this->hSerial==INVALID_HANDLE_VALUE)
    {
        //If not success full display an Error
        if(GetLastError()==ERROR_FILE_NOT_FOUND){

            //Print Error if neccessary
            printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);

        }
        else
        {
            printf("ERROR!!!");
        }
    }
    else
    {
        //If connected we try to set the comm parameters
        DCB dcbSerialParams = {0};

        //Try to get the current
        if (!GetCommState(this->hSerial, &dcbSerialParams))
        {
            //If impossible, show an error
            printf("failed to get current serial parameters!");
        }
        else
        {
            //Define serial connection parameters for the arduino board
            dcbSerialParams.BaudRate=CBR_9600;
            dcbSerialParams.ByteSize=8;
            dcbSerialParams.StopBits=ONESTOPBIT;
            dcbSerialParams.Parity=NOPARITY;
            //Setting the DTR to Control_Enable ensures that the Arduino is properly
            //reset upon establishing a connection
            dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

             //Set the parameters and check for their proper application
             if(!SetCommState(this->hSerial, &dcbSerialParams))
             {
                printf("ALERT: Could not set Serial Port parameters");
             }
             else
             {
                 //If everything went fine we're connected
                 this->connected = true;
                 //Flush any remaining characters in the buffers 
                 PurgeComm(this->hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
                 //We wait 2s as the arduino board will be reseting
                 Sleep(ARDUINO_WAIT_TIME);
             }
        }
    }

}

Serial::~Serial()
{
    //Check if we are connected before trying to disconnect
    if(this->connected)
    {
        //We're no longer connected
        this->connected = false;
        //Close the serial handler
        CloseHandle(this->hSerial);
    }
}

int Serial::ReadData(char *buffer, unsigned int nbChar)
{
    //Number of bytes we'll have read
    DWORD bytesRead;
    //Number of bytes we'll really ask to read
    unsigned int toRead;

    //Use the ClearCommError function to get status info on the Serial port
    ClearCommError(this->hSerial, &this->errors, &this->status);

    //Check if there is something to read
    if(this->status.cbInQue>0)
    {
        //If there is we check if there is enough data to read the required number
        //of characters, if not we'll read only the available characters to prevent
        //locking of the application.
        if(this->status.cbInQue>nbChar)
        {
            toRead = nbChar;
        }
        else
        {
            toRead = this->status.cbInQue;
        }

        //Try to read the require number of chars, and return the number of read bytes on success
        if(ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL) )
        {
            return bytesRead;
        }

    }

    //If nothing has been read, or that an error was detected return 0
    return 0;

}


bool Serial::WriteData(char *buffer, unsigned int nbChar)
{
    DWORD bytesSend;

    //Try to write the buffer on the Serial port
    if(!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
    {
        //In case it don't work get comm error and return false
        ClearCommError(this->hSerial, &this->errors, &this->status);
        return false;
    }
    else
        return true;
}

#elif defined (__linux__)
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#define RS232_PORTNR  38
int error;
struct termios new_port_settings,
       old_port_settings[RS232_PORTNR];


Serial::Serial(char *portName)
{
	int status;
	this->connected = false;
	this->hSerial = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
	if(this->hSerial==-1)
    {
		perror("unable to open comport ");
    }
	if(flock(this->hSerial, LOCK_EX | LOCK_NB) != 0)
    {
		close(this->hSerial);
		perror("Another process has locked the comport.");
	}

	error = tcgetattr(this->hSerial, old_port_settings);
	if(error==-1)
	{
		close(this->hSerial);
		flock(this->hSerial, LOCK_UN);  /* free the port so that others can use it. */
		perror("unable to read portsettings ");
	}
	memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

	int baudr = B9600,
		cbits = CS8,
		bstop = CSTOPB,
		cpar = 0,
		ipar = IGNPAR;
	
	if(ioctl(this->hSerial, TIOCMGET, &status) == -1)
	{
		tcsetattr(this->hSerial, TCSANOW, old_port_settings);
		flock(this->hSerial, LOCK_UN);  /* free the port so that others can use it. */
		perror("unable to get portstatus");
	}
	
	new_port_settings.c_cflag = cbits | cpar | bstop | CLOCAL | CREAD;
	new_port_settings.c_iflag = ipar;
	new_port_settings.c_oflag = 0;
	new_port_settings.c_lflag = 0;
	new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
	new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */

	cfsetispeed(&new_port_settings, baudr);
	cfsetospeed(&new_port_settings, baudr);
	
	error = tcsetattr(this->hSerial, TCSANOW, &new_port_settings);
	if(error==-1)
	{
		printf("ALERT: Could not set Serial Port parameters");
	}
	else
	{
		//If everything went fine we're connected
        this->connected = true;
		//Flush any remaining characters in the buffers 
        tcflush(this->hSerial, TCOFLUSH);
        //We wait 2s as the arduino board will be reseting
        usleep(ARDUINO_WAIT_TIME*1000);
	}
}

Serial::~Serial()
{
    //Check if we are connected before trying to disconnect
    if(this->connected)
    {
        //We're no longer connected
        this->connected = false;
        //Close the serial handler
        tcsetattr(this->hSerial, TCSANOW, old_port_settings);
		close(this->hSerial);

		flock(this->hSerial, LOCK_UN);  /* free the port so that others can use it. */
    }
}

bool Serial::WriteData(char *buffer, unsigned int nbChar)
{
  int n = write(this->hSerial, buffer, nbChar);
  if(n < 0)
  {
    if(errno == EAGAIN)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  return true;
}

int Serial::ReadData(char *buffer, unsigned int nbChar)
{
    //Number of bytes we'll have read
    unsigned long bytesRead;
	
    //Try to read the require number of chars, and return the number of read bytes on success
	bytesRead = read(this->hSerial, buffer, nbChar);
    if(bytesRead>=0)
    {
		buffer[bytesRead]=0; /* always put a "null" at the end of a string! */
        return bytesRead;
    }

    //If nothing has been read, or that an error was detected return 0
    return 0;
}

#endif

bool Serial::IsConnected()
{
    //Simply return the connection status
    return this->connected;
}

=======
#include "SerialClass.h"

#if defined (_MSC_VER)

Serial::Serial(char *portName)
{
    //We're not yet connected
    this->connected = false;

    //Try to connect to the given port throuh CreateFile
    this->hSerial = CreateFile(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

    //Check if the connection was successfull
    if(this->hSerial==INVALID_HANDLE_VALUE)
    {
        //If not success full display an Error
        if(GetLastError()==ERROR_FILE_NOT_FOUND){

            //Print Error if neccessary
            printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);

        }
        else
        {
            printf("ERROR!!!");
        }
    }
    else
    {
        //If connected we try to set the comm parameters
        DCB dcbSerialParams = {0};

        //Try to get the current
        if (!GetCommState(this->hSerial, &dcbSerialParams))
        {
            //If impossible, show an error
            printf("failed to get current serial parameters!");
        }
        else
        {
            //Define serial connection parameters for the arduino board
            dcbSerialParams.BaudRate=CBR_9600;
            dcbSerialParams.ByteSize=8;
            dcbSerialParams.StopBits=ONESTOPBIT;
            dcbSerialParams.Parity=NOPARITY;
            //Setting the DTR to Control_Enable ensures that the Arduino is properly
            //reset upon establishing a connection
            dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

             //Set the parameters and check for their proper application
             if(!SetCommState(this->hSerial, &dcbSerialParams))
             {
                printf("ALERT: Could not set Serial Port parameters");
             }
             else
             {
                 //If everything went fine we're connected
                 this->connected = true;
                 //Flush any remaining characters in the buffers 
                 PurgeComm(this->hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
                 //We wait 2s as the arduino board will be reseting
                 Sleep(ARDUINO_WAIT_TIME);
             }
        }
    }

}

Serial::~Serial()
{
    //Check if we are connected before trying to disconnect
    if(this->connected)
    {
        //We're no longer connected
        this->connected = false;
        //Close the serial handler
        CloseHandle(this->hSerial);
    }
}

int Serial::ReadData(char *buffer, unsigned int nbChar)
{
    //Number of bytes we'll have read
    DWORD bytesRead;
    //Number of bytes we'll really ask to read
    unsigned int toRead;

    //Use the ClearCommError function to get status info on the Serial port
    ClearCommError(this->hSerial, &this->errors, &this->status);

    //Check if there is something to read
    if(this->status.cbInQue>0)
    {
        //If there is we check if there is enough data to read the required number
        //of characters, if not we'll read only the available characters to prevent
        //locking of the application.
        if(this->status.cbInQue>nbChar)
        {
            toRead = nbChar;
        }
        else
        {
            toRead = this->status.cbInQue;
        }

        //Try to read the require number of chars, and return the number of read bytes on success
        if(ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL) )
        {
            return bytesRead;
        }

    }

    //If nothing has been read, or that an error was detected return 0
    return 0;

}


bool Serial::WriteData(char *buffer, unsigned int nbChar)
{
    DWORD bytesSend;

    //Try to write the buffer on the Serial port
    if(!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
    {
        //In case it don't work get comm error and return false
        ClearCommError(this->hSerial, &this->errors, &this->status);
        return false;
    }
    else
        return true;
}

#elif defined (__linux__)
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#define RS232_PORTNR  38
int error;
struct termios new_port_settings,
       old_port_settings[RS232_PORTNR];


Serial::Serial(char *portName)
{
	int status;
	this->connected = false;
	this->hSerial = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
	if(this->hSerial==-1)
    {
		perror("unable to open comport ");
    }
	if(flock(this->hSerial, LOCK_EX | LOCK_NB) != 0)
    {
		close(this->hSerial);
		perror("Another process has locked the comport.");
	}

	error = tcgetattr(this->hSerial, old_port_settings);
	if(error==-1)
	{
		close(this->hSerial);
		flock(this->hSerial, LOCK_UN);  /* free the port so that others can use it. */
		perror("unable to read portsettings ");
	}
	memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

	int baudr = B9600,
		cbits = CS8,
		bstop = CSTOPB,
		cpar = 0,
		ipar = IGNPAR;
	
	if(ioctl(this->hSerial, TIOCMGET, &status) == -1)
	{
		tcsetattr(this->hSerial, TCSANOW, old_port_settings);
		flock(this->hSerial, LOCK_UN);  /* free the port so that others can use it. */
		perror("unable to get portstatus");
	}
	
	new_port_settings.c_cflag = cbits | cpar | bstop | CLOCAL | CREAD;
	new_port_settings.c_iflag = ipar;
	new_port_settings.c_oflag = 0;
	new_port_settings.c_lflag = 0;
	new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
	new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */

	cfsetispeed(&new_port_settings, baudr);
	cfsetospeed(&new_port_settings, baudr);
	
	error = tcsetattr(this->hSerial, TCSANOW, &new_port_settings);
	if(error==-1)
	{
		printf("ALERT: Could not set Serial Port parameters");
	}
	else
	{
		//If everything went fine we're connected
        this->connected = true;
		//Flush any remaining characters in the buffers 
        tcflush(this->hSerial, TCOFLUSH);
        //We wait 2s as the arduino board will be reseting
        usleep(ARDUINO_WAIT_TIME*1000);
	}
}

Serial::~Serial()
{
    //Check if we are connected before trying to disconnect
    if(this->connected)
    {
        //We're no longer connected
        this->connected = false;
        //Close the serial handler
        tcsetattr(this->hSerial, TCSANOW, old_port_settings);
		close(this->hSerial);

		flock(this->hSerial, LOCK_UN);  /* free the port so that others can use it. */
    }
}

bool Serial::WriteData(char *buffer, unsigned int nbChar)
{
  int n = write(this->hSerial, buffer, nbChar);
  if(n < 0)
  {
    if(errno == EAGAIN)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  return true;
}

int Serial::ReadData(char *buffer, unsigned int nbChar)
{
    //Number of bytes we'll have read
    unsigned long bytesRead;
	
    //Try to read the require number of chars, and return the number of read bytes on success
	bytesRead = read(this->hSerial, buffer, nbChar);
    if(bytesRead>=0)
    {
		buffer[bytesRead]=0; /* always put a "null" at the end of a string! */
        return bytesRead;
    }

    //If nothing has been read, or that an error was detected return 0
    return 0;
}

#endif

bool Serial::IsConnected()
{
    //Simply return the connection status
    return this->connected;
}

>>>>>>> f22ec6c22c6fa5e43b341e05886f52a49472abd7
