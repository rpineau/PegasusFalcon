//
//  falcon.h
//  Created by Rodolphe Pineau on 2020/11/26.
//  Pegasus Falcon Rotator X2 plugin
//

#ifndef __PEGASUS_FALCON__
#define __PEGASUS_FALCON__
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

#include <math.h>
#include <string.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <exception>
#include <typeinfo>
#include <stdexcept>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

#define PLUGIN_DEBUG 3

#define DRIVER_VERSION 1.0
#define PLUGIN_ID   2

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 5000
#define LOG_BUFFER_SIZE 256

#define MAKE_ERR_CODE(P_ID, DTYPE, ERR_CODE)  (((P_ID<<24) & 0xff000000) | ((DTYPE<<16) & 0x00ff0000)  | (ERR_CODE & 0x0000ffff))


enum Falcon_Errors  {PLUGIN_OK = 0, NOT_CONNECTED, FALCON_CANT_CONNECT, FALCON_BAD_CMD_RESPONSE, COMMAND_FAILED};
enum RotorStatus    {IDLE = 0, MOVING};
enum RotorDir       {NORMAL = 0 , REVERSE};

typedef struct {
    bool    bReady;
    char    szVersion[SERIAL_BUFFER_SIZE];
    double  dCurPos;
    bool    bMoving;
    bool    bReverse;
} FalconStatus;

// field indexes in response for A command
#define fSTATUS     0
#define fSTEPPOS    1
#define fDEGPOS     2
#define fMOVING     3
#define fLIMIT      4
#define fDEROT      5
#define fREVERSE    6

class CFalconRotator
{
public:
    CFalconRotator();
    ~CFalconRotator();

    int         Connect(const char *pszPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return m_bIsConnected; };

    void        SetSerxPointer(SerXInterface *p) { m_pSerx = p; };

    // move commands
    int         haltFalcon();
    int         gotoPosition(double dPosDeg);

    // command complete functions
    int         isGoToComplete(bool &bComplete);
    int         isMotorMoving(bool &bMoving);

    // getter and setter
    void        setDebugLog(bool bEnable) {m_bDebugLog = bEnable; };

    int         getStatus(int &nStatus);
    int         getConsolidatedStatus(void);

    int         getFirmwareVersion(std::string &sVersion);
    int         getPosition(double &dPosition);
    int         syncMotorPosition(double dPos);

    int         setReverseEnable(bool bEnabled);
    int         getReverseEnable(bool &bEnabled);

protected:

    int             deviceCommand(const char *pszCmd, char *pszResult, int nResultMaxLen);
    int             deviceCommand(std::string sCmd, std::string &sResult);
    int             readResponse(char *pszRespBuffer, int nBufferLen);
    int             readResponse(std::string &RespBuffer);
    int             parseResp(std::string sIn, std::vector<std::string> &svFields, char cSeparator);

    SerXInterface   *m_pSerx;

    bool            m_bDebugLog;
    bool            m_bIsConnected;
    std::string     m_sFirmwareVersion;

    std::string&    trim(std::string &str, const std::string &filter );
    std::string&    ltrim(std::string &str, const std::string &filter);
    std::string&    rtrim(std::string &str, const std::string &filter);
    std::vector<std::string>    m_svParsedRespForFA;

    FalconStatus    m_globalStatus;
    double          m_dTargetPos;
    int             m_nPosLimit;
    bool            m_bPosLimitEnabled;
	bool			m_bAbborted;
	

#ifdef PLUGIN_DEBUG
	std::string m_sLogfilePath;
	// timestamp for logs
	char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif

};

#endif //__PEGASUS_FALCON__
