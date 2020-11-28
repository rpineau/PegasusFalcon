//
//  falcon.cpp
//  Created by Rodolphe Pineau on 2020/11/26.
//  Pegasus Falcon Rotator X2 plugin
//


#include "falcon.h"


CFalconRotator::CFalconRotator()
{
    m_globalStatus.bReady = false;
    memset(m_globalStatus.szVersion,0,SERIAL_BUFFER_SIZE);

    m_dTargetPos = 0;
    m_bAbborted = false;
    
    m_pSerx = NULL;


#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
	m_sLogfilePath = getenv("HOMEDRIVE");
	m_sLogfilePath += getenv("HOMEPATH");
	m_sLogfilePath += "\\PegasusFalconLog.txt";
#elif defined(SB_LINUX_BUILD)
	m_sLogfilePath = getenv("HOME");
	m_sLogfilePath += "/PegasusFalconLog.txt";
#elif defined(SB_MAC_BUILD)
	m_sLogfilePath = getenv("HOME");
	m_sLogfilePath += "/PegasusFalconLog.txt";
#endif
	Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#ifdef	PLUGIN_DEBUG
	ltime = time(NULL);
	char *timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CFalconRotator Constructor Called.\n", timestamp);
	fflush(Logfile);
#endif

}

CFalconRotator::~CFalconRotator()
{
#if defined PLUGIN_DEBUG
	// Close LogFile
	if (Logfile) fclose(Logfile);
#endif
}

int CFalconRotator::Connect(const char *pszPort)
{
    int nErr = PLUGIN_OK;

    if(!m_pSerx)
        return ERR_COMMNOLINK;

#if defined PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CFalconRotator::Connect Called %s\n", timestamp, pszPort);
	fflush(Logfile);
#endif

    // 19200 8N1
    nErr = m_pSerx->open(pszPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1");
    if(nErr == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return nErr;

#if defined PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CFalconRotator::Connect] connected to %s\n", timestamp, pszPort);
    fprintf(Logfile, "[%s] [CFalconRotator::Connect] Getting Firmware\n", timestamp);
	fflush(Logfile);
#endif
	
    // get status so we can figure out what device we are connecting to.
#if defined PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CFalconRotator::Connect getting device type\n", timestamp);
	fflush(Logfile);
#endif


    nErr = getFirmwareVersion(m_sFirmwareVersion);
    return nErr;
}

void CFalconRotator::Disconnect()
{
    if(m_bIsConnected && m_pSerx)
        m_pSerx->close();
 
	m_bIsConnected = false;
}

#pragma mark move commands
int CFalconRotator::haltFalcon()
{
    int nErr;
    std::string sResp;
    
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    nErr = deviceCommand("FH\n", sResp);
	m_bAbborted = true;
	
	return nErr;
}

int CFalconRotator::gotoPosition(double dPosDeg)
{
    int nErr;
    std::string sResp;
    std::string sCmd;
    std::stringstream ss;
    
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    while(dPosDeg > 359.99) {
        dPosDeg -= 360;
    }

    while(dPosDeg <0) {
        dPosDeg += 360;
    }

    ss << std::fixed << std::setprecision(2) << dPosDeg;
    sCmd = "MD:" + ss.str() + "\n";
    
#if defined PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CFalconRotator::gotoPosition moving to %3.2f\n", timestamp, dPosDeg);
    fflush(Logfile);
#endif

    nErr = deviceCommand(sCmd, sResp);
    m_dTargetPos = dPosDeg;

    return nErr;
}

#pragma mark command complete functions

int CFalconRotator::isGoToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    bool bIsMoving;
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;
    
    bComplete = false;
    // check if we're still moving
    isMotorMoving(bIsMoving);
    if(bIsMoving)
        return nErr;
    
    getPosition(m_globalStatus.dCurPos);
#if defined PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CFalconRotator::isGoToComplete m_globalStatus.nCurPos = %3.2f steps\n", timestamp, m_globalStatus.dCurPos);
    fprintf(Logfile, "[%s] CFalconRotator::isGoToComplete m_nTargetPos = %3.2f steps\n", timestamp, m_dTargetPos);
    fflush(Logfile);
#endif
    if(m_bAbborted) {
		bComplete = true;
		m_dTargetPos = m_globalStatus.dCurPos;
		m_bAbborted = false;
	}
    
    
    else if ((m_dTargetPos <= m_globalStatus.dCurPos + 0.1) && (m_dTargetPos >= m_globalStatus.dCurPos - 0.1))
        bComplete = true;
    else
        bComplete = false;
    return nErr;
}

int CFalconRotator::isMotorMoving(bool &bMoving)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svParsedResp;
    
    if(!m_bIsConnected)
		return ERR_COMMNOLINK;
	

    // OK_SMFC or OK_DMFC
    nErr = deviceCommand("FR\n", sResp);
    if(nErr)
        return nErr;

    nErr = parseResp(sResp, svParsedResp, ':');
    if(nErr)
        return nErr;
    if(svParsedResp.size()<2)
        return FALCON_BAD_CMD_RESPONSE;

    if(svParsedResp[1] == "1") {
        bMoving = true;
        m_globalStatus.bMoving = MOVING;
    }
    else {
        bMoving = false;
        m_globalStatus.bMoving = IDLE;
    }

    return nErr;
}

#pragma mark getters and setters
int CFalconRotator::getStatus(int &nStatus)
{
    int nErr;
    std::string sResp;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    // OK_SMFC or OK_DMFC
    nErr = deviceCommand("#\n", sResp);
    if(nErr)
        return nErr;

    if(sResp.find("OK_") != -1) {
        nStatus = PLUGIN_OK;
        nErr = PLUGIN_OK;
    }
    else {
        nErr = COMMAND_FAILED;
    }
    return nErr;
}

int CFalconRotator::getConsolidatedStatus()
{
    int nErr;
    std::string sResp;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    nErr = deviceCommand("FA\n", sResp);
    if(nErr)
        return nErr;

#if defined PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CFalconRotator::getConsolidatedStatus about to parse response\n", timestamp);
	fflush(Logfile);
#endif

    // parse response
    nErr = parseResp(sResp, m_svParsedRespForFA, ':');
    if(m_svParsedRespForFA.size()<7) {
    #if defined PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CFalconRotator::getConsolidatedStatus response error.. not enought fields\n", timestamp);
        fflush(Logfile);
    #endif
        return FALCON_BAD_CMD_RESPONSE;
    }
#if defined PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CFalconRotator::getConsolidatedStatus response parsing done\n", timestamp);
	fflush(Logfile);
#endif
    if(m_svParsedRespForFA.empty()) {
#if defined PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CFalconRotator::getConsolidatedStatus parsing returned an empty vector\n", timestamp);
        fflush(Logfile);
#endif
        return FALCON_BAD_CMD_RESPONSE;
    }

#if defined PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CFalconRotator::getConsolidatedStatus Status = %s\n", timestamp, m_svParsedRespForFA[fSTATUS].c_str());
	fflush(Logfile);
#endif

	if(m_svParsedRespForFA[fSTATUS].find("OK_")!= -1) {
        m_globalStatus.bReady = true;
    }
    else {
        m_globalStatus.bReady = false;
    }
    m_globalStatus.dCurPos = std::stod(m_svParsedRespForFA[fDEGPOS]);
    m_globalStatus.bMoving = (m_svParsedRespForFA[fMOVING] == "1");
    m_globalStatus.bReverse = (m_svParsedRespForFA[fREVERSE] == "1");
#if defined PLUGIN_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	
	fprintf(Logfile, "[%s] CFalconRotator::getConsolidatedStatus nCurPos        : %3.2f\n", timestamp, m_globalStatus.dCurPos);
    fprintf(Logfile, "[%s] CFalconRotator::getConsolidatedStatus bMoving        : %s\n", timestamp, m_globalStatus.bMoving?"Yes":"No");
    fprintf(Logfile, "[%s] CFalconRotator::getConsolidatedStatus bReverse       : %s\n", timestamp, m_globalStatus.bReverse?"Yes":"No");
	fflush(Logfile);
#endif

	
    return nErr;
}

int CFalconRotator::getFirmwareVersion(std::string &sVersion)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svParsedResp;
    
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = deviceCommand("FV\n", sResp);
    if(nErr)
        return nErr;

    nErr = parseResp(sResp, svParsedResp, ':');
    if(nErr)
        return nErr;

    if(svParsedResp.size()<2)
        return FALCON_BAD_CMD_RESPONSE;
    
    sVersion.assign(svParsedResp[1]);
    return nErr;
}


int CFalconRotator::getPosition(double &dPosition)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> svParsedResp;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    nErr = deviceCommand("FD\n", sResp);
    if(nErr)
        return nErr;

    nErr = parseResp(sResp, svParsedResp, ':');
    if(nErr)
        return nErr;

    if(svParsedResp.size()<2)
        return FALCON_BAD_CMD_RESPONSE;

    // convert response
    dPosition = std::stod(svParsedResp[1]);

    return nErr;
}


int CFalconRotator::syncMotorPosition(double dPosDeg)
{
    int nErr = PLUGIN_OK;

    std::string sResp;
    std::string sCmd;
    std::stringstream ss;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    ss << std::fixed << std::setprecision(2) << dPosDeg;
    sCmd = "SD:" + ss.str() + "\n";

    nErr = deviceCommand(sCmd, sResp);
    nErr |= getConsolidatedStatus();
    return nErr;
}

int CFalconRotator::setReverseEnable(bool bEnabled)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::string sCmd;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    sCmd = "FN:" + (bEnabled ? std::string("1") : std::string("0")) + "\n";
#if defined PLUGIN_DEBUGG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CFalconRotator::setReverseEnable setting reverse : %s\n", timestamp, sCmd.c_str());
    fflush(Logfile);
#endif

    nErr = deviceCommand(sCmd, sResp);

#if defined PLUGIN_DEBUG
    if(nErr) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CFalconRotator::setReverseEnable **** ERROR **** setting reverse (\"%s\") : %d\n", timestamp, sCmd.c_str(), nErr);
        fflush(Logfile);
    }
#endif

    return nErr;
}

int CFalconRotator::getReverseEnable(bool &bEnabled)
{
    int nErr;
	
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    nErr = getConsolidatedStatus();
    bEnabled = m_globalStatus.bReverse;

    return nErr;
}


#pragma mark command and response functions

int CFalconRotator::deviceCommand(std::string sCmd, std::string &sResult)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    unsigned long  ulBytesWrite;
    
    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    m_pSerx->purgeTxRx();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CFalconRotator::deviceCommand Sending %s\n", timestamp, sCmd.c_str());
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)sCmd.c_str(), sCmd.size(), ulBytesWrite);
    m_pSerx->flushTx();

    // printf("Command %s sent. wrote %lu bytes\n", szCmd, ulBytesWrite);
    if(nErr){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CFalconRotator::deviceCommand] writeFile Error.\n", timestamp);
        fflush(Logfile);
#endif
        return nErr;
    }

    nErr = readResponse(sResp);
    if(nErr)
        return nErr;

    sResult.assign(sResp);
    return nErr;
}


int CFalconRotator::readResponse(std::string &RespBuffer)
{
    int nErr = PLUGIN_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char pszRespBuffer[SERIAL_BUFFER_SIZE];
    char *pszBufPtr;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(pszRespBuffer, 0, (size_t) SERIAL_BUFFER_SIZE);
    pszBufPtr = pszRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CFalconRotator::readResponse] readFile Error.\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CFalconRotator::readResponse] timeout.\n", timestamp);
            fflush(Logfile);
#endif
            nErr = ERR_NORESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
    } while (*pszBufPtr++ != '\n' && ulTotalBytesRead < SERIAL_BUFFER_SIZE );

    RespBuffer.assign(pszRespBuffer);
    RespBuffer.assign(trim(RespBuffer, "\r\n"));

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CFalconRotator::readResponse] response = %s\n", timestamp, RespBuffer.c_str());
            fflush(Logfile);
#endif

    return nErr;
}


int CFalconRotator::parseResp(std::string sIn, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = PLUGIN_OK;
    std::string sSegment;
    std::stringstream ssTmp(sIn);
    
    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }
    
    if(svFields.size()==0) {
        nErr = ERR_BADFORMAT;
    }
    return nErr;
}

std::string& CFalconRotator::trim(std::string &str, const std::string& filter )
{
    return ltrim(rtrim(str, filter), filter);
}

std::string& CFalconRotator::ltrim(std::string& str, const std::string& filter)
{
    str.erase(0, str.find_first_not_of(filter));
    return str;
}

std::string& CFalconRotator::rtrim(std::string& str, const std::string& filter)
{
    str.erase(str.find_last_not_of(filter) + 1);
    return str;
}

