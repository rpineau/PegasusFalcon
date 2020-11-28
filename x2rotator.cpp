
#include "x2rotator.h"


X2Rotator::X2Rotator(const char* pszDriverSelection,
						const int& nInstanceIndex,
						SerXInterface					* pSerX, 
						TheSkyXFacadeForDriversInterface	* pTheSkyX, 
						SleeperInterface					* pSleeper,
						BasicIniUtilInterface			* pIniUtil,
						LoggerInterface					* pLogger,
						MutexInterface					* pIOMutex,
						TickCountInterface				* pTickCount)
{
	m_nInstanceIndex				= nInstanceIndex;
	m_pSerX							= pSerX;		
	m_pTheSkyXForMounts				= pTheSkyX;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;	
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_nInstanceIndex = nInstanceIndex;
	m_bLinked = false;
	m_dPosition = 0;
	m_bDoingGoto = false;
	m_dTargetPosition = 0;
	m_nGotoStartStamp = 0;
    
    mRotator.SetSerxPointer(m_pSerX);
}

X2Rotator::~X2Rotator()
{
	if (GetSerX())
		delete GetSerX();
	if (GetTheSkyXFacadeForDrivers())
		delete GetTheSkyXFacadeForDrivers();
	if (GetSleeper())
		delete GetSleeper();
	if (GetSimpleIniUtil())
		delete GetSimpleIniUtil();
	if (GetLogger())
		delete GetLogger();
	if (GetMutex())
		delete GetMutex();
	if (GetTickCountInterface())
		delete GetTickCountInterface();

}

int	X2Rotator::queryAbstraction(const char* pszName, void** ppVal) 
{
	*ppVal = NULL;

	if (!strcmp(pszName, SerialPortParams2Interface_Name))
		*ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);
	else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
	
	return SB_OK;
}


int X2Rotator::execModalSettingsDialog()
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*                    ui = uiutil.X2UI();
    X2GUIExchangeInterface*            dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    bool bReversed = false;
    bool bNewReversed = false;
    double dPos;
    
    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("PegasusFalcon.ui", deviceType(), m_nInstanceIndex)))
        return nErr;


    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());
    if(m_bLinked) {
        dx->setEnabled("pushButton", true);
        dx->setEnabled("pushButton_2", true);
        dx->setEnabled("doubleSpinBox", true);
        nErr = mRotator.getPosition(dPos);
        if(!nErr)
            dx->setPropertyDouble("doubleSpinBox", "value", dPos);
        dx->setEnabled("reverseDir", true);
        nErr = mRotator.getReverseEnable(bReversed);
        if(!nErr)
            dx->setChecked("reverseDir", bReversed);
    }
    else {
        dx->setEnabled("pushButton", false);
        dx->setEnabled("pushButton_2", false);
        dx->setEnabled("doubleSpinBox", false);
        dx->setEnabled("reverseDir", false);
    }
    //Display the user interface
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK) {
        if(m_bLinked) {
            bNewReversed = dx->isChecked("reverseDir");
            if(bNewReversed != bReversed)
                mRotator.setReverseEnable(bNewReversed); // no need to set it if it didn't change
        }
    }

    return nErr;
}


void X2Rotator::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    double dNewPos;
    int nErr = SB_OK;
    std::string sErrMsg;
    
    if (!strcmp(pszEvent, "on_pushButton_2_clicked")) {
        uiex->propertyDouble("doubleSpinBox", "value", dNewPos);
        nErr = mRotator.syncMotorPosition(dNewPos);
        if(nErr) {
            sErrMsg = "Sync to new position failed : Error = " + std::to_string(nErr);
            uiex->messageBox("IFW Homing", sErrMsg.c_str());
        }
    }
}


int	X2Rotator::establishLink(void)						
{
    int nErr = SB_OK;
    char szPort[SERIAL_BUFFER_SIZE];
    
    // get serial port device name
    portNameOnToCharPtr(szPort,SERIAL_BUFFER_SIZE);
    nErr = mRotator.Connect(szPort);
    if(nErr) {
        m_bLinked = false;
    }
    else
        m_bLinked = true;

	return nErr;
}
int	X2Rotator::terminateLink(void)						
{
    int nErr = SB_OK;
    
    mRotator.Disconnect();
    
    m_bLinked = false;
	return nErr;
}
bool X2Rotator::isLinked(void) const					
{
	return m_bLinked;
}

void X2Rotator::deviceInfoNameShort(BasicStringInterface& str) const				
{
	str = "Falcon Rotator";
}
void X2Rotator::deviceInfoNameLong(BasicStringInterface& str) const				
{
	str = "Pegasus Falcon Rotator";
}
void X2Rotator::deviceInfoDetailedDescription(BasicStringInterface& str) const		
{
	str = "Pegasus Falcon Rotator";
}
void X2Rotator::deviceInfoFirmwareVersion(BasicStringInterface& str)				
{
    if(m_bLinked) {
        std::string sFirmware;
        X2MutexLocker ml(GetMutex());
        mRotator.getFirmwareVersion(sFirmware);
        str = sFirmware.c_str();
    }
    else
        str = "N/A";
}
void X2Rotator::deviceInfoModel(BasicStringInterface& str)							
{
    str = "Falcon Rotator";
}

void X2Rotator::driverInfoDetailedInfo(BasicStringInterface& str) const
{
    str = str = "Falcon Rotator X2 plugin by Rodolphe Pineau";;
}
double X2Rotator::driverInfoVersion(void) const				
{
	return DRIVER_VERSION;
}

int	X2Rotator::position(double& dPosition)			
{
    int nErr = SB_OK;
    
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    nErr = mRotator.getPosition(dPosition);

    return nErr;
}
int	X2Rotator::abort(void)							
{
    int nErr = SB_OK;
    
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    nErr = mRotator.haltFalcon();

    return nErr;
}

int	X2Rotator::startRotatorGoto(const double& dTargetPosition)	
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = mRotator.gotoPosition(dTargetPosition);
    if(nErr)
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_ROTATOR, nErr);

    else
        return SB_OK;
}
int	X2Rotator::isCompleteRotatorGoto(bool& bComplete) const	
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Rotator* pMe = (X2Rotator*)this;
    X2MutexLocker ml(pMe->GetMutex());
    nErr = pMe->mRotator.isGoToComplete(bComplete);

    if(nErr)
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_ROTATOR, nErr);
    return nErr;

}
int	X2Rotator::endRotatorGoto(void)							
{
    int nErr = SB_OK;
    double dPosition;
    
    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    nErr = mRotator.getPosition(dPosition);
    if(nErr)
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_ROTATOR, nErr);
    
    return nErr;
}


//
// SerialPortParams2Interface
//
#pragma mark - SerialPortParams2Interface

void X2Rotator::portName(BasicStringInterface& str) const
{
    char szPortName[SERIAL_BUFFER_SIZE];

    portNameOnToCharPtr(szPortName, SERIAL_BUFFER_SIZE);

    str = szPortName;

}

void X2Rotator::setPortName(const char* szPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, szPort);
}


void X2Rotator::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
}

