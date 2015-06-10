#include "mw.h"
#include "ui_mw.h"

MW::MW(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MW)
{
    busHandle=-1;
    ui->setupUi(this);
    logMessage(Debug,QString("Qt version %1").arg(QT_VERSION_STR));
}

MW::~MW()
{
    on_disconnect_clicked();
    delete ui;
}

void MW::logMessage(MW::MsgSeverity severity, QString msg)
{
    //TODO add color colding for different severities
    ui->log->appendHtml(msg);
}

//if fast=true then do only checks that do not need communication via SM bus (local checks only,
//such as errors in received packets, but not reporting errors in invalid parameter values)*/
bool MW::checkAndReportSMBusErrors(bool fast)
{
    QString errs;

    /*SM bus & SM devices have three categories of status & error bits:
     *
     *1) Bus status & error bits. These are returned on each SM library call (the SM_STAUTS type)
     *   and accumulated into a internal variable that may be read by getCumulativeStatus function.
     *   This value reports errors that happen on with communication layer (phyiscal device problems
     *   such as not available of bus device or checksum error).
     *
     *2) Device side SM status & error bits. reading this requires working connection to a target device
     *   in order to read SMP_CUMULATIVE_STATUS parameter.
     *   This value contains errors that successfully were transferred to target but were not accepted
     *   by some reason (i.e. if invalid paramter address or value was used).
     *
     *3) Device specific state & errors, such as servo drive stauts and fault bits on SMP_STATUS and
     *   SMP_FAULTS parameters. These states are not checked in this function.
     *
     */


    //read SMP_CUMULATIVE_STATUS
    smint32 SMDeviceSideCommStatus;
    if(fast==false)
    {
        smRead1Parameter(busHandle,deviceAddress,SMP_CUMULATIVE_STATUS,&SMDeviceSideCommStatus);
        //if we have some error bits on, reset them, so we can spot new errors later
        if(SMDeviceSideCommStatus!=0)
        {
            smSetParameter(busHandle,deviceAddress,SMP_CUMULATIVE_STATUS,0);
        }
    }
    else
    {
        SMDeviceSideCommStatus=0;//a cludge to avoid false errors being reported in stringifySMBusErrors
    }

    //read cumulative bus status errors and all convert (1) and (2) bits to human readable form:
    errs=stringifySMBusErrors(getCumulativeStatus(busHandle), SMDeviceSideCommStatus);

    //reset local errors bits
    resetCumulativeStatus(busHandle);

    //if there were errors, log them
    if(errs.isEmpty()==false)
    {
       logMessage(Error,errs);
       return true;
    }

    return false;
}

QString MW::stringifySMBusErrors(SM_STATUS smStat, smint32 smDeviceErrors)
{
    QString errorString;

    if( ((smStat!=SM_OK && smStat!=SM_NONE) || smDeviceErrors!=SMP_CMD_STATUS_ACK ))
    {
            QString errorFlags, smErrorFlags;
            //these faults are from SM bus host side
            if(smStat&SM_ERR_NODEVICE) errorFlags+="* NoDevice (check port name)<br>";
            if(smStat&SM_ERR_PARAMETER) errorFlags+="* InvalidParameter (API)<br>";
            if(smStat&SM_ERR_COMMUNICATION) errorFlags+="* Communication (cheksum mismatch)<br>";
            if(smStat&SM_ERR_LENGTH) errorFlags+="* DataLegth (timeout or app error)<br>";
            if(smStat&SM_ERR_BUS) errorFlags+="* BusError<br>";

            if(!(smStat&SM_ERR_NODEVICE))//ignore device side faults if nodevice is active because it would make no sense
            {
                //device errors are read from the device (so connection must be working). these are error flags of device side of SM bus
                if(smDeviceErrors&SMP_CMD_STATUS_NACK) smErrorFlags+="* Command fail (NACK)<br>";
                if(smDeviceErrors&SMP_CMD_STATUS_INVALID_ADDR) smErrorFlags+="* Invalid param address<br>";
                if(smDeviceErrors&SMP_CMD_STATUS_INVALID_VALUE) smErrorFlags+="* Invalid param value<br>";
                if(smDeviceErrors&SMP_CMD_STATUS_VALUE_TOO_HIGH) smErrorFlags+="* Value too high<br>";
                if(smDeviceErrors&SMP_CMD_STATUS_VALUE_TOO_LOW) smErrorFlags+="* Value too low<br>";

            }
            errorString="";
            if(errorFlags.size())
                errorString+="Bus error flags: <br>"+errorFlags+"<br>";
            else
                errorString="Communication error.";
            if(smErrorFlags.size()) errorString+="<br>Device errors: <br>"+smErrorFlags;
    }

    return errorString;
}

void MW::on_connect_clicked()
{
    busHandle=smOpenBus(ui->busName->text().toLatin1());
    if(busHandle>=0)
    {
        logMessage(Info,"Successfully connected bus "+ui->busName->text());
        deviceAddress=ui->deviceAddress->value();
    }
    else
        logMessage(Error,"Couldn't connect to bus "+ui->busName->text());

    checkAndReportSMBusErrors();
}

void MW::on_disconnect_clicked()
{
    logMessage(Info,"Closing bus (if open)");
    smCloseBus(busHandle);
}

void MW::on_enableDrive_clicked()
{
    logMessage(Info,"Enabling drive");
    smSetParameter(busHandle,deviceAddress,SMP_CONTROL_BITS1,SMP_CB1_ENABLE);
    checkAndReportSMBusErrors();
}

void MW::on_disableDrive_clicked()
{
    logMessage(Info,"Disabling drive");
    smSetParameter(busHandle,deviceAddress,SMP_CONTROL_BITS1,0);
    checkAndReportSMBusErrors();
}

void MW::on_movehome_clicked()
{
    logMessage(Info,"Start homing");
    smSetParameter(busHandle,deviceAddress,SMP_HOMING_CONTROL,1);
    checkAndReportSMBusErrors();
}

void MW::on_clearfaults_clicked()
{
    logMessage(Info,"Clearing faults");
    //faults are cleared by writing 0 to faults register
    smSetParameter(busHandle,deviceAddress,SMP_FAULTS,0);
    checkAndReportSMBusErrors();
}

void MW::on_setParams_clicked()
{
    logMessage(Info,"Setting params");
    smSetParameter(busHandle,deviceAddress,SMP_TRAJ_PLANNER_VEL,ui->CVL->value());
    checkAndReportSMBusErrors();
}

void MW::on_setSetpoint_clicked()
{
    logMessage(Info,QString("Setpoint to %1").arg(ui->setPoint->value()));
    smSetParameter(busHandle,deviceAddress,SMP_ABSOLUTE_POS_TARGET,ui->setPoint->value());
    checkAndReportSMBusErrors();
}



void MW::on_readStatus_clicked()
{
    logMessage(Info,"Reading status");
    smint32 currentSetpoint,currentPosFeedback, statusBits, faultBits;
    //with this function we can read 3 parameters at once (faster than calling three one parameter read commands)
    smRead3Parameters(busHandle,deviceAddress,SMP_ABSOLUTE_POS_TARGET,&currentSetpoint,SMP_ACTUAL_POSITION_FB,&currentPosFeedback, SMP_STATUS, &statusBits);
    smRead1Parameter(busHandle,deviceAddress,SMP_FAULTS,&faultBits);

    if(checkAndReportSMBusErrors()==false)//if no errors
    {
        logMessage(Info,QString(
                       "*setpoint of %1 <br>"
                       "*position feedback %2 (currently gives raw 16 bit position counter value, this is subject to change on final SMV2 release)<br>"
                       "*status bits %3<br>"
                       "*fault bits %4<br>").arg(currentSetpoint).arg(currentPosFeedback).arg(statusBits).arg(faultBits));

        logMessage(Info,QString("Some of the status bits deciphered:<br>"
                                "*enabled=%1<br> "
                                "*run=%2<br> "
                                "*homing active=%3<br> "
                                "*fault stop=%4<br>").arg(bool(statusBits&STAT_ENABLED)).arg(bool(statusBits&STAT_RUN)).arg(bool(statusBits&STAT_HOMING)).arg(bool(statusBits&STAT_FAULTSTOP)));

        logMessage(Info,QString("Some of the fault bits deciphered:<br>"
                                "*tracking error=%1<br> "
                                "*over velocity=%2<br> "
                                "*over current=%3<br> "
                                "*under voltage=%4<br>").arg(bool(faultBits&FLT_FOLLOWERROR)).arg(bool(faultBits&FLT_OVERVELOCITY)).arg(bool(faultBits&FLT_OVERCURRENT)).arg(bool(faultBits&FLT_UNDERVOLTAGE)));
    }

}

/*
  //an raw example to run homing in torque mode
  //drive must be fully configured in torque and position modes, homing must be configured and drive enabled to run this
void MW::on_test_clicked()
{
     smSetParameter(busHandle, deviceAddress, SMP_CONTROL_MODE, CM_POSITION);//set drive in position mode

     smSetParameter(busHandle,deviceAddress,SMP_HOMING_CONTROL, 1);//start homing

     smint32 status, center;
     smRead1Parameter(busHandle, deviceAddress, SMP_STATUS, &status);

     while (bool(status & STAT_HOMING)) {
        smRead1Parameter(busHandle, deviceAddress, SMP_STATUS, &status);//loop here until homing is done. TODO add timeout and status reporting or better, do this in separate thread
     }
     smRead1Parameter(busHandle, deviceAddress, SMP_ACTUAL_POSITION_FB, &center);//read position where we ended up

     smSetParameter(busHandle, deviceAddress, SMP_CONTROL_MODE, CM_TORQUE);//set to torque mode
}
*/

void MW::on_readArbitraryParameter_clicked()
{
    logMessage(Info,QString("Reading parameter from address %1").arg(ui->arbitraryParameterNumber->value()));
    smint32 value;
    smRead1Parameter(busHandle,deviceAddress,ui->arbitraryParameterNumber->value(),&value);

    if(checkAndReportSMBusErrors()==false)
    {
        logMessage(Info,QString("Parameter read, value is %1").arg(value));
        ui->arbitraryParameterValue->setValue(value);//set it to gui also
    }
    else
        logMessage(Info,QString("Check that parameter address is defined in simplemotion_defs.h"));
}

void MW::on_writeArbitraryParameter_clicked()
{
    logMessage(Info,QString("Setting value %2 to parameter address %1").arg(ui->arbitraryParameterNumber->value()).arg(ui->arbitraryParameterValue->value()));
    smSetParameter(busHandle,deviceAddress,ui->arbitraryParameterNumber->value(),ui->arbitraryParameterValue->value());
    if(checkAndReportSMBusErrors()==true)
    {
        logMessage(Info,QString("Check that parameter address is defined in simplemotion_defs.h and value is within valid min-max range."));
    }
}


/* This method shows how to perform mutliple read/write tasks with single SM transmission.
 *
 * For optimization tips, see: http://granitedevices.com/wiki/Optimizing_SimpleMotion_V2_performance
 * And: http://granitedevices.com/wiki/SMV2USB_adapter_latency_settings
 */
void MW::on_speedOptimizedWriteRead_clicked()
{
    //initialize some variables that do not need to be updated on every cycle.
    //TODO: implement it so that it gets called only on the first execution and not always (uncomment the lines etc)!
    //if(fastWriteReadInitialised==false)
    //{
        setupSpeedOptimizedWriteRead();
    //    fastWriteReadInitialised=true;
    //}


    /* Variables:
    -paramValue1 is the parameter value written to parameter number defined in paramAddr1
    -readOutParamAddr1-3 will define the parameter numbers to read out into readout1-3
    */

    //VALUES DEFINED
    int paramAddr1=SMP_ABSOLUTE_SETPOINT;
    int paramValue1=ui->setPoint->value();
    int readOutParamAddr1=SMP_ACTUAL_TORQUE;
    int readOutParamAddr2=SMP_ACTUAL_POSITION_FB;
    int readOutParamAddr3=SMP_ACTUAL_VELOCITY_FB;
    smint32 nul, readout1, readout2, readout3;

    //WRITING PARAMETERS
    //set an parameter number where next values are written. This line consumes 2 bytes of outbound payload
    smAppendSMCommandToQueue( busHandle, SMPCMD_SETPARAMADDR, paramAddr1 );

    //write the parameter value to previously set parameter address. SMPCMD_24B consumes 3 bytes of outbound payload
    smAppendSMCommandToQueue( busHandle, SMPCMD_24B, paramValue1);

    //READING PARAMETERS
    //set parameter parameter number which will be returned from each command executed after this
    smAppendSMCommandToQueue( busHandle, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_ADDR ); //consumes 2 bytes
    /*write new values to parameter SMP_RETURN_PARAM_ADDR. after execution these commands return the parameter values
    defined by readOutParamAddrN*/
    smAppendSMCommandToQueue( busHandle, SMPCMD_24B, readOutParamAddr1 ); //consumes 3 bytes
    smAppendSMCommandToQueue( busHandle, SMPCMD_24B, readOutParamAddr2 ); //consumes 3 bytes
    smAppendSMCommandToQueue( busHandle, SMPCMD_24B, readOutParamAddr3 ); //consumes 3 bytes

    //execute commands over SM bus
    smExecuteCommandQueue( busHandle, deviceAddress );

    /*read value commands (one per each append command). Size of inbound payload depends on
    what was set to SMP_RETURN_PARAM_LEN at initialization */
    smGetQueuedSMCommandReturnValue( busHandle,&nul );
    smGetQueuedSMCommandReturnValue( busHandle,&nul );

    smGetQueuedSMCommandReturnValue( busHandle,&nul );

    /*the next readouts reflect to the last three commands appended before smExecuteCommandQueue
    and here we get the readout values*/

    smGetQueuedSMCommandReturnValue( busHandle,&readout1 );
    smGetQueuedSMCommandReturnValue( busHandle,&readout2 );
    smGetQueuedSMCommandReturnValue( busHandle,&readout3 );

    //argument fast=true will check only local SM bus errors (received packet validity). Will not report if drive ignores some written value.
    //During debugging, set argument fast=false to have full error checking active.
    checkAndReportSMBusErrors(true);

    //print results
    logMessage(Info,QString("Setpoint set to %1 and same time readouts were: ACTUAL_TORQUE=%2, ACTUAL_POSITION_FB=%3, ACTUAL_VELOCITY_FB=%4").arg(paramValue1).arg(readout1).arg(readout2).arg(readout3));
}

/* this initialization function needs to be called only once before on_speedOptimizedWriteRead_clicked() will work properly
 * assuming that these values are not modified by other functions */
void MW::setupSpeedOptimizedWriteRead()
{
    /*setting return data lenght to 24 bits (22 bits maximum returned integer value
    as 2 bits are used by SM protocol). 16 bit data will fit in return value without clipping. */
    smSetParameter( busHandle, deviceAddress, SMP_RETURN_PARAM_LEN, SMPRET_24B);
}
