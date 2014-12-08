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

bool MW::checkAndReportSMBusErrors()
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
    smRead1Parameter(busHandle,deviceAddress,SMP_CUMULATIVE_STATUS,&SMDeviceSideCommStatus);

    //read cumulative bus status errors and all convert (1) and (2) bits to human readable form:
    errs=stringifySMBusErrors(getCumulativeStatus(busHandle), SMDeviceSideCommStatus);

    //reset errors bits
    smSetParameter(busHandle,deviceAddress,SMP_CUMULATIVE_STATUS,0);
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
