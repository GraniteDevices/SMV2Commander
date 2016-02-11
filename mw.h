#ifndef MW_H
#define MW_H

#include <QMainWindow>
#include <QString>
#include <simplemotion.h>

namespace Ui {
class MW;
}

class MW : public QMainWindow
{
    Q_OBJECT
    
public:
    enum MsgSeverity {Debug,Info,Error};

    explicit MW(QWidget *parent = 0);
    ~MW();
public slots:
    
private slots:

    void on_connect_clicked();

    void on_disconnect_clicked();

    void on_enableDrive_clicked();

    void on_disableDrive_clicked();

    void on_movehome_clicked();

    void on_clearfaults_clicked();

    void on_setParams_clicked();

    void on_setSetpoint_clicked();

    void on_readStatus_clicked();

    void on_readArbitraryParameter_clicked();

    void on_writeArbitraryParameter_clicked();

    void on_speedOptimizedWriteRead_clicked();

private:
    Ui::MW *ui;


    //generates one line string to tell digital input state. bits=register read from device, bitNumber=bit that identifies status of the input.
    QString formatDigitalStateAsString(QString inputName, smint32 bits, int bitNumber);
    //generate message telling state of all digital inputs. digitalInsRegister=register read from device.
    QString getDigitalInsString( smint32 digitalInsRegister );

    void setupSpeedOptimizedWriteRead();
    void logMessage(MsgSeverity severity, QString msg);
    //if fast=true then do only checks that do not need communication via SM bus (local checks only,
    //such as errors in received packets, but not reporting errors in invalid parameter values)*/
    bool checkAndReportSMBusErrors(bool fast=false);
    QString stringifySMBusErrors(SM_STATUS smStat, smint32 smDeviceErrors);

    smint32 deviceAddress;
    smbus busHandle;
};

#endif // MW_H
