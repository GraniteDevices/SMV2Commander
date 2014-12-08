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

    void on_test_clicked();

private:
    Ui::MW *ui;

    void logMessage(MsgSeverity severity, QString msg);
    bool checkAndReportSMBusErrors();
    QString stringifySMBusErrors(SM_STATUS smStat, smint32 smDeviceErrors);

    smint32 deviceAddress;
    smbus busHandle;
};

#endif // MW_H
