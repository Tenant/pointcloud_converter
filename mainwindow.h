#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

enum SRC_FMT{
    PCAP_HDL32 = 1
};

enum DST_FMT{
    ROSBAG_POINTCLOUD2 = 1,
    CSV_TIMESTAMP = 2
};

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void slotSelectSourceData();
    void slotSelectDestination();
    void slotStartProcessing();

private:
    Ui::MainWindow *ui;

    QString _strSourcePath;
    QString _strSourceDir;
    QString _strSourceName;
};
#endif // MAINWINDOW_H
