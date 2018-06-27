#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QVBoxLayout>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
//    mediaStreamWidget = new MediaStreamWidget("rtsp://192.168.61.215/chn1");
//    mediaStreamWidget = new MediaStreamWidget("rtsp://192.168.0.1/chn1");
    mediaStreamWidget = new MediaStreamWidget("rtsp://184.72.239.149/vod/mp4:BigBuckBunny_175k.mov", this);
//    mediaStreamWidget = new MediaStreamWidget("rtsp://127.0.0.1:1235/test1.sdp");

//    layout()->setAlignment(Qt::AlignTop);
//    layout()->addWidget(mediaStreamWidget);
    mediaStreamWidget->reloadStream();

    connect(mediaStreamWidget, SIGNAL(mouseClickPointGot(QPointF)), this, SLOT(clickPoint(QPointF)));
    connect(mediaStreamWidget, SIGNAL(reloadStreamDone(bool)), this, SLOT(reloadStreamDone(bool)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::clickPoint(QPointF point)
{
    qDebug() << point;
}

void MainWindow::reloadStreamDone(bool gotStream)
{
    qDebug() << "MainWindow::reloadStreamDone " << gotStream;
    mediaStreamWidget->updateGeometry(geometry());
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    mediaStreamWidget->updateGeometry(geometry());
//    qDebug() << geometry();
}
