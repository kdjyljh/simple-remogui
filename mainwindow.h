#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "MediaStreamWidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    virtual void resizeEvent(QResizeEvent *event) override;

private slots:
    void clickPoint(QPointF point);
    void reloadStreamDone(bool gotStream);

private:
    Ui::MainWindow *ui;
    MediaStreamWidget *mediaStreamWidget;
};

#endif // MAINWINDOW_H
