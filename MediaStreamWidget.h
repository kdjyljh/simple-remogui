#ifndef MEDIASTREAMWIDGET_H
#define MEDIASTREAMWIDGET_H

#include <QWidget>
#include <string>
#include <boost/shared_ptr.hpp>
#include "MediaStreamProc.h"

class MediaStreamWidget : public QWidget
{
    Q_OBJECT
public:
    MediaStreamWidget(std::string streamUrl, QWidget *parent = nullptr);

public slots:
    //重新读取视频流，立即返回
    //如果指定streamUrl则调用会使用该url，否则使用构造对象时指定的url
    //注意:此函数调用可能会改变对象的url
    void reloadStream(std::string streamUrl = "");
    void updateGeometry(const QRect &parentGeo);

signals:
    void mouseClickPointGot(QPointF point);
    void reloadStreamDone(bool gotStream);

protected:
    virtual void mouseReleaseEvent(QMouseEvent *ev) override;
    virtual void paintEvent(QPaintEvent *event) override;

private slots:
    void drawImage();

private:
    std::string url;
    boost::shared_ptr<MediaStreamProc> mediaStreamProc;
    MediaFrame *curFrame;
    QImage image;
    QRect parentGeo;
};

#endif // MEDIASTREAMWIDGET_H
