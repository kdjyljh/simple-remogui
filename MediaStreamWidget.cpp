#include "MediaStreamWidget.h"
#include <boost/make_shared.hpp>
#include <QPaintEvent>
#include <QPainter>
#include <QMouseEvent>
#include <glog/logging.h>
#include <QDebug>

MediaStreamWidget::MediaStreamWidget(std::string streamUrl, QWidget *parent) :
        QWidget(parent),
        url(streamUrl),
        curFrame(nullptr),
        parentGeo(QRect(0, 0, 0, 0)),
        mediaStreamProc(new MediaStreamProc) {
    if (url.empty()) {
        url = "rtsp://192.168.0.1/chn1"; //默认url
    }

//    image = QImage("/home/jianghualuo/work/data/images/0ed2c9154d04d2ce3403fd0ba3543505.jpg");

    connect(mediaStreamProc.get(), SIGNAL(readStreamDone(bool)), this, SIGNAL(reloadStreamDone(bool)));
    connect(mediaStreamProc.get(), SIGNAL(imageGot()), this, SLOT(drawImage()), Qt::BlockingQueuedConnection);
}

void MediaStreamWidget::reloadStream(std::string streamUrl) {
    if (!streamUrl.empty()) {
        url = streamUrl;
    }
    mediaStreamProc->setUrl(url);

    mediaStreamProc->readStream();
}

void MediaStreamWidget::mouseReleaseEvent(QMouseEvent *ev) {
    QPoint recPos = ev->pos();
    double posX = static_cast<double>(recPos.x()) / width();
    double posY = static_cast<double>(recPos.y()) / height();
    emit mouseClickPointGot(QPointF(posX, posY));
}

void MediaStreamWidget::paintEvent(QPaintEvent *event) {
    if (nullptr == curFrame) {
        return;
    }

    QPainter painter(this);

//    image = QImage(curFrame->image.data[0], curFrame->image.width, curFrame->image.height, QImage::Format_RGB32)/*.copy()*/;
    if (!image.isNull()) {
        QRect refRect = parentGeo;
        if (refRect.isNull()) {
            refRect = rect();
        }
        //按比例缩放,保持长宽比,按宽高的最小值确定缩放比例
        int w = std::min((double)refRect.width(), ((double)refRect.height()) / image.height() * image.width());
        int h = ((double)image.height() / image.width()) * w;
        int x = (refRect.width() - w) / 2.0;
        int y = 0;
//        QImage showImage = image.scaled(parentGeo.size(), Qt::KeepAspectRatio); //不需要scale,在drawImage里面scale
        setGeometry(x, y, w, h);
        painter.drawImage(rect(), image, image.rect(), Qt::NoFormatConversion);
//        image = QImage();
//        av_freep(&curFrame->image.data[0]); //不能用free(),必须用av_freep()
    }

    if (curFrame->hasAiInfo) {
        MediaFrame_AI_Info aiInfo = curFrame->ai_info;
        QPen pen(Qt::red, Qt::SolidLine);
        QRect ret;
        pen.setColor(Qt::red); //body使用红色
        pen.setWidth(5);
        painter.setPen(pen);
        for (int i = 0; i < aiInfo.u8NumPerson; ++i) {
            float x1 = (*((float *) ((char *) aiInfo.u8PersonRois + i * 16)));
            float y1 = (*((float *) ((char *) aiInfo.u8PersonRois + i * 16 + 4)));
            float x2 = (*((float *) ((char *) aiInfo.u8PersonRois + i * 16 + 8)));
            float y2 = (*((float *) ((char *) aiInfo.u8PersonRois + i * 16 + 12)));
//            LOG(INFO) << x1 << ", " << y1 << ", "<< x2 << ", " << y2;
            ret = QRect(QPoint(width() * x1, height() * y1),
                        QPoint(width() * x2, height() * y2));
            painter.drawRect(ret);
        }
        pen.setColor(Qt::green); //face使用绿色
        painter.setPen(pen);
        for (int i = 0; i < aiInfo.u8NumFace; ++i) {
            ret = QRect(QPoint(width() * (*((float *) ((char *) aiInfo.u8FaceRois + i * 16))),
                               height() * (*((float *) ((char *) aiInfo.u8FaceRois + i * 16 + 4)))),
                        QPoint(width() * (*((float *) ((char *) aiInfo.u8FaceRois + i * 16 + 8))),
                               height() * (*((float *) ((char *) aiInfo.u8FaceRois + i * 16 + 12)))));
            painter.drawRect(ret);
        }
        pen.setColor(Qt::white); //target使用白色
        painter.setPen(pen);
        for (int i = 0; i < aiInfo.u8NumTarget; ++i) {
            float x1 = (*((float *) ((char *) aiInfo.u8TargetRois + 2 + i * 18)));
            float y1 = (*((float *) ((char *) aiInfo.u8TargetRois + 2 + i * 18 + 4)));
            float x2 = (*((float *) ((char *) aiInfo.u8TargetRois + 2 + i * 18 + 8)));
            float y2 = (*((float *) ((char *) aiInfo.u8TargetRois + 2 + i * 18 + 12)));
//            LOG(INFO) << x1 << ", " << y1 << ", "<< x2 << ", " << y2;
            ret = QRect(QPoint(width() * x1, height() * y1),
                        QPoint(width() * x2, height() * y2));
            painter.drawRect(ret);
        }
        pen.setColor(Qt::blue); //hand使用蓝色
        painter.setPen(pen);
        for (int i = 0; i < aiInfo.u8NumHand; ++i) {
            ret = QRect(QPoint(width() * (*((float *) ((char *) aiInfo.u8Hand + i * 16))),
                               height() * (*((float *) ((char *) aiInfo.u8Hand + i * 16 + 4)))),
                        QPoint(width() * (*((float *) ((char *) aiInfo.u8Hand + i * 16 + 8))),
                               height() * (*((float *) ((char *) aiInfo.u8Hand + i * 16 + 12)))));
            painter.drawRect(ret);
        }

        painter.setPen(QColor(0, 160, 230, 150)); //当前手势使用透明蓝色
        painter.setPen(pen);
        QFont font;
        font.setPointSize(16);
        painter.setFont(font);
        painter.drawText(50, 50, QString("当前手势: (type:%1,count:%2)").arg(aiInfo.u8HpType).arg(aiInfo.u8HpCount));
    }

//    LOG(INFO) << "MediaStreamWidget::paintEvent done";
}

void MediaStreamWidget::drawImage() {
    if (mediaStreamProc->getCurFrame()) {
        curFrame = mediaStreamProc->getCurFrame();
        image = QImage(curFrame->image.data[0], curFrame->image.width, curFrame->image.height, QImage::Format_RGB32).copy();
//        image = QImage(curFrame->image.width, curFrame->image.height, QImage::Format_RGB32);
//        memcpy(image.bits(), curFrame->image.data[0], image.byteCount());
        //getCurFrame调用返回后可能立刻会被释放
        repaint(); //repaint()不能保证paintEvent调用完成后再返回,所以必须在这里进行图片内存的copy
    }
}

void MediaStreamWidget::updateGeometry(const QRect &parentGeo) {
    this->parentGeo = parentGeo;
    update();
}
