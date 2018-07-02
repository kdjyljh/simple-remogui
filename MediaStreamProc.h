#ifndef MediaStreamProc_H
#define MediaStreamProc_H

#include <QObject>
#include <QImage>
#include <QThread>

#include <stdio.h>
#include <deque>
#include <map>
#include <utility>
#include <boost/thread.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/pixdesc.h>
#ifdef VAAPI_ENABLE
#include <libavutil/hwcontext.h>
#endif
#include <libavutil/opt.h>
#include <libavutil/avassert.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavformat/avformat.h>
}

#pragma pack(1)
struct MediaFrame_AI_Info
{
    uint8_t u8SDKStatus;
    uint64_t u64TimeStamp;
    uint8_t u8HpType;
    uint8_t u8HpCount;
    uint8_t u8CapSF;
    uint8_t u8CapSA;
    uint8_t u8CapSL;
    uint8_t u8CtrlType;
    float fCtrlDx;
    float fCtrlDy;
    float fCtrlScale;
    float fCtrlGap;
    uint8_t u8NumTarget;
    uint8_t u8NumPerson;
    uint8_t u8NumFace;
    uint8_t u8NumHand;
    uint8_t u8TargetRois[2][18];
    uint8_t u8PersonRois[15][16];
    uint8_t u8FaceRois[15][16];
    uint8_t u8Hand[15][16];
};
#pragma pack()

struct MediaFrame {
    MediaFrame_AI_Info ai_info;
    AVFrame image;
    bool hasAiInfo;
};

class MediaStreamProc : public QObject
{
    Q_OBJECT
public:
    MediaStreamProc(QObject *parent = nullptr);
    ~MediaStreamProc();
//    AVFrame *getDecodedFrame() {return &decoded_frame;}
    MediaFrame *getCurFrame();
    bool isValid() {return streamDecoderReady && streamInputReady;}
    void setUrl(std::string newUrl) {
        if (!newUrl.empty()) {
            url = newUrl;
        }
    }

signals:
    void imageGot(const QImage &image);
    void imageGot();
    bool readStreamDone(bool gotStream);

public slots:
    bool readStream(); // 重新获取流
    bool syncReadStream();

private slots:
    void _readStream();

private:
    void readFrame();
    void decodeFrame();
    void play();
    int decode_write_normal(AVCodecContext *avctx, AVPacket *packet);
    //一个packet里面可能以有多个算法nal,全部找出来放到aiRoisMap中
    void putAiInfo2MapFromFrame(const AVPacket &packet);

#ifdef VAAPI_ENABLE
    int decode_write_vaapi(AVCodecContext *avctx, AVPacket *packet);
    static enum AVPixelFormat get_hw_format(AVCodecContext *ctx,
                                     const enum AVPixelFormat *pix_fmts);
    int hw_decoder_init(AVCodecContext *ctx, const enum AVHWDeviceType type);
    int vaapiInit(); //返回0表示成功
#endif
    int init(); //返回0表示成功
    int normalInit(); //返回0表示成功
    void deInit();
    void pushFrame(const MediaFrame &frame);
    void popFrame(MediaFrame &frame);
    void pushPacket(const AVPacket &packet);
    void popPacket(AVPacket &packet);

    std::map<uint64_t, MediaFrame_AI_Info>::iterator findRoisMapIt(int64_t pts);

private:
    AVFormatContext *input_ctx;
    AVStream *video;
    AVCodecContext *decoder_ctx;
    AVCodec *decoder;
    AVBufferRef *hw_device_ctx;
    AVInputFormat *input_format;
    int video_stream;
//    AVPacket packet;
//    AVFrame decoded_frame;
    std::string url;
    std::string deviceType;
    int frame_width;
    int frame_height;
    MediaFrame curFrame;
    std::string decoderCfg;

    enum DecoderType {
        DecoderType_None    = 0,
        DecoderType_Normal    = 1,
        DecoderType_Vaapi   = 2,
    } decoderType;

private:
    boost::mutex mtxStreamInputReady;
    boost::condition_variable cvStreamInputReady;
    bool streamInputReady; //多线程共享变量，需要加锁

    boost::mutex mtxStreamDecoderReady;
    boost::condition_variable cvStreamDecoderReady;
    bool streamDecoderReady; //多线程共享变量，需要加锁

    QThread *readStreamThread; //连接流线程
    boost::thread readFrameThread; // 读取数据包线程
    boost::thread playThread;
    boost::thread decodeFrameThread;

    std::deque<AVPacket> packetsQuue;
    boost::mutex mtxPacketsQuue;
    boost::condition_variable cvPacketsQuue;

    std::deque<MediaFrame> frameQueue;
    boost::mutex mtxFrameQueue;
    boost::condition_variable cvFrameQueue;
    boost::mutex mtxFrameQueueFull;
    boost::condition_variable cvFrameQueueFull;
    const unsigned int frameQueueSize;

    std::deque<AVFrame> pendingAiRoiFrameQue;
    std::map<uint64_t, MediaFrame_AI_Info> aiRoisMap;
    unsigned int pendingFrameCounter; //寻找包数目计数器
    const unsigned int pendingFrameSize; //当在中未找到roi时,继续寻找的包数目
    //当pendingAiRoiFrameQue大小超过这个值时
    //会将pendingAiRoiFrameQue里面的frame直接显示(不匹配框),取出的frame个数为drainPendingAiRoiFrameSize
    const unsigned int drainPendingAiRoiFrameMax;
    const unsigned int drainPendingAiRoiFrameSize;
    const unsigned int aiRoisMapMaxSize;
    const unsigned int syncRoiDiffMax; //寻找框时框pts和frame的pts之间差值的最大值
    AVFrame curSyncRoiFrame; //当前正在同步框的frame
};

#endif // MediaStreamProc_H
