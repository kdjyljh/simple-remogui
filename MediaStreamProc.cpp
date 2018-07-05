#include "MediaStreamProc.h"
//#include "ffmpeg-custom.h"
#include <glog/logging.h>
#include <QTimer>
#include <boost/asio.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
//#include "commlog.h"

#ifdef VAAPI_ENABLE
static enum AVPixelFormat hw_pix_fmt;
enum AVHWDeviceType type;
int MediaStreamProc::hw_decoder_init(AVCodecContext *ctx, const enum AVHWDeviceType type)
{
    int err = 0;

    if ((err = av_hwdevice_ctx_create(&hw_device_ctx, type,
                                      nullptr, nullptr, 0)) < 0) {
        LOG(INFO) << "Failed to create specified HW device";
        return err;
    }
    ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);

    return err;
}

enum AVPixelFormat MediaStreamProc::get_hw_format(AVCodecContext *ctx,
                                        const enum AVPixelFormat *pix_fmts)
{
    const enum AVPixelFormat *p;

    for (p = pix_fmts; *p != -1; p++) {
        if (*p == hw_pix_fmt)
            return *p;
    }

    LOG(INFO) << "Failed to get HW surface format.";
    return AV_PIX_FMT_NONE;
}

int MediaStreamProc::decode_write_vaapi(AVCodecContext *avctx, AVPacket *packet) {
    if (nullptr == avctx || nullptr == packet) {
        return -1;
    }

    int ret = avcodec_send_packet(avctx, packet);
    if (ret < 0) {
        LOG(INFO) << "MediaStreamProc::decode_write send packet error";
        return ret;
    }

    MediaFrame mediaFrame;
    memset(&mediaFrame, 0, sizeof(MediaFrame_AI_Info));
    mediaFrame.hasAiInfo = decodeAiInfoFrame(*packet, mediaFrame.ai_info);

    while (ret >= 0) {
        AVFrame *frame = nullptr;
        AVFrame *sw_frame = nullptr;
        if (nullptr == (frame = av_frame_alloc()) || (nullptr == (sw_frame = av_frame_alloc()))) {
            LOG(INFO) << "MediaStreamProc::decode_write alloc frame error";
            ret = AVERROR(ENOMEM);
            return ret;
        }

        ret = avcodec_receive_frame(avctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            av_frame_free(&frame);
            av_frame_free(&sw_frame);
            return 0;
        }
        else if (ret < 0) {
            LOG(INFO) << "MediaStreamProc::decode_write decode error";
            av_frame_free(&frame);
            av_frame_free(&sw_frame);
            return ret;
        }

        AVFrame *tmp_frame = nullptr;
        if (frame->format == hw_pix_fmt) {
            /* retrieve data from GPU to CPU */
            if ((ret = av_hwframe_transfer_data(sw_frame, frame, 0)) < 0) {
                LOG(INFO) << "MediaStreamProc::decode_write transfer data error";
                av_frame_free(&frame);
                av_frame_free(&sw_frame);
                return ret;
            }
            tmp_frame = sw_frame;
        } else
            tmp_frame = frame;

//        int frame_width = tmp_frame->width;
//        int frame_height = tmp_frame->height;
//        struct SwsContext *img_convert_ctx =
//                sws_getContext(frame_width, frame_height, (AVPixelFormat)tmp_frame->format, frame_width, frame_height,
//                                                            AV_PIX_FMT_RGB32, SWS_BICUBIC, 0, 0, 0);;
//        int dstStride[1]{frame_width * 4};
//        uint8_t *dst[1]{(uint8_t*)malloc(frame_height * dstStride[0])};
//        sws_scale(img_convert_ctx, (const uint8_t * const *)tmp_frame->data, tmp_frame->linesize,
//                  0, frame_height, dst, dstStride);
//        QImage image(frame_width, frame_height, QImage::Format_RGB32);
//        memcpy(image.bits(), dst[0], frame_width * frame_height * 4);
//        emit imageGot();

//        av_frame_free(&frame);
//        av_frame_free(&sw_frame);
//        sws_freeContext(img_convert_ctx);
//        free(dst[0]);

        struct SwsContext *img_convert_ctx =
                sws_getContext(tmp_frame->width, tmp_frame->height, (AVPixelFormat)tmp_frame->format,
                               tmp_frame->width, tmp_frame->height, AV_PIX_FMT_RGB32, SWS_FAST_BILINEAR, 0, 0, 0);

        AVFrame decoded_frame;
        frame_width = decoder_ctx->width;
        frame_height = decoder_ctx->height;
        decoded_frame.format = AV_PIX_FMT_RGB32;
        decoded_frame.width = frame_width;
        decoded_frame.height = frame_height;
        if ((ret = av_image_alloc(decoded_frame.data, decoded_frame.linesize,
                                  frame_width, frame_height, AV_PIX_FMT_RGB32, 32)) < 0) {
            LOG(INFO) << "MediaStreamProc::decode_write alloc decoded frame failed";
            ret = AVERROR(ENOMEM);
            av_frame_free(&frame);
            av_frame_free(&sw_frame);
            return ret;
        }
        sws_scale(img_convert_ctx, (const uint8_t * const *)tmp_frame->data, tmp_frame->linesize, 0, tmp_frame->height,
                  decoded_frame.data, decoded_frame.linesize);
        mediaFrame.image = decoded_frame;
        pushFrame(mediaFrame);
//        QImage image(decoded_frame.data[0], tmp_frame->width, tmp_frame->height, QImage::Format_RGB32);
//        QImage image(frame_width, frame_height, QImage::Format_RGB32);
//        memcpy(image.bits(), dst_frame.data[0], frame_width * frame_height * 4); //让QImage管理自己到内存，不使用dst_frame.data[0]
//        emit imageGot(image);

        av_frame_free(&frame);
        av_frame_free(&sw_frame);
        sws_freeContext(img_convert_ctx);
    }

    return ret;
}

int MediaStreamProc::vaapiInit() {
    int i;
    int ret;
    std::string errorMsg;

//    char *device_type = "vdpau";
    const char *device_type = deviceType.c_str();
    const char *input_file = url.c_str();
    int setResult = -1;
    char errorStr[1024] = {0};

    LOG(INFO) << "1.av_hwdevice_find_type_by_name ###########################";
    type = av_hwdevice_find_type_by_name(device_type);
    if (type == AV_HWDEVICE_TYPE_NONE) {
        char msg[1024];
        int size = 0;
        size = snprintf(msg, sizeof(msg), "Device type %s is not supported.\n", device_type);
        size += snprintf(msg + size, sizeof(msg), "Available device types:");
        while((type = av_hwdevice_iterate_types(type)) != AV_HWDEVICE_TYPE_NONE)
            snprintf(msg + size, sizeof(msg), " %s", av_hwdevice_get_type_name(type));

        errorMsg = msg;
        ret = -1;
        goto error;
    }

    /* open the input file */
    LOG(INFO) << "2.avformat_open_input ###########################";
    if ((ret = avformat_open_input(&input_ctx, input_file, nullptr, nullptr)) != 0) {
        errorMsg += "Cannot open input file "; errorMsg += input_file;
        goto error;
    }
    input_ctx->probesize = 1024;

    LOG(INFO) << "3.avformat_find_stream_info ###########################";
    if ((ret = avformat_find_stream_info(input_ctx, nullptr)) < 0) {
        errorMsg += "Cannot find input stream information";
        goto error;
    }

    /* find the video stream information */
    LOG(INFO) << "4.av_find_best_stream ###########################";
    ret = av_find_best_stream(input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder, 0);
    if (ret < 0) {
        errorMsg += "Cannot find a video stream in the input file";
        goto error;
    }
    video_stream = ret;

    LOG(INFO) << "5.avcodec_get_hw_config ###########################";
    for (i = 0;; i++) {
        const AVCodecHWConfig *config = avcodec_get_hw_config(decoder, i);
        if (!config) {
            fprintf(stderr, "Decoder %s does not support device type %s.\n",
                    decoder->name, av_hwdevice_get_type_name(type));
            errorMsg += "Decoder "; errorMsg += decoder->name;
            errorMsg += " does not support device type "; errorMsg += av_hwdevice_get_type_name(type);
            goto error;
        }
        if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
            config->device_type == type) {
            hw_pix_fmt = config->pix_fmt;
            break;
        }
    }

    LOG(INFO) << "6.avcodec_alloc_context3 ###########################";
    if (!(decoder_ctx = avcodec_alloc_context3(decoder))) {
        ret = AVERROR(ENOMEM);
        errorMsg += "avcodec_alloc_context3 failed";
        goto error;
    }

    LOG(INFO) << "7.avcodec_parameters_to_context ###########################";
    video = input_ctx->streams[video_stream];
    if ((ret = avcodec_parameters_to_context(decoder_ctx, video->codecpar)) < 0) {
        errorMsg += "avcodec_parameters_to_context failed";
        goto error;
    }

    decoder_ctx->get_format  = get_hw_format;
    av_opt_set_int(decoder_ctx, "refcounted_frames", 1, 0);

    LOG(INFO) << "8.hw_decoder_init ###########################";
    if (hw_decoder_init(decoder_ctx, type) < 0) {
        return -1;
    }

    LOG(INFO) << "9.avcodec_open2 ###########################";
    if ((ret = avcodec_open2(decoder_ctx, decoder, nullptr)) < 0) {
        errorMsg += "Failed to open codec for stream #"; errorMsg += video_stream;
        goto error;
    }

//    setResult = av_opt_set(decoder_ctx->priv_data, "preset", "slow", AV_OPT_SEARCH_CHILDREN);
//    setResult = av_opt_set_int(decoder_ctx, "crf", 2, 0);
//    setResult = av_opt_set(decoder_ctx->codec, "preset", "slow", 0);
    av_strerror(setResult, errorStr, 1024);
    LOG(INFO) << "set error" << errorStr;
    setResult = av_opt_set(decoder_ctx->priv_data, "tune", "zerolatency", 0);
    setResult = av_opt_set(decoder_ctx->priv_data, "crf", "51", 0);

    LOG(INFO) << "MediaStreamProc::vaapiInit init video stream success!!!!";
    LOG(INFO) << "MediaStreamProc::vaapiInit decoder name:" << decoder_ctx->codec->name;

    streamDecoderReady = true;
    streamInputReady = true;
    return 0;

    error:
    LOG(INFO) << "MediaStreamProc::vaapiInit failed error:" << errorMsg;
    streamInputReady = false;
    streamDecoderReady = false;
    av_strerror(ret, errorStr, 1024);
    LOG(INFO) << "MediaStreamProc::vaapiInit got an error:" << ret << " :" << errorStr;
    return ret;
}
#endif

#ifdef CUSTOM_FFMPEG
extern "C" {
#include "libavformat/rtpdec.h"
#include "libavformat/rtsp.h"
}
#endif

//rtsp://192.168.0.1/livestream/12
//rtsp://184.72.239.149/vod/mp4://BigBuckBunny_175k.mov
///home/jianghualuo/work/data/videos/bandicam.avi
///home/jianghualuo/work/data/videos/FigureSkating.mp4
MediaStreamProc::MediaStreamProc(QObject *parent) :
        QObject(parent),
        input_ctx(nullptr),
        video(nullptr),
        decoder_ctx(nullptr),
        decoder(nullptr),
        hw_device_ctx(nullptr),
        video_stream(-1),
        streamInputReady(false),
        streamDecoderReady(false),
        frame_width(-1),
        frame_height(-1),
        frameQueueSize(3),
        pendingFrameSize(10),
        pendingFrameCounter(0),
        deviceType("vaapi"),
        drainPendingAiRoiFrameMax(15),
        drainPendingAiRoiFrameSize(1),
        aiRoisMapMaxSize(50),
        syncRoiDiffMax(150),
        decoderType(DecoderType_None),
        readStreamThread(new QThread(this)) {
    LOG(INFO) << "MediaStreamProc::MediaStreamProc constructor this:" << this;
//    av_log_set_level(AV_LOG_SKIP_REPEATED); //关闭ffmpeg显示的log
#ifdef VAAPI_ENABLE
    try {
        boost::property_tree::ptree root;
        boost::property_tree::read_json("remo_gui.json", root);
        url = root.get<std::string>("VideoStreamUrl");
        decoderCfg = root.get<std::string>("VideoStreamDecoder");
    } catch (boost::property_tree::ptree_error &e) {
        LOG(INFO) << "MediaStreamProc::MediaStreamProc json parse error:" << e.what();
    }
#endif

    curFrame.image.format = AV_PIX_FMT_NONE;
    moveToThread(readStreamThread);
    input_format = av_find_input_format("rtsp");
    readStreamThread->start(QThread::HighestPriority);
//    readStream(); //异步读取视频流
}

MediaStreamProc::~MediaStreamProc() {
    readStreamThread->quit();
    readStreamThread->wait();

    deInit();
}

//返回0成功，其他值失败
int MediaStreamProc::init() {
    int ret = -1;
#ifdef VAAPI_ENABLE
    if (decoderCfg.empty() || decoderCfg == "Auto") {
        if ((ret = vaapiInit())) {
            LOG(INFO) << "MediaStreamProc::init vaapiInit failed try normal!!!!!!!!!!!!!";
            deInit();
            if ((ret = normalInit())) {
                LOG(INFO) << "MediaStreamProc::init normalInit failed!!!!!!!!!!!!!";
            } else {
                decoderType = DecoderType_Normal;
            }
        } else {
            decoderType = DecoderType_Vaapi;
        }
    } else if ("Normal" == decoderCfg) {
        ret = normalInit();
        decoderType = DecoderType_Normal;
    } else if ("Vaapi" == decoderCfg) {
        ret = vaapiInit();
        decoderType = DecoderType_Vaapi;
    }
#else
    ret = normalInit();
    decoderType = DecoderType_Normal;
#endif

    return ret;
}

void MediaStreamProc::deInit() {
    //先关闭解码和取流线程，再释放资源
    readFrameThread.interrupt();
    readFrameThread.join();
    playThread.interrupt();
//    playThread.join(); //不使用jion，可能产生死锁
    playThread.detach(); //使用detach释放线程资源
    decodeFrameThread.interrupt();
    decodeFrameThread.join();

    avcodec_close(decoder_ctx);
    avcodec_free_context(&decoder_ctx);
    LOG(INFO) << "MediaStreamProc::deInit end input_ctx:" << input_ctx;
    avformat_close_input(&input_ctx);
    av_buffer_unref(&hw_device_ctx);

    input_ctx = nullptr;
    video = nullptr;
    decoder_ctx = nullptr;
    decoder = nullptr;
    hw_device_ctx = nullptr;
    video_stream = -1;
#ifdef VAAPI_ENABLE
    type = AV_HWDEVICE_TYPE_NONE;
#endif
    streamInputReady = false;
    streamDecoderReady = false;

    pendingFrameCounter = 0;
}

void MediaStreamProc::readFrame() {
    int ret = 0;
    while (ret >= 0) {
        boost::this_thread::interruption_point();

        AVPacket packet;
        {
            boost::unique_lock<boost::mutex> lock(mtxStreamInputReady);
            while (!streamInputReady) {
                cvStreamInputReady.wait(lock);
            }

            if ((ret = av_read_frame(input_ctx, &packet)) < 0)
                break;
        }

        pushPacket(packet);
    }
}

bool MediaStreamProc::readStream() {
    QTimer::singleShot(0, this, SLOT(_readStream()));
    return true;
}

void MediaStreamProc::_readStream() {
    int i = 50; //如果无视频流，重复连接50次
    {
        do {
            LOG(INFO) << "0.deInit #########################";
            deInit();
            if (!init()) {
                break;
            }
            --i;
            boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        } while (i);
    }

    emit readStreamDone(i);

    if (i) {
        readFrameThread = boost::thread(&MediaStreamProc::readFrame, this);
        decodeFrameThread = boost::thread(&MediaStreamProc::decodeFrame, this);
        playThread = boost::thread(&MediaStreamProc::play, this);
        LOG(INFO) << "5.MediaStreamProc::play thread started###########################";
    }
}

void MediaStreamProc::pushFrame(const MediaFrame &frame) {
    boost::unique_lock<boost::mutex> lock(mtxFrameQueue);

    //frameQueueSize控制缓存匹配好框的帧的长度,越大延迟越严重
    if (frameQueue.size() >= frameQueueSize) {
        boost::unique_lock<boost::mutex> lockFull(mtxFrameQueueFull);
        while (frameQueue.size() >= frameQueueSize) {
            lock.unlock();
            cvFrameQueue.notify_one();
            cvFrameQueueFull.wait(lockFull);
        }
        frameQueue.push_back(frame);
    } else {
        frameQueue.push_back(frame);
        cvFrameQueue.notify_one();
    }
}

void MediaStreamProc::popFrame(MediaFrame &frame) {
    boost::unique_lock<boost::mutex> lock(mtxFrameQueue);
    while (frameQueue.empty()) {
        cvFrameQueue.wait(lock);
    }

    frame = frameQueue.front();
    frameQueue.pop_front();

    cvFrameQueueFull.notify_one();
}

void MediaStreamProc::play() {
    //为了使播放速度尽量平稳,会按fps进行播放,当两帧之间的时间不够时会使用sleep补足
    //fps太小会造成延迟越来越大
    int fps = 40;
    boost::posix_time::time_duration frameTime(boost::posix_time::microsec(1000000 / fps));
    boost::posix_time::ptime lastShown = boost::posix_time::microsec_clock::universal_time() + frameTime;
    while (true) {
        //使用异常捕获interruption_point，立即停止播放线程，并释放资源
        //因为emit imageGot();使用BlockingQueuedConnection，
        // 为了避免和析构函数里面到join产生死锁，也必须马上跳出
        try {
            boost::this_thread::interruption_point();

            if (curFrame.image.format != AV_PIX_FMT_NONE) {
                av_freep(&curFrame.image.data[0]);
            }
            popFrame(curFrame);

            boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - lastShown;
            boost::this_thread::sleep(frameTime - diff);

            emit imageGot(); //直到图片显示完成后再返回
//            LOG(INFO) << "MediaStreamProc::play display frame done pts:" << curFrame.image.pts
//                      << " diff:" << (boost::posix_time::microsec_clock::universal_time() - lastShown).total_milliseconds()
//                      << " frameQueue:" << frameQueue.size();
            lastShown = boost::posix_time::microsec_clock::universal_time();
        } catch (boost::thread_interrupted &e) {
            LOG(INFO) << "MediaStreamProc::play interrupt";
            break;
        }
    }
}

MediaFrame *MediaStreamProc::getCurFrame() {
    return &curFrame;
}

void MediaStreamProc::decodeFrame() {

    int ret = 0;
    while (ret >= 0) {
        boost::this_thread::interruption_point();

        AVPacket packet;
        popPacket(packet);
        {
            boost::unique_lock<boost::mutex> lock(mtxStreamDecoderReady);
            while (!streamDecoderReady) {
                cvStreamDecoderReady.wait(lock);
            }
            if (video_stream == packet.stream_index) {
#ifdef VAAPI_ENABLE
                if (decoderType == DecoderType_Normal) {
                    decode_write_normal(decoder_ctx, &packet);
                } else if (decoderType == DecoderType_Vaapi) {
                    ret = decode_write_vaapi(decoder_ctx, &packet);
                }
#else
                decode_write_normal(decoder_ctx, &packet);
#endif
            }
        }
        av_packet_unref(&packet);
    }
}

void MediaStreamProc::pushPacket(const AVPacket &packet) {
    boost::unique_lock<boost::mutex> lock(mtxPacketsQuue);
    packetsQuue.push_back(packet);
    cvPacketsQuue.notify_one();
}

void MediaStreamProc::popPacket(AVPacket &packet) {
    boost::unique_lock<boost::mutex> lock(mtxPacketsQuue);
    while (packetsQuue.empty()) {
        cvPacketsQuue.wait(lock);
    }
    packet = packetsQuue.front();
    packetsQuue.pop_front();
}

bool MediaStreamProc::syncReadStream() {
    _readStream();
    return streamInputReady && streamDecoderReady;
}

int MediaStreamProc::normalInit() {
    int ret;
    char errorStr[1024] = {0};

    LOG(INFO) << "1.###################MediaStreamProc::normalInit avformat_open_input";
    if ((ret = avformat_open_input(&input_ctx, url.c_str(), nullptr, nullptr)) < 0) {
        goto error;
    }
    LOG(INFO) << "2.###################MediaStreamProc::normalInit avformat_find_stream_info";
    if ((ret = avformat_find_stream_info(input_ctx, nullptr)) < 0) {
        goto error;
    }
    /* select the video stream */
    LOG(INFO) << "3.###################MediaStreamProc::normalInit av_find_best_stream";
    ret = av_find_best_stream(input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder, 0);
    if (ret < 0) {
        goto error;
    }
    video_stream = ret;
    /* create decoding context */
    decoder_ctx = avcodec_alloc_context3(decoder);
    if (!decoder_ctx) {
        LOG(INFO) << "MediaStreamProc::normalInit avcodec_alloc_context3 error";
        ret = AVERROR(ENOMEM);
        return ret;
    }

    avcodec_parameters_to_context(decoder_ctx, input_ctx->streams[video_stream]->codecpar);
    av_opt_set_int(decoder_ctx, "refcounted_frames", 1, 0);
    /* init the video decoder */
    LOG(INFO) << "4.###################MediaStreamProc::normalInit avcodec_open2";
    if ((ret = avcodec_open2(decoder_ctx, decoder, nullptr)) < 0) {
        goto error;
    }

    streamInputReady = true;
    streamDecoderReady = true;
    LOG(INFO) << "MediaStreamProc::normalInit init video stream success!!!!";
    LOG(INFO) << "MediaStreamProc::normalInit decoder name:" << decoder_ctx->codec->name;
    return 0;

error:
    streamInputReady = false;
    streamDecoderReady = false;
    av_strerror(ret, errorStr, 1024);
    LOG(INFO) << "MediaStreamProc::normalInit url:" << url << " got an error:" << ret << " :" << errorStr;
    return ret;
}

int MediaStreamProc::decode_write_normal(AVCodecContext *avctx, AVPacket *packet) {
    int ret = 0;
    char errorStr[1024] = {0};
    AVFrame decoded_frame; // scaled_frame
    MediaFrame mediaFrame;
    static bool pushDecodedFrame = false;

    ret = avcodec_send_packet(avctx, packet);
    if (ret < 0) {
        av_strerror(ret, errorStr, 1024);
        LOG(INFO) << "MediaStreamProc::decode_write_normal avcodec_send_packet error:" << errorStr;
        return ret;
    }

    putAiInfo2MapFromFrame(*packet); //先将packet里面的所有框信息放入aiRoisMap中

    //删掉一些框数据,避免内存一直增长
    if (aiRoisMap.size() > aiRoisMapMaxSize) {
        int counter = 10;
        while (counter--) aiRoisMap.erase(aiRoisMap.begin());
    }

    while (ret >= 0) {
        AVFrame *frame = av_frame_alloc();
        ret = avcodec_receive_frame(avctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            av_frame_free(&frame);
            break;
        } else if (ret < 0) {
            av_frame_free(&frame);
            av_strerror(ret, errorStr, 1024);
            LOG(INFO) << "MediaStreamProc::decode_write_normal avcodec_receive_frame error:" << errorStr;
            return ret;
        }
        if (ret >= 0) {
            frame->pts = frame->best_effort_timestamp;

            AVPixelFormat format = (AVPixelFormat)frame->format;
            if (format == AV_PIX_FMT_YUVJ420P) {
                format = AV_PIX_FMT_YUV420P;
            }
            struct SwsContext *img_convert_ctx =
                    sws_getContext(frame->width, frame->height, format,
                                   frame->width, frame->height, AV_PIX_FMT_RGB32, SWS_BICUBIC, nullptr, nullptr, nullptr);
            frame_width = decoder_ctx->width; // H264, maybe frame->width is better
            frame_height = decoder_ctx->height;
            decoded_frame.format = AV_PIX_FMT_RGB32;
            decoded_frame.width = frame_width;
            decoded_frame.height = frame_height;
            if ((ret = av_image_alloc(decoded_frame.data, decoded_frame.linesize,
                                      frame_width, frame_height, AV_PIX_FMT_RGB32, 32)) < 0) {
                LOG(INFO) << "MediaStreamProc::decode_write_normal alloc decoded frame failed";
                ret = AVERROR(ENOMEM);
                sws_freeContext(img_convert_ctx);
                av_frame_free(&frame);
                return ret;
            }
            sws_scale(img_convert_ctx, (const uint8_t * const *)frame->data, frame->linesize, 0, frame->height,
                      decoded_frame.data, decoded_frame.linesize);

            decoded_frame.pts = frame->pts;

            sws_freeContext(img_convert_ctx);
        }
        av_frame_free(&frame);

//        av_freep(&decoded_frame.data[0]);
//        uint32_t rtpPts = static_cast<RTPDemuxContext*>(
//                static_cast<RTSPState*>(input_ctx->priv_data)->rtsp_streams[0]->transport_priv)->timestamp;
//        LOG(INFO) << "MediaStreamProc::decode_write_normal frame-pts:" << decoded_frame.pts
//                  << " queueSize:" << pendingAiRoiFrameQue.size() << " mapSize:" << aiRoisMap.size()
//                  << " pendingFrameCounter:" << pendingFrameCounter << " frameQueue:" << frameQueue.size();

        if (aiRoisMap.empty()) { //当前无框数据,不需要进行同步
            //当没有框数据时,为了避免累计太多frame,不进行匹配框的步骤
            mediaFrame.image = decoded_frame;
            mediaFrame.hasAiInfo = false;
            pushFrame(mediaFrame); //直接显示
            continue; //继续解码,不进行匹配框
        }

        pushDecodedFrame = true;

        //特殊处理
        //当有新的一帧解码图像时,并且pendingFrameCounter为0,同时pendingAiRoiFrameQue为空,直接push到pendingAiRoiFrameQue里面
        //否则到next_sync_roi时会直接跳转到: pendingAiRoiFrameQue中无数据,解码下一帧数据 的那个分支
        //永远无法进入正常的匹配框的过程
        if (0 == pendingFrameCounter && pendingAiRoiFrameQue.empty()) {
            pendingAiRoiFrameQue.push_back(decoded_frame);
            pushDecodedFrame = false;
        }

        std::map<uint64_t, MediaFrame_AI_Info>::iterator roisMapIt;
next_sync_roi:
        if (0 == pendingFrameCounter) {
            //pendingFrameCounter为0表示重新开始了一次新的找框循环
            //否则表明是为curSyncRoiFrame寻找框数据而读的AVPacket,所以curSyncRoiFrame保持不变
            if (!pendingAiRoiFrameQue.empty()) { //如果还有未同步的解码帧
                //如果不做限制,累积的frame会越来越多
                //为了避免内存被耗尽,当frame size到达drainPendingAiRoiFrameMax时
                //会直接显示drainPendingAiRoiFrameSize个无框的frame
                //必须在curSyncRoiFrame处理完成后再进行drain,否则会造成帧乱序,画面闪烁
                if (pendingAiRoiFrameQue.size() >= drainPendingAiRoiFrameMax) {
                    int counter = drainPendingAiRoiFrameSize;
                    while (!pendingAiRoiFrameQue.empty() && counter--) {
                        mediaFrame.image = pendingAiRoiFrameQue.front();
                        mediaFrame.hasAiInfo = false;
                        pendingAiRoiFrameQue.pop_front();

                        //尝试寻找ROI
                        roisMapIt = findRoisMapIt(mediaFrame.image.pts);
                        if ((roisMapIt != aiRoisMap.end())) {
                            mediaFrame.hasAiInfo = true;
                            mediaFrame.ai_info = roisMapIt->second;
//                            LOG(INFO) << "MediaStreamProc::decode_write_normal find roi while draining frame-pts:" << mediaFrame.image.pts;
                            aiRoisMap.erase(roisMapIt++); //将同步后的框删掉
                        }
                        pushFrame(mediaFrame);
//                        LOG(INFO) << "MediaStreamProc::decode_write_normal push find once roi frame draining........";
                    }
                }

                if (pendingAiRoiFrameQue.empty()) continue;

                //再每次从队列pendingAiRoiFrameQue前取出
                curSyncRoiFrame = pendingAiRoiFrameQue.front();
                pendingAiRoiFrameQue.pop_front();
//                LOG(INFO) << "MediaStreamProc::decode_write_normal pop pendingAiRoiFrameQue for frame pts:" << curSyncRoiFrame.pts
//                          << " queueSize:" << pendingAiRoiFrameQue.size() << " pendingFrameCounter:" << pendingFrameCounter;
            } else {
                continue; //pendingAiRoiFrameQue中无数据,解码下一帧数据
            }
        }

        memset(&mediaFrame, 0, sizeof(MediaFrame));
        roisMapIt = findRoisMapIt(curSyncRoiFrame.pts);

        if (roisMapIt != aiRoisMap.end()) { //已经找到当前frame的框数据
            mediaFrame.image = curSyncRoiFrame;
            mediaFrame.hasAiInfo = true;
            mediaFrame.ai_info = roisMapIt->second;
//            LOG(INFO) << "MediaStreamProc::decode_write_normal find rois for frame pts:"
//                      << curSyncRoiFrame.pts << " ai-pts:" << roisMapIt->second.u64TimeStamp
//                      << " queueSize:" << pendingAiRoiFrameQue.size() << " pendingFrameCounter:" << pendingFrameCounter;
            aiRoisMap.erase(roisMapIt); //将同步后的框删掉
            goto display_frame;
        } else { //未找到frame对应的框数据
            if (pendingAiRoiFrameQue.size() >= drainPendingAiRoiFrameMax) { //如果已经达到drainPendingAiRoiFrameMax
                //寻找当前帧的框,进行匹配
                mediaFrame.image = curSyncRoiFrame;
                mediaFrame.hasAiInfo = false;
                roisMapIt = findRoisMapIt(curSyncRoiFrame.pts); //尝试寻找ROI
                if ((roisMapIt != aiRoisMap.end())) {
                    mediaFrame.hasAiInfo = true;
                    mediaFrame.ai_info = roisMapIt->second;
                    LOG(INFO) << "MediaStreamProc::decode_write_normal find roi while draining frame-pts:" << curSyncRoiFrame.pts;
                    aiRoisMap.erase(roisMapIt++); //将同步后的框删掉
                }

                goto display_frame;
            }

            if (pendingFrameCounter < pendingFrameSize) { //未达到最大寻找包数目
                if (pushDecodedFrame) { //有新的解码帧才push
                    pendingAiRoiFrameQue.push_back(decoded_frame); //将解码帧放入等待同步帧队列
                    pushDecodedFrame = false;
                }
                ++pendingFrameCounter;
//                LOG(INFO) << "MediaStreamProc::decode_write_normal get another frame pts:"
//                          << decoded_frame.pts << " queueSize:" << pendingAiRoiFrameQue.size() << " pendingFrameCounter" << pendingFrameCounter;
                continue; //继续读取下一个AVPacket的roi数据或者解码下一个frame
            } else { //已经达到最大寻找包数目,还是没找到同步的框数据
                mediaFrame.hasAiInfo = false;
                mediaFrame.image = curSyncRoiFrame;
                LOG(INFO) << "MediaStreamProc::decode_write_normal can't find rois for frame pts:" << curSyncRoiFrame.pts
                          << " pendingAiRoiFrameQue:" << pendingAiRoiFrameQue.size() << " mapSize:" << aiRoisMap.size()
                          << " pendingFrameCounter:" << pendingFrameCounter;
                goto display_frame; //直接显示无框的frame
            }
        }

display_frame:
        pushFrame(mediaFrame); //将同步后的frame和框一起送到显示队列
        pendingFrameCounter = 0; //寻找包数目计数器置0

        //push本该放入的帧
        if (pushDecodedFrame) { //有新的解码帧才push
            pendingAiRoiFrameQue.push_back(decoded_frame); //将解码帧放入等待同步帧队列
            pushDecodedFrame = false;
        }
//        LOG(INFO) << "MediaStreamProc::decode_write_normal display_frame done pts:" << curSyncRoiFrame.pts
//                  << " pendingAiRoiFrameQue:" << pendingAiRoiFrameQue.size() << " mapSize:" << aiRoisMap.size()
//                  << " pendingFrameCounter:" << pendingFrameCounter;
        goto next_sync_roi; //继续找该解码帧的框数据

    }

    return 0;
}

void MediaStreamProc::putAiInfo2MapFromFrame(const AVPacket &packet) {
    if (packet.stream_index != video_stream) {
        return;
    }

    //查找0x00 0x00 0x00 0x01 0x0d，算法的NAL头
    //算法的NAL头是每一帧的最后一个
    for (int i = packet.size - 5; i >= 0; --i) {
        uint64_t nal_header = 0x0d01000000; //本地是小端，将字节反序
        uint64_t data;
        memset(&data, 0, 8);
        memcpy(&data, packet.data + i, 5);

        if (data == nal_header) {
            MediaFrame_AI_Info aiInfo;
            memcpy(&aiInfo, packet.data + i + 5, sizeof(MediaFrame_AI_Info));
            //由于FFmpeg的RTP时间戳转到AVPacket时间戳精度损失,最后2位数未使用
            if (!aiRoisMap.insert({aiInfo.u64TimeStamp, aiInfo}).second) {
                LOG(INFO) << "MediaStreamProc::putAiInfo2MapFromFrame insert failed key:" << aiInfo.u64TimeStamp << " exited";
            }
//            uint32_t rtpPts = static_cast<RTPDemuxContext *>(
//                    static_cast<RTSPState *>(input_ctx->priv_data)->rtsp_streams[0]->transport_priv)->timestamp;
//            LOG(INFO) << "MediaStreamProc::putAiInfo2MapFromFrame get ai-roi ai-pts:" << aiInfo.u64TimeStamp << " rtp-pts:" << rtpPts;
//            LOG(INFO) << "MediaStreamProc::putAiInfo2MapFromFrame get ai-roi ai-pts:" << aiInfo.u64TimeStamp
//                      << " ai-pts/100:" << aiInfo.u64TimeStamp / 100 << " pkt-pts:" << packet.pts  << " pkt-pts/100:" << packet.pts / 100;
        }
    }

    //    LOG(INFO) << "MediaStreamProc::decodeAiInfoFrame packet data:";
//    CHAR_BUFF_TO_LOG(std::vector<char>(packet.data, packet.data + packet.size));

    //0x00 0x00 0x00 0x01 0x0d
//        CHAR_BUFF_TO_LOG(std::vector<char>(packet.data, packet.data + packet.size));
//        LOG(INFO) << "size:" << packet.size;
//    for (int i = 0; i < packet.size - 4; ++i) {
//        uint64_t nal_header = 0x01000000; //本地是小端，将字节反序
//        uint64_t data;
//        memset(&data, 0, 8);
//        memcpy(&data, packet.data + i, 4);
//        if (data == nal_header) {
//            LOG(INFO) << "nal type:" << ((packet.data[i + 4]) & 0x1f);
//        }
//    }
//    LOG(INFO) << "package:" << "done##################";
}

std::map<uint64_t, MediaFrame_AI_Info>::iterator MediaStreamProc::findRoisMapIt(int64_t pts) {
   auto roisMapIt = aiRoisMap.lower_bound(pts);
    if (roisMapIt != aiRoisMap.begin() && roisMapIt != aiRoisMap.end()) {
        auto before = std::prev(roisMapIt);
        if ((roisMapIt->first - pts) > (pts - before->first)) {
            roisMapIt = before;
        }

        if (syncRoiDiffMax < std::abs(static_cast<int64_t>(roisMapIt->first - pts))) {
            roisMapIt = aiRoisMap.end(); //相差太大,赋值aiRoisMap.end()表示未找到
        }
    }

    return roisMapIt;
}
