#ifndef COMMLOG_H
#define COMMLOG_H
#include <glog/logging.h>
#include <glog/log_severity.h>
#include <boost/thread/mutex.hpp>

extern boost::mutex glog_flag_mtx;


#define STR_TO_LOG_FILE(str) \
do { \
    glog_flag_mtx.lock(); \
    int store = FLAGS_logtostderr; \
    FLAGS_logtostderr = 0; \
    LOG(INFO) << str;\
    FLAGS_logtostderr = store; \
    glog_flag_mtx.unlock(); \
} while(0);

#define STR_TO_LOG_STDERROR(str) \
do { \
    glog_flag_mtx.lock(); \
    int store = FLAGS_logtostderr; \
    FLAGS_logtostderr = 1; \
    LOG(INFO) << str;\
    FLAGS_logtostderr = store; \
    glog_flag_mtx.unlock(); \
} while(0);

//#define STR_TO_LOG_FILE(str) \
//do { \
//    google::SetLogDestination(google::INFO, "/tmp/remo_gui.test");\
//    LOG(INFO) << str;\
//    google::LogToStderr();\
//} while(0);

//#define STR_TO_LOG_STDERROR(str) \
//do { \
//    google::LogToStderr();\
//    LOG(INFO) << str;\
//} while(0);

//#define CHAR_BUFF_TO_LOG_FILE(buff) \
//do { \
//    char temp[64]{0}; \
//    std::string str; \
//    for (auto it : buff) { \
//        snprintf(temp, sizeof(temp), "0x%02hhx ", it); \
//        str += temp; \
//    } \
//    STR_TO_LOG_FILE(str); \
//} while(0);

//#define CHAR_BUFF_TO_LOG_STDERROR(buff) \
//do { \
//    char temp[64]{0}; \
//    std::string str; \
//    for (auto it : buff) { \
//        snprintf(temp, sizeof(temp), "0x%02hhx ", it); \
//        str += temp; \
//    } \
//    STR_TO_LOG_STDERROR(str); \
//} while(0);

#define CHAR_BUFF_TO_LOG(buff) \
    do { \
        char temp[64]{0}; \
        std::string str; \
        for (auto it : buff) { \
            snprintf(temp, sizeof(temp), "0x%02hhx ", it); \
            str += temp; \
        } \
        LOG(INFO) << str; \
    } while(0);
#endif // COMMLOG_H
