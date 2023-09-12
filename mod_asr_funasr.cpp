#include <switch.h>
#include <fstream>
#include <math.h>
#include "nlsClient.h"
#include "nlsEvent.h"
#include "speechTranscriberRequest.h"
#include "nlsCommonSdk/Token.h"
#include <sys/time.h>

#define ASIO_STANDALONE 1
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/config/asio_client.hpp>
#include <atomic>
#include <thread>

#include "audio.h"
#include "nlohmann/json.hpp"


#define MAX_FRAME_BUFFER_SIZE (1024*1024) //1MB
#define SAMPLE_RATE 8000

using namespace AlibabaNlsCommon;
using AlibabaNls::NlsClient;
using AlibabaNls::NlsEvent;
using AlibabaNls::LogDebug;
using AlibabaNls::SpeechTranscriberRequest;

struct AsrParamCallBack 
{
    std::string caller;
    std::string callee;
    char *sUUID ;
};

//======================================== ali asr start ===============
typedef struct 
{
    switch_core_session_t   *session;
    switch_media_bug_t      *bug;
    SpeechTranscriberRequest *request;
    int                     started;
    int                     stoped;
    int                     starting;
    int                     datalen;
    switch_mutex_t          *mutex;
    switch_memory_pool_t *pool;
    switch_audio_resampler_t *resampler;
} switch_da_t;

std::string g_appkey = "";
std::string g_akId = "";
std::string g_akSecret = "";
std::string g_token = "";
std::string g_nlsUrl="";
bool        g_debug = false;
long        g_expireTime = -1;
float       g_vol_multiplier = 1.0f;

SpeechTranscriberRequest* generateAsrRequest(AsrParamCallBack * cbParam);

//请求token
int generateToken(std::string akId, std::string akSecret, std::string* token, long* expireTime) 
{
    NlsToken nlsTokenRequest;
    nlsTokenRequest.setAccessKeyId(akId);
    nlsTokenRequest.setKeySecret(akSecret);
    //打印请求token的参数    
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "begin send generate token rquest: akId=%s, akSecret=%s\n", akId.c_str(), akSecret.c_str());
    int ret = nlsTokenRequest.applyNlsToken();
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "request success, status code=%d, token=%s, expireTime=%d, message=%s\n", ret, nlsTokenRequest.getToken(), nlsTokenRequest.getExpireTime(), nlsTokenRequest.getErrorMsg());
    if (ret < 0) 
    {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "generateToken Failed: %s\n", nlsTokenRequest.getErrorMsg());
        return -1;
    }
    *token = nlsTokenRequest.getToken();
    if (*token == "") 
    {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "generateToken Failed: token is '' \n");
        return -1;
    }
    *expireTime = nlsTokenRequest.getExpireTime();
    return 0;
}
/**
 * 识别启动回调函数
 * 
 * @param cbEvent 
 * @param cbParam 
 */
void onTranscriptionStarted(NlsEvent* cbEvent, void* cbParam) 
{
    AsrParamCallBack* tmpParam = (AsrParamCallBack*)cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionStarted: status code=%d, task id=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId());
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionStarted: all response=%s\n", cbEvent->getAllResponse());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if((pvt = (switch_da_t*)switch_channel_get_private(channel, "asr"))) 
        {
            switch_mutex_lock(pvt->mutex);
            pvt->started = 1;
            pvt->starting = 0;
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"I need lock!!!!!!!!!!!! \n"  );
            switch_mutex_unlock(pvt->mutex);
        }
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }
}
/**
 * @brief 一句话开始回调函数
 * 
 * @param cbEvent 
 * @param cbParam 
 */
void onAsrSentenceBegin(NlsEvent* cbEvent, void* cbParam) 
{
    AsrParamCallBack* tmpParam = (AsrParamCallBack*)cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceBegin: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceBegin: status code=%d, task id=%s, index=%d, time=%d\n", cbEvent->getStatusCode(), cbEvent->getTaskId(),
                    cbEvent->getSentenceIndex(),
                    cbEvent->getSentenceTime());
}

char *dupAsrResult(const char *allResponse) 
{
    const char *p = strstr(allResponse, "\"result\":\"");
    if (!p) {
        return strdup("");
    } 
    
    const char *begin = p + 10;
    const char *end = strchr(begin, '\"');

    if (!end) {
        return strdup("");
    }

    return strndup(begin, end - begin);
}

/**
 * @brief 一句话结束回调函数
 * 
 * @param cbEvent 
 * @param cbParam 
 */
void onAsrSentenceEnd(NlsEvent* cbEvent, void* cbParam) 
{
    AsrParamCallBack* tmpParam = (AsrParamCallBack*)cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceEnd: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceEnd: status code=%d, task id=%s, index=%d, time=%d, begin_time=%d, result=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId(),
                    cbEvent->getSentenceIndex(),
                    cbEvent->getSentenceTime(),
                    cbEvent->getSentenceBeginTime(),
                    cbEvent->getResult()
                    );
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrSentenceEnd: all response=%s\n", cbEvent->getAllResponse());
    switch_event_t *event = NULL;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if(switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) 
        {
            event->subclass_name = (char*)malloc(strlen("start_asr_") + strlen(tmpParam->sUUID) + 1);
            strcpy(event->subclass_name, "start_asr_");
            strcat(event->subclass_name, tmpParam->sUUID);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", tmpParam->sUUID);
    
            char *result = dupAsrResult(cbEvent->getAllResponse());
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Response", result);
            free(result);
    
            // switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Response", cbEvent->getAllResponse());
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Channel", switch_channel_get_name(channel));
            //switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Timestamp",currtime);
            //switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Answered",answered);
            switch_event_fire(&event);
        }
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }
}

/**
 * @brief 识别结果变化回调函数
 * 
 * @param cbEvent 
 * @param cbParam 
 */
void onAsrTranscriptionResultChanged(NlsEvent* cbEvent, void* cbParam) 
{
    AsrParamCallBack* tmpParam = (AsrParamCallBack*)cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionResultChanged: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionResultChanged: status code=%d, task id=%s, index=%d, time=%d, result=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId(),
                    cbEvent->getSentenceIndex(),
                    cbEvent->getSentenceTime(),
                    cbEvent->getResult()
                    );
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTranscriptionResultChanged: all response=%s\n", cbEvent->getAllResponse());
    switch_event_t *event = NULL;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) 
        {
            event->subclass_name = strdup("update_asr");
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", tmpParam->sUUID);
    
            char *result = dupAsrResult(cbEvent->getAllResponse());
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Response", result);
            free(result);
            // switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Response", cbEvent->getAllResponse());
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Channel", switch_channel_get_name(channel));
            //switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Timestamp",currtime);
            //switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Answered",answered);
            switch_event_fire(&event);
        }
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);    
    }
}
/**
 * @brief 语音转写结束回调函数
 * 
 * @param cbEvent 
 * @param cbParam 
 */
void onAsrTranscriptionCompleted(NlsEvent* cbEvent, void* cbParam) 
{
    AsrParamCallBack* tmpParam = (AsrParamCallBack*)cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionCompleted: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionCompleted: status code=%d, task id=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if((pvt = (switch_da_t*)switch_channel_get_private(channel, "asr"))) 
        {
            //        if(pvt->frameDataBuffer){
            //            free(pvt->frameDataBuffer);
            //        }
        }
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }
}
/**
 * @brief 异常识别回调函数
 * 
 * @param cbEvent 
 * @param cbParam 
 */
void onAsrTaskFailed(NlsEvent* cbEvent, void* cbParam) 
{
    AsrParamCallBack* tmpParam = (AsrParamCallBack*)cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTaskFailed: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTaskFailed: status code=%d, task id=%s, error message=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId(), cbEvent->getErrorMessage());
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTaskFailed: all response=%s\n", cbEvent->getAllResponse());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if((pvt = (switch_da_t*)switch_channel_get_private(channel, "asr"))) 
        {
            switch_mutex_lock(pvt->mutex);
            pvt->started = 0;
            switch_mutex_unlock(pvt->mutex);
        }
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }
}
/**
 * @brief 二次结果返回回调函数, 开启enable_nlp后返回
 * 
 * @param cbEvent 
 * @param cbParam 
 */
void onAsrSentenceSemantics(NlsEvent* cbEvent, void* cbParam) 
{
    AsrParamCallBack* tmpParam = (AsrParamCallBack*)cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceSemantics: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceSemantics: all response=%s\n", cbEvent->getAllResponse());
}
/**
 * @brief 识别通道关闭回调函数
 * 
 * @param cbEvent 
 * @param cbParam 
 */
void onAsrChannelClosed(NlsEvent* cbEvent, void* cbParam) 
{
    AsrParamCallBack* tmpParam = (AsrParamCallBack*)cbParam;
    switch_event_t *event = NULL;
    if(switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) 
    {
        event->subclass_name = strdup("stop_asr");
        switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
        switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Close", cbEvent->getResult());
        switch_event_fire(&event);
    }
    delete tmpParam;
}
/**
 * @brief asr请求构建
 * 
 * @param cbParam 
 * @return SpeechTranscriberRequest* 
 */
SpeechTranscriberRequest* generateAsrRequest(AsrParamCallBack * cbParam) 
{
    time_t now;
    time(&now);
    if (g_expireTime - now < 10) 
    {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "the token will be expired, please generate new token by AccessKey-ID and AccessKey-Secret\n");
        if (-1 == generateToken(g_akId, g_akSecret, &g_token, &g_expireTime)) 
        {
            return NULL;
        }
    }
    SpeechTranscriberRequest* request = NlsClient::getInstance()->createTranscriberRequest();
    if (request == NULL) 
    {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "createTranscriberRequest failed.\n" );
        return NULL;
    }
    request->setOnTranscriptionStarted(onTranscriptionStarted, cbParam);
    // 设置识别启动回调函数
    request->setOnTranscriptionResultChanged(onAsrTranscriptionResultChanged, cbParam);
    // 设置识别结果变化回调函数
    request->setOnTranscriptionCompleted(onAsrTranscriptionCompleted, cbParam);
    // 设置语音转写结束回调函数
    request->setOnSentenceBegin(onAsrSentenceBegin, cbParam);
    // 设置一句话开始回调函数
    request->setOnSentenceEnd(onAsrSentenceEnd, cbParam);
    // 设置一句话结束回调函数
    request->setOnTaskFailed(onAsrTaskFailed, cbParam);
    // 设置异常识别回调函数
    request->setOnChannelClosed(onAsrChannelClosed, cbParam);
    // 设置识别通道关闭回调函数
    request->setOnSentenceSemantics(onAsrSentenceSemantics, cbParam);
    //设置二次结果返回回调函数, 开启enable_nlp后返回
    request->setAppKey(g_appkey.c_str());
    // 设置AppKey, 必填参数, 请参照官网申请
    request->setUrl(g_nlsUrl.c_str());
    // 设置ASR 服务地址, 可使用公网 或 ECS 内网地址，具体参见: https://help.aliyun.com/document_detail/84428.html?spm=a2c4g.148847.0.0.1b704938UF5b6y
    request->setFormat("pcm");
    // 设置音频数据编码格式, 默认是pcm
    request->setSampleRate(SAMPLE_RATE);
    // 设置音频数据采样率, 可选参数，目前支持16000, 8000. 默认是16000
    request->setIntermediateResult(true);
    // 设置是否返回中间识别结果, 可选参数. 默认false
    request->setPunctuationPrediction(true);
    // 设置是否在后处理中添加标点, 可选参数. 默认false
    request->setInverseTextNormalization(true);
    // 设置是否在后处理中执行数字转写, 可选参数. 默认false
    request->setToken(g_token.c_str());
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "nls url is:%s, vol multiplier is:%f\n", g_nlsUrl.c_str(), g_vol_multiplier);
    return request;
}
//======================================== ali asr end ===============
//======================================== freeswitch module start ===============
SWITCH_MODULE_LOAD_FUNCTION(mod_asr_load);
SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_asr_shutdown);
extern "C" 
{
    SWITCH_MODULE_DEFINITION(mod_asr, mod_asr_load, mod_asr_shutdown, NULL);
}
;
/**
 * 配置加载 aliyun的appkey，akid，aksecret
 * 
 * @return switch_status_t 执行状态：
 */
static switch_status_t load_config() 
{
    const char *cf = "aliasr.conf";
    switch_xml_t cfg, xml, settings, param;
    if (!(xml = switch_xml_open_cfg(cf, &cfg, NULL))) 
    {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "Open of %s failed\n", cf);
        switch_xml_free(xml);
        return SWITCH_STATUS_TERM;
    }
    settings = switch_xml_child(cfg, "settings");
    if (!settings) 
    {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "No settings in asr config\n");
        switch_xml_free(xml);
        return SWITCH_STATUS_TERM;
    }
    for (param = switch_xml_child(settings, "param"); param; param = param->next) 
    {
        char *var = (char *) switch_xml_attr_soft(param, "name");
        char *val = (char *) switch_xml_attr_soft(param, "value");
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Read conf: %s = %s\n", var, val);
        //strcasecmp：忽略大小写比较字符串（二进制）
        if (!strcasecmp(var, "appkey")) 
        {
            g_appkey = val;
            continue;
        }
        if (!strcasecmp(var, "akid")) 
        {
            g_akId =  val;
            continue;
        }
        if (!strcasecmp(var, "aksecret")) 
        {
            g_akSecret=  val;
            continue;
        }
        if (!strcasecmp(var, "nlsurl")) 
        {
            g_nlsUrl=  val;
            continue;
        }
        if (!strcasecmp(var, "debug")) 
        {
            if (!strcasecmp(val, "true")) {
                g_debug = true;
            }
            continue;
        }
        if (!strcasecmp(var, "newdb")) 
        {
            double db = atof(val);
            g_vol_multiplier = pow(10,db/20);
            continue;
        }
    }
    return SWITCH_STATUS_SUCCESS;
}

void adjustVolume(int16_t *pcm, size_t pcmlen) {
    int32_t pcmval;
    for (size_t ctr = 0; ctr < pcmlen; ctr++) {
        pcmval = pcm[ctr] * g_vol_multiplier;
        if (pcmval < 32767 && pcmval > -32768) {
            pcm[ctr] = pcmval;
        } else if (pcmval > 32767) {
            pcm[ctr] = 32767;
        } else if (pcmval < -32768) {
            pcm[ctr] = -32768;
        }
    }
}

/**
 * asr 回调处理
 * 
 * @param bug 
 * @param user_data 
 * @param type 
 * @return switch_bool_t 
 */
static switch_bool_t asr_callback(switch_media_bug_t *bug, void *user_data, switch_abc_type_t type) 
{
    switch_da_t *pvt = (switch_da_t *)user_data;
    switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
    switch (type) 
    {
        case SWITCH_ABC_TYPE_INIT: 
        {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Channel Init:%s\n", switch_channel_get_name(channel));

            switch_codec_t *read_codec = switch_core_session_get_read_codec(pvt->session);
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "read_codec=[%s]!\n",read_codec->implementation->iananame);

            if (pvt->stoped ==1) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "SWITCH_ABC_TYPE_INIT: pvt->stoped\n");
                return SWITCH_TRUE;
            }

            switch_mutex_lock(pvt->mutex);
            if(pvt->started ==0 ) 
            {
                if(pvt->starting ==0) 
                {
                    pvt->starting = 1;
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Starting Transaction \n" );
                    AsrParamCallBack *cbParam  = new AsrParamCallBack;
                    cbParam->sUUID= switch_channel_get_uuid(channel);
                    switch_caller_profile_t  *profile = switch_channel_get_caller_profile(channel);
                    cbParam->caller = profile->caller_id_number;
                    cbParam->callee = profile->callee_id_number;
                    SpeechTranscriberRequest* request = generateAsrRequest(cbParam);
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Caller %s. Callee %s\n",cbParam->caller.c_str() , cbParam->callee.c_str() );
                    if(request == NULL) 
                    {
                        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "Asr Request init failed.%s\n", switch_channel_get_name(channel));
                        switch_mutex_unlock(pvt->mutex);
                        return SWITCH_TRUE;
                    }
                    pvt->request = request;
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Init SpeechTranscriberRequest.%s\n", switch_channel_get_name(channel));
                    if (pvt->request->start() < 0) 
                    {
                        pvt->stoped = 1;
                        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "start() failed. may be can not connect server. please check network or firewalld:%s\n", switch_channel_get_name(channel));
                        NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                        // start()失败，释放request对象
                    }
                }
            }
            switch_mutex_unlock(pvt->mutex);
        }
        break;
        case SWITCH_ABC_TYPE_CLOSE: 
        {
            if (pvt->request) 
            {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Stop Succeed channel: %s\n", switch_channel_get_name(channel));
                pvt->request->stop();
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr stoped:%s\n", switch_channel_get_name(channel));
                //7: 识别结束, 释放request对象
                NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr released:%s\n", switch_channel_get_name(channel));
            }
        }
        break;
        case SWITCH_ABC_TYPE_READ: 
        {
            uint8_t data[SWITCH_RECOMMENDED_BUFFER_SIZE];
            switch_frame_t frame = { 0 };
            frame.data = data;
            frame.buflen = sizeof(data);
            if (switch_core_media_bug_read(bug, &frame, SWITCH_FALSE) != SWITCH_STATUS_FALSE) 
            {
                switch_mutex_lock(pvt->mutex);
                //====== resample ==== ///
                switch_codec_implementation_t read_impl;
                memset(&read_impl, 0, sizeof(switch_codec_implementation_t));
                switch_core_session_get_read_impl(pvt->session, &read_impl);
                int datalen = frame.datalen;
                int16_t *dp = (int16_t *)frame.data;
                if (read_impl.actual_samples_per_second != 8000) 
                {
                    if (!pvt->resampler) 
                    {
                        if (switch_resample_create(&pvt->resampler,
                                                                           read_impl.actual_samples_per_second,
                                                                           8000,
                                                                           8 * (read_impl.microseconds_per_packet / 1000) * 2,
                                                                           SWITCH_RESAMPLE_QUALITY,
                                                                           1) != SWITCH_STATUS_SUCCESS) 
                        {
                            switch_mutex_unlock(pvt->mutex);
                            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "Unable to allocate resampler\n");
                            return SWITCH_FALSE;
                        }
                    }
                    switch_resample_process(pvt->resampler, dp, (int) datalen / 2 / 1);
                    memcpy(dp, pvt->resampler->to, pvt->resampler->to_len * 2 * 1);
                    int samples = pvt->resampler->to_len;
                    datalen = pvt->resampler->to_len * 2 * 1;
                    if (g_debug) {
                        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "ASR new samples:%d\n", samples);
                    }
                }
                if (g_vol_multiplier != 1.0f) {
                    adjustVolume((int16_t*)dp, (size_t)datalen / 2);
                }
                if (pvt->request->sendAudio((uint8_t *)dp, (size_t)datalen) <0) 
                {
                    pvt->stoped =1;
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "send audio failed:%s\n", switch_channel_get_name(channel));
                    pvt->request->stop();
                    NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                }
                if (g_debug) {
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "SWITCH_ABC_TYPE_READ: send audio %d\n", datalen);
                }
                switch_mutex_unlock(pvt->mutex);
            } else 
            {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "switch_core_media_bug_read failed\n");
            }
        }
        break;
        default:
                break;
    }
    return SWITCH_TRUE;
}
/**
 *  定义添加的函数
 */
SWITCH_STANDARD_APP(stop_asr_session_function) 
{
    switch_da_t *pvt;
    switch_channel_t *channel = switch_core_session_get_channel(session);
    if ((pvt = (switch_da_t*)switch_channel_get_private(channel, "asr"))) 
    {
        switch_channel_set_private(channel, "asr", NULL);
        switch_core_media_bug_remove(session, &pvt->bug);
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_DEBUG, "%s Stop ASR\n", switch_channel_get_name(channel));
    }
}
/**
 *  定义添加的函数
 * 
 *  注意：App函数是自带session的，Api中是没有的
 *       App函数中没有stream用于控制台输出的流；Api中是有的
 *       App函数不需要返回值；Api中是有的
 */
SWITCH_STANDARD_APP(start_asr_session_function) 
{
    switch_channel_t *channel = switch_core_session_get_channel(session);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Starting asr:%s\n", switch_channel_get_name(channel));
    switch_status_t status;
    switch_da_t *pvt;
    switch_codec_implementation_t read_impl;
    //memset是计算机中C/C++语言初始化函数。作用是将某一块内存中的内容全部设置为指定的值， 这个函数通常为新申请的内存做初始化工作。
    memset(&read_impl, 0, sizeof(switch_codec_implementation_t));
    //获取读媒体编码实现方法
    switch_core_session_get_read_impl(session, &read_impl);
    if (!(pvt = (switch_da_t*)switch_core_session_alloc(session, sizeof(switch_da_t)))) 
    {
        return;
    }
    pvt->started = 0;
    pvt->stoped = 0;
    pvt->starting = 0;
    pvt->datalen = 0;
    pvt->session = session;
    if ((status = switch_core_new_memory_pool(&pvt->pool)) != SWITCH_STATUS_SUCCESS) 
    {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Memory Error!\n");
        return;
    }
    switch_mutex_init(&pvt->mutex,SWITCH_MUTEX_NESTED,pvt->pool);
    //session添加media bug
    if ((status = switch_core_media_bug_add(session, "asr", NULL,
            asr_callback, pvt, 0, 
            // SMBF_READ_REPLACE | SMBF_WRITE_REPLACE |  SMBF_NO_PAUSE | SMBF_ONE_ONLY, 
    SMBF_READ_STREAM | SMBF_NO_PAUSE,
            &(pvt->bug))) != SWITCH_STATUS_SUCCESS) 
    {
        return;
    }
    switch_channel_set_private(channel, "asr", pvt);
    switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_INFO, "%s Start ASR\n", switch_channel_get_name(channel));
}
/**
 *  定义load函数，加载时运行
 */
SWITCH_MODULE_LOAD_FUNCTION(mod_asr_load) 
{
    if (load_config() != SWITCH_STATUS_SUCCESS) 
    {
        return SWITCH_STATUS_FALSE;
    }
    int ret = NlsClient::getInstance()->setLogConfig("log-transcriber", LogDebug);
    if (-1 == ret) 
    {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "set log failed\n");
        return SWITCH_STATUS_FALSE;
    }
    NlsClient::getInstance()->startWorkThread(4);
    switch_application_interface_t *app_interface;
    *module_interface = switch_loadable_module_create_module_interface(pool, modname);
    SWITCH_ADD_APP(app_interface, "start_asr", "asr", "asr",start_asr_session_function, "", SAF_MEDIA_TAP);
    SWITCH_ADD_APP(app_interface, "stop_asr", "asr", "asr", stop_asr_session_function, "", SAF_NONE);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " asr_load\n");
    return SWITCH_STATUS_SUCCESS;
}
/**
 *  定义shutdown函数，关闭时运行
 */
SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_asr_shutdown) 
{
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " asr_shutdown\n");
    return SWITCH_STATUS_SUCCESS;
}
