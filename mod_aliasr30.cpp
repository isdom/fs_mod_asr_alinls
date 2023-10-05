#include <switch.h>
#include <fstream>
#include <math.h>
#include "nlsClient.h"
#include "nlsEvent.h"
#include "speechTranscriberRequest.h"
#include "nlsCommonSdk/Token.h"
#include <sys/time.h>

// #include <libks/ks_thread_pool.h>

#define MAX_FRAME_BUFFER_SIZE (1024*1024) //1MB
#define SAMPLE_RATE 8000

using namespace AlibabaNlsCommon;
using AlibabaNls::NlsClient;
using AlibabaNls::NlsEvent;
using AlibabaNls::LogDebug;
using AlibabaNls::SpeechTranscriberRequest;

struct AsrParamCallBack {
    std::string caller;
    std::string callee;
    char *sUUID;
};

//======================================== ali asr start ===============
struct raw_pcm;

typedef struct raw_pcm {
    struct raw_pcm *_next;
    switch_time_t _from_answered;
    uint32_t _actual_samples_per_second;
    int _microseconds_per_packet;
    uint32_t _rawlen;
    uint8_t _rawdata[];
} raw_pcm_t;

#define _OFFSET_OF(m,y) (uint8_t*)(&((m*)0)->y)
const static int PCM_PAYLOAD_LEN = _OFFSET_OF(raw_pcm_t, _rawdata) - _OFFSET_OF(raw_pcm_t, _from_answered);

typedef struct {
    switch_core_session_t *session;
    switch_media_bug_t *bug = nullptr;
    SpeechTranscriberRequest *request = nullptr;
    int started;
    int stoped;
    int starting;
    int datalen;
    switch_mutex_t *mutex;
    switch_memory_pool_t *pool;
    switch_audio_resampler_t *resampler = nullptr;
    char *appkey = nullptr;
    char *nlsurl = nullptr;
    char *savepcm = nullptr;
    char *asr_dec_vol = nullptr;
    float vol_multiplier = 0.0f;
    raw_pcm_t *pcm_header = nullptr;
    raw_pcm_t *pcm_tail = nullptr;
} switch_da_t;

std::string g_akId = "";
std::string g_akSecret = "";
std::string g_token = "";
long g_expireTime = -1;
bool g_debug = false;

SpeechTranscriberRequest *generateAsrRequest(AsrParamCallBack *cbParam, switch_da_t *pvt);

/**
 * 根据AccessKey ID和AccessKey Secret重新生成一个token，
 * 并获取其有效期时间戳
 */
int generateToken(std::string akId, std::string akSecret, std::string *token, long *expireTime) {
    NlsToken nlsTokenRequest;
    nlsTokenRequest.setAccessKeyId(akId);
    nlsTokenRequest.setKeySecret(akSecret);
    //打印请求token的参数    
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "begin send generate token rquest: akId=%s, akSecret=%s\n",
                      akId.c_str(), akSecret.c_str());
    int ret = nlsTokenRequest.applyNlsToken();
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "request success, status code=%d, token=%s, expireTime=%d, message=%s\n", ret,
                      nlsTokenRequest.getToken(), nlsTokenRequest.getExpireTime(), nlsTokenRequest.getErrorMsg());
    if (ret < 0) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "generateToken Failed: %s\n",
                          nlsTokenRequest.getErrorMsg());
        return -1;
    }
    *token = nlsTokenRequest.getToken();
    if (token->empty()) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "generateToken Failed: token is '' \n");
        return -1;
    }
    *expireTime = nlsTokenRequest.getExpireTime();
    return 0;
}

/**
 * 识别启动回调函数
 * 
 * @brief 调用start(), 成功与云端建立连接, sdk内部线程上报started事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrTranscriptionStarted(NlsEvent *cbEvent, void *cbParam) {
    auto *tmpParam = (AsrParamCallBack *) cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: status code=%d, task id=%s\n",
                      cbEvent->getStatusCode(), cbEvent->getTaskId());
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: all response=%s\n",
                      cbEvent->getAllResponse());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if ((pvt = (switch_da_t *) switch_channel_get_private(channel, "asr"))) {
            switch_mutex_lock(pvt->mutex);
            pvt->started = 1;
            pvt->starting = 0;
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "I need lock!!!!!!!!!!!! \n");
            switch_mutex_unlock(pvt->mutex);
        }
        switch_event_t *event = nullptr;
        if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
            event->subclass_name = strdup("begin_asr");
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", tmpParam->sUUID);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Channel", switch_channel_get_name(channel));
            switch_event_fire(&event);
        }

        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }
}

/**
 * @brief 服务端检测到了一句话的开始, sdk内部线程上报SentenceBegin事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrSentenceBegin(NlsEvent *cbEvent, void *cbParam) {
    auto *tmpParam = (AsrParamCallBack *) cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceBegin: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrSentenceBegin: status code=%d, task id=%s, index=%d, time=%d\n",
                      cbEvent->getStatusCode(),
                      cbEvent->getTaskId(),
                      cbEvent->getSentenceIndex(),
                      cbEvent->getSentenceTime());
}

char *dupAsrResult(const char *allResponse) {
    const char *p = strstr(allResponse, R"("result":")");
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
 * @brief 服务端检测到了一句话结束, sdk内部线程上报SentenceEnd事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrSentenceEnd(NlsEvent *cbEvent, void *cbParam) {
    auto *tmpParam = (AsrParamCallBack *) cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceEnd: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrSentenceEnd: status code=%d, task id=%s, index=%d, time=%d, begin_time=%d, result=%s\n",
                      cbEvent->getStatusCode(), cbEvent->getTaskId(),
                      cbEvent->getSentenceIndex(),
                      cbEvent->getSentenceTime(),
                      cbEvent->getSentenceBeginTime(),
                      cbEvent->getResult()
    );
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrSentenceEnd: all response=%s\n",
                      cbEvent->getAllResponse());
    switch_event_t *event = nullptr;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
            event->subclass_name = (char *) malloc(strlen("start_asr_") + strlen(tmpParam->sUUID) + 1);
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
 * @brief 识别结果发生了变化, sdk在接收到云端返回到最新结果时,
 *        sdk内部线程上报ResultChanged事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrTranscriptionResultChanged(NlsEvent *cbEvent, void *cbParam) {
    auto *tmpParam = (AsrParamCallBack *) cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionResultChanged: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrTranscriptionResultChanged: status code=%d, task id=%s, index=%d, time=%d, result=%s\n",
                      cbEvent->getStatusCode(), cbEvent->getTaskId(),
                      cbEvent->getSentenceIndex(),
                      cbEvent->getSentenceTime(),
                      cbEvent->getResult()
    );
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTranscriptionResultChanged: all response=%s\n",
                      cbEvent->getAllResponse());
    switch_event_t *event = nullptr;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
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
 * @brief 服务端停止实时音频流识别时, sdk内部线程上报Completed事件
 * @note 上报Completed事件之后，SDK内部会关闭识别连接通道.
         此时调用sendAudio会返回负值, 请停止发送.
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void onAsrTranscriptionCompleted(NlsEvent *cbEvent, void *cbParam) {
    auto *tmpParam = (AsrParamCallBack *) cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionCompleted: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrTranscriptionCompleted: status code=%d, task id=%s\n", cbEvent->getStatusCode(),
                      cbEvent->getTaskId());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if ((pvt = (switch_da_t *) switch_channel_get_private(channel, "asr"))) {
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
 * @brief 识别过程(包含start(), sendAudio(), stop())发生异常时, sdk内部线程上报TaskFailed事件
 * @note 上报TaskFailed事件之后, SDK内部会关闭识别连接通道. 此时调用sendAudio会返回负值, 请停止发送
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void onAsrTaskFailed(NlsEvent *cbEvent, void *cbParam) {
    auto *tmpParam = (AsrParamCallBack *) cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTaskFailed: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrTaskFailed: status code=%d, task id=%s, error message=%s\n", cbEvent->getStatusCode(),
                      cbEvent->getTaskId(), cbEvent->getErrorMessage());
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTaskFailed: all response=%s\n",
                      cbEvent->getAllResponse());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(tmpParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if ((pvt = (switch_da_t *) switch_channel_get_private(channel, "asr"))) {
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
void onAsrSentenceSemantics(NlsEvent *cbEvent, void *cbParam) {
    auto *tmpParam = (AsrParamCallBack *) cbParam;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceSemantics: %s\n", tmpParam->sUUID);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceSemantics: all response=%s\n",
                      cbEvent->getAllResponse());
}

/**
 * @brief 识别结束或发生异常时，会关闭连接通道, sdk内部线程上报ChannelCloseed事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void onAsrChannelClosed(NlsEvent *cbEvent, void *cbParam) {
    auto *tmpParam = (AsrParamCallBack *) cbParam;
    switch_event_t *event = nullptr;
    if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
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
SpeechTranscriberRequest *generateAsrRequest(AsrParamCallBack *cbParam, switch_da_t *pvt) {
    time_t now;
    time(&now);
    if (g_expireTime - now < 10) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                          "the token will be expired, please generate new token by AccessKey-ID and AccessKey-Secret\n");
        if (-1 == generateToken(g_akId, g_akSecret, &g_token, &g_expireTime)) {
            return nullptr;
        }
    }
    SpeechTranscriberRequest *request = NlsClient::getInstance()->createTranscriberRequest();
    if (request == nullptr) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "createTranscriberRequest failed.\n");
        return nullptr;
    }
    request->setOnTranscriptionStarted(onAsrTranscriptionStarted, cbParam);
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
    request->setAppKey(pvt->appkey);
    // 设置AppKey, 必填参数, 请参照官网申请
    request->setUrl(pvt->nlsurl);
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
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "nls url is:%s, vol multiplier is:%f\n",
                      pvt->nlsurl, pvt->vol_multiplier);
    return request;
}
//======================================== ali asr end ===============
//======================================== freeswitch module start ===============
SWITCH_MODULE_LOAD_FUNCTION(mod_aliasr_load);

SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_aliasr_shutdown);

extern "C"
{
SWITCH_MODULE_DEFINITION(mod_aliasr, mod_aliasr_load, mod_aliasr_shutdown, NULL);
};

/**
 * 配置加载 aliyun的appkey，akid，aksecret
 * 
 * @return switch_status_t 执行状态：
 */
static switch_status_t load_config() {
    const char *cf = "aliasr.conf";
    switch_xml_t cfg, xml, settings, param;
    if (!(xml = switch_xml_open_cfg(cf, &cfg, nullptr))) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "Open of %s failed\n", cf);
        switch_xml_free(xml);
        return SWITCH_STATUS_TERM;
    }
    settings = switch_xml_child(cfg, "settings");
    if (!settings) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "No settings in asr config\n");
        switch_xml_free(xml);
        return SWITCH_STATUS_TERM;
    }
    for (param = switch_xml_child(settings, "param"); param; param = param->next) {
        char *var = (char *) switch_xml_attr_soft(param, "name");
        char *val = (char *) switch_xml_attr_soft(param, "value");
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Read conf: %s = %s\n", var, val);
        //strcasecmp：忽略大小写比较字符串（二进制）
        if (!strcasecmp(var, "akid")) {
            g_akId = val;
            continue;
        }
        if (!strcasecmp(var, "aksecret")) {
            g_akSecret = val;
            continue;
        }
        if (!strcasecmp(var, "debug")) {
            if (!strcasecmp(val, "true")) {
                g_debug = true;
            }
            continue;
        }
    }
    return SWITCH_STATUS_SUCCESS;
}

void adjustVolume(int16_t *pcm, size_t pcmlen, float vol_multiplier) {
    int32_t pcmval;
    for (size_t ctr = 0; ctr < pcmlen; ctr++) {
        pcmval = pcm[ctr] * vol_multiplier;
        if (pcmval < 32767 && pcmval > -32768) {
            pcm[ctr] = pcmval;
        } else if (pcmval > 32767) {
            pcm[ctr] = 32767;
        } else if (pcmval < -32768) {
            pcm[ctr] = -32768;
        }
    }
}

static void append_current_pcm(switch_da_t *pvt,
                               switch_channel_t *channel,
                               const switch_frame_t &frame,
                               const switch_codec_implementation_t &read_impl) {
    switch_channel_timetable_t *times = switch_channel_get_timetable(channel);

    // channel->caller_profile->times->answered = switch_micro_time_now();
// https://github.com/signalwire/freeswitch/blob/792eee44d0611422cce3c3194f95125916a7d268/src/switch_channel.c#L3834C3-L3834C70
    auto pcm = (raw_pcm_t*)malloc(sizeof(raw_pcm_t) + frame.datalen);
    pcm->_next = nullptr;
    pcm->_from_answered = switch_micro_time_now() - times->answered;
    pcm->_actual_samples_per_second = read_impl.actual_samples_per_second;
    pcm->_microseconds_per_packet = read_impl.microseconds_per_packet;
    pcm->_rawlen = frame.datalen;
    memcpy(pcm->_rawdata, frame.data, pcm->_rawlen);
    if (!pvt->pcm_header) {
        pvt->pcm_tail = pvt->pcm_header = pcm;
    } else {
        pvt->pcm_tail->_next = pcm;
        pvt->pcm_tail = pcm;
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
static switch_bool_t asr_callback(switch_media_bug_t *bug, void *user_data, switch_abc_type_t type) {
    auto *pvt = (switch_da_t *) user_data;
    switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
    switch (type) {
        case SWITCH_ABC_TYPE_INIT: {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Channel Init:%s\n",
                              switch_channel_get_name(channel));

            switch_codec_t *read_codec = switch_core_session_get_read_codec(pvt->session);
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "read_codec=[%s]!\n",
                              read_codec->implementation->iananame);

            if (pvt->stoped == 1) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "SWITCH_ABC_TYPE_INIT: pvt->stoped\n");
                return SWITCH_TRUE;
            }

            switch_mutex_lock(pvt->mutex);
            if (pvt->started == 0) {
                if (pvt->starting == 0) {
                    pvt->starting = 1;
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Starting Transaction \n");
                    auto *cbParam = new AsrParamCallBack;
                    cbParam->sUUID = switch_channel_get_uuid(channel);
                    switch_caller_profile_t *profile = switch_channel_get_caller_profile(channel);
                    cbParam->caller = profile->caller_id_number;
                    cbParam->callee = profile->callee_id_number;
                    SpeechTranscriberRequest *request = generateAsrRequest(cbParam, pvt);
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Caller %s. Callee %s\n",
                                      cbParam->caller.c_str(), cbParam->callee.c_str());
                    if (!request) {
                        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "Asr Request init failed.%s\n",
                                          switch_channel_get_name(channel));
                        switch_mutex_unlock(pvt->mutex);
                        return SWITCH_TRUE;
                    }
                    pvt->request = request;
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Init SpeechTranscriberRequest.%s\n",
                                      switch_channel_get_name(channel));
                    if (pvt->request->start() < 0) {
                        pvt->stoped = 1;
                        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                                          "start() failed. may be can not connect server. please check network or firewalld:%s\n",
                                          switch_channel_get_name(channel));
                        NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                        // start()失败，释放request对象
                    }
                }
            }
            switch_mutex_unlock(pvt->mutex);
        }
            break;
        case SWITCH_ABC_TYPE_CLOSE: {
            if (pvt->request) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Stop Succeed channel: %s\n",
                                  switch_channel_get_name(channel));
                pvt->request->stop();
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr stoped:%s\n",
                                  switch_channel_get_name(channel));
                //7: 识别结束, 释放request对象
                NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr released:%s\n",
                                  switch_channel_get_name(channel));
            }
        }
            break;
        case SWITCH_ABC_TYPE_READ: {
            uint8_t data[SWITCH_RECOMMENDED_BUFFER_SIZE];
            switch_frame_t frame = {0};
            frame.data = data;
            frame.buflen = sizeof(data);
            if (switch_core_media_bug_read(bug, &frame, SWITCH_FALSE) != SWITCH_STATUS_FALSE) {
                switch_mutex_lock(pvt->mutex);
                //====== resample ==== ///
                // TODO, move to ASR start,bcs switch_codec_implementation_t will not change inside session

                switch_codec_implementation_t read_impl;
                memset(&read_impl, 0, sizeof(switch_codec_implementation_t));
                switch_core_session_get_read_impl(pvt->session, &read_impl);

                if (pvt->savepcm) {
                    append_current_pcm(pvt, channel, frame, read_impl);
                }

                int datalen = frame.datalen;
                int16_t *dp = (int16_t *) frame.data;
                if (read_impl.actual_samples_per_second != 8000) {
                    if (!pvt->resampler) {
                        if (switch_resample_create(&pvt->resampler,
                                                   read_impl.actual_samples_per_second,
                                                   8000,
                                                   8 * (read_impl.microseconds_per_packet / 1000) * 2,
                                                   SWITCH_RESAMPLE_QUALITY,
                                                   1) != SWITCH_STATUS_SUCCESS) {
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
                if (pvt->asr_dec_vol) {
                    adjustVolume((int16_t *) dp, (size_t) datalen / 2, pvt->vol_multiplier);
                }
                if (pvt->request->sendAudio((uint8_t *) dp, (size_t) datalen) < 0) {
                    pvt->stoped = 1;
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "send audio failed:%s\n",
                                      switch_channel_get_name(channel));
                    pvt->request->stop();
                    NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                }
                if (g_debug) {
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "SWITCH_ABC_TYPE_READ: send audio %d\n",
                                      datalen);
                }
                switch_mutex_unlock(pvt->mutex);
            } else {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "switch_core_media_bug_read failed\n");
            }
        }
            break;
        default:
            break;
    }
    return SWITCH_TRUE;
}

void save_pcm_to(switch_da_t *pvt, const char *filename) {
    FILE *output = fopen(filename, "wb");
    if (output) {
        raw_pcm_t *pcm = pvt->pcm_header;
        while (pcm) {
            fwrite(&(pcm->_from_answered), PCM_PAYLOAD_LEN + pcm->_rawlen, 1, output);
            pcm = pcm->_next;
        }
        fclose(output);
    } else {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "failed to fopen %s\n", filename);
    }
}

void load_pcm_from(switch_da_t *pvt, const char *filename) {
    FILE *input = fopen(filename, "rb");
    if (input) {
        while(!feof(input)) {
            raw_pcm_t hdr;

            if (fread(&(hdr._from_answered), PCM_PAYLOAD_LEN, 1, input) <= 0) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "read raw_pcm_t hdr failed\n");
            } else {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "read raw_pcm_t hdr, _rawlen: %d\n",
                                  hdr._rawlen);
                auto pcm = (raw_pcm_t*)malloc(sizeof(raw_pcm_t) + hdr._rawlen);
                pcm->_next = nullptr;
                pcm->_from_answered = hdr._from_answered;
                pcm->_actual_samples_per_second = hdr._actual_samples_per_second;
                pcm->_microseconds_per_packet = hdr._microseconds_per_packet;
                pcm->_rawlen = hdr._rawlen;
                if (fread(pcm->_rawdata, hdr._rawlen, 1, input) <= 0) {
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "read _rawdata(%d) failed\n",
                                      hdr._rawlen);
                } else {
                    if (!pvt->pcm_header) {
                        pvt->pcm_tail = pvt->pcm_header = pcm;
                    } else {
                        pvt->pcm_tail->_next = pcm;
                        pvt->pcm_tail = pcm;
                    }
                }
            }
        }

        fclose(input);
    } else {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "failed to fopen %s\n", filename);
    }
}

void release_pcm(switch_da_t *pvt) {
    raw_pcm_t *pcm = pvt->pcm_header;
    while (pcm) {
        raw_pcm_t *pcm_next = pcm->_next;
        free(pcm);
        pcm = pcm_next;
    }
    pvt->pcm_header = pvt->pcm_tail = nullptr;
}

int count_pcm(switch_da_t *pvt) {
    int count = 0;
    raw_pcm_t *pcm = pvt->pcm_header;
    while (pcm) {
        count++;
        pcm = pcm->_next;
    }
    return count;
}

static void *SWITCH_THREAD_FUNC replay_thread(switch_thread_t *thread, void *obj) {

    auto pvt = (switch_da_t *) obj;
    switch_channel_t *channel = switch_core_session_get_channel(pvt->session);

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "replay thread started for %s\n",
                      switch_channel_get_name(channel));

    switch_channel_timetable_t *times = switch_channel_get_timetable(channel);
    raw_pcm_t *current = pvt->pcm_header;

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Channel Init:%s\n",
                      switch_channel_get_name(channel));

    switch_codec_t *read_codec = switch_core_session_get_read_codec(pvt->session);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "read_codec=[%s]!\n",
                      read_codec->implementation->iananame);

    switch_mutex_lock(pvt->mutex);
    if (pvt->started == 0) {
        if (pvt->starting == 0) {
            pvt->starting = 1;
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Starting Transaction \n");
            auto *cbParam = new AsrParamCallBack;
            cbParam->sUUID = switch_channel_get_uuid(channel);
            switch_caller_profile_t *profile = switch_channel_get_caller_profile(channel);
            cbParam->caller = profile->caller_id_number;
            cbParam->callee = profile->callee_id_number;
            SpeechTranscriberRequest *request = generateAsrRequest(cbParam, pvt);
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Caller %s. Callee %s\n",
                              cbParam->caller.c_str(), cbParam->callee.c_str());
            if (!request) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "Asr Request init failed.%s\n",
                                  switch_channel_get_name(channel));
                switch_mutex_unlock(pvt->mutex);
                goto end;
            }
            pvt->request = request;
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Init SpeechTranscriberRequest.%s\n",
                              switch_channel_get_name(channel));
            if (pvt->request->start() < 0) {
                pvt->stoped = 1;
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                                  "start() failed. may be can not connect server. please check network or firewalld:%s\n",
                                  switch_channel_get_name(channel));
                switch_mutex_unlock(pvt->mutex);
                goto end;
            }
        }
    }
    switch_mutex_unlock(pvt->mutex);
    int idx = 0;
    while (current) {
        switch_time_t duration = switch_micro_time_now() - times->answered;
        if (current->_from_answered <= duration) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                              "(%d)-> org:%ld, replay:%ld\n", idx++, duration, current->_from_answered);
            // replay to asr
            switch_mutex_lock(pvt->mutex);

            //====== resample ==== ///
            if (current->_actual_samples_per_second != 8000) {
                if (!pvt->resampler) {
                    if (switch_resample_create(&pvt->resampler,
                                               current->_actual_samples_per_second,
                                               8000,
                                               8 * (current->_microseconds_per_packet / 1000) * 2,
                                               SWITCH_RESAMPLE_QUALITY,
                                               1) != SWITCH_STATUS_SUCCESS) {
                        switch_mutex_unlock(pvt->mutex);
                        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "Unable to allocate resampler\n");
                        goto end;
                    }
                }
                switch_resample_process(pvt->resampler, (int16_t *)current->_rawdata, (int) current->_rawlen / 2 / 1);
                memcpy(current->_rawdata, pvt->resampler->to, pvt->resampler->to_len * 2 * 1);
                int samples = pvt->resampler->to_len;
                current->_rawlen = pvt->resampler->to_len * 2 * 1;
                if (g_debug) {
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "ASR new samples:%d\n", samples);
                }
            }
            if (pvt->asr_dec_vol) {
                adjustVolume((int16_t *) current->_rawdata, (size_t) current->_rawlen / 2, pvt->vol_multiplier);
            }
            if (pvt->request->sendAudio(current->_rawdata, (size_t) current->_rawlen) < 0) {
                pvt->stoped = 1;
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "send audio failed:%s\n",
                                  switch_channel_get_name(channel));
                switch_mutex_unlock(pvt->mutex);
                goto end;
            }
            if (g_debug) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "SWITCH_ABC_TYPE_READ: send audio %d\n",
                                  current->_rawlen);
            }
            switch_mutex_unlock(pvt->mutex);
        } else {
            switch_yield(10 * 1000);
        }
        current = current->_next;
    }

    end:
    if (pvt->request) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Stop Succeed channel: %s\n",
                          switch_channel_get_name(channel));
        pvt->request->stop();
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr stoped:%s\n",
                          switch_channel_get_name(channel));
        //7: 识别结束, 释放request对象
        NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr released:%s\n",
                          switch_channel_get_name(channel));
    }

    switch_channel_hangup(channel, SWITCH_CAUSE_NONE);

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "replay thread ended and hangup for %s\n",
                      switch_channel_get_name(channel));
    // release pcm to avoid save pcm data again
    release_pcm(pvt);
    return nullptr;
}

switch_status_t on_channel_destroy(switch_core_session_t *session) {
    switch_da_t *pvt;
    switch_channel_t *channel = switch_core_session_get_channel(session);
    switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                      "%s on_destroy, release all resource for session\n",
                      switch_channel_get_name(channel));

    if ((pvt = (switch_da_t *) switch_channel_get_private(channel, "asr"))) {
        switch_channel_set_private(channel, "asr", nullptr);
        if (pvt->resampler) {
            switch_resample_destroy(&pvt->resampler);
            switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                              "%s on_destroy: switch_resample_destroy\n",
                              switch_channel_get_name(channel));
        }
        switch_mutex_destroy(pvt->mutex);
        switch_core_destroy_memory_pool(&pvt->pool);
        if (pvt->pcm_header) {
            // try to save pcms
            if (pvt->savepcm) {
                save_pcm_to(pvt, pvt->savepcm);
            }
            // then destroy pcms
            release_pcm(pvt);
        }
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                          "%s on_destroy: switch_mutex_destroy & switch_core_destroy_memory_pool\n",
                          switch_channel_get_name(channel));
    }
    return SWITCH_STATUS_SUCCESS;
}

switch_state_handler_table_t asr_cs_handlers = {
        /*! executed when the state changes to init */
        // switch_state_handler_t on_init;
        nullptr,
        /*! executed when the state changes to routing */
        // switch_state_handler_t on_routing;
        nullptr,
        /*! executed when the state changes to execute */
        // switch_state_handler_t on_execute;
        nullptr,
        /*! executed when the state changes to hangup */
        // switch_state_handler_t on_hangup;
        nullptr,
        /*! executed when the state changes to exchange_media */
        // switch_state_handler_t on_exchange_media;
        nullptr,
        /*! executed when the state changes to soft_execute */
        // switch_state_handler_t on_soft_execute;
        nullptr,
        /*! executed when the state changes to consume_media */
        // switch_state_handler_t on_consume_media;
        nullptr,
        /*! executed when the state changes to hibernate */
        // switch_state_handler_t on_hibernate;
        nullptr,
        /*! executed when the state changes to reset */
        // switch_state_handler_t on_reset;
        nullptr,
        /*! executed when the state changes to park */
        // switch_state_handler_t on_park;
        nullptr,
        /*! executed when the state changes to reporting */
        // switch_state_handler_t on_reporting;
        nullptr,
        /*! executed when the state changes to destroy */
        // switch_state_handler_t on_destroy;
        on_channel_destroy,
        // int flags;
        0
};

#define MAX_API_ARGC 5

// uuid_replay_aliasr <uuid> appkey=<appkey> nls=<nlsurl> loadpcm=<local path> debug=<true/false>
SWITCH_STANDARD_API(uuid_replay_aliasr_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "uuid_replay_aliasr: parameter missing.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_status_t status = SWITCH_STATUS_SUCCESS;
    switch_core_session_t *ses = nullptr;
    char *_appkey = nullptr;
    char *_nlsurl = nullptr;
    char *_asr_dec_vol = nullptr;
    char *_loadpcm = nullptr;
//    bool        _debug = false;

    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *mycmd = switch_core_strdup(pool, cmd);

    char *argv[MAX_API_ARGC];
    memset(argv, 0, sizeof(char *) * MAX_API_ARGC);

    int argc = switch_split(mycmd, ' ', argv);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", mycmd, argc);

    if (argc < 1) {
        stream->write_function(stream, "uuid is required.\n");
        switch_goto_status(SWITCH_STATUS_SUCCESS, end);
    }

    for (int idx = 1; idx < MAX_API_ARGC; idx++) {
        if (argv[idx]) {
            char *ss[2] = {0, 0};
            int cnt = switch_split(argv[idx], '=', ss);
            if (cnt == 2) {
                char *var = ss[0];
                char *val = ss[1];
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "process arg: %s = %s\n", var, val);
                if (!strcasecmp(var, "appkey")) {
                    _appkey = val;
                    continue;
                }
                if (!strcasecmp(var, "nls")) {
                    _nlsurl = val;
                    continue;
                }
//                if (!strcasecmp(var, "debug")) {
//                    if (!strcasecmp(val, "true")) {
//                        _debug = true;
//                    }
//                    continue;
//                }
                if (!strcasecmp(var, "asr_dec_vol")) {
                    _asr_dec_vol = val;
                    continue;
                }
                if (!strcasecmp(var, "loadpcm")) {
                    _loadpcm = val;
                    continue;
                }
            }
        }
    }

    if (!_appkey || !_nlsurl || !_loadpcm) {
        stream->write_function(stream, "appkey/nls/loadpcm are required.\n");
        switch_goto_status(SWITCH_STATUS_SUCCESS, end);
    }

    ses = switch_core_session_force_locate(argv[0]);
    if (!ses) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "replay to aliasr failed, can't found session by %s\n",
                          argv[0]);
    } else {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "starting replay to aliasr:%s\n",
                          switch_channel_get_name(channel));

        switch_thread_t *thread = nullptr;
        switch_threadattr_t *thd_attr = nullptr;
        switch_da_t *pvt;
        if (!(pvt = (switch_da_t *) switch_core_session_alloc(ses, sizeof(switch_da_t)))) {
            switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
        }
        pvt->started = 0;
        pvt->stoped = 0;
        pvt->starting = 0;
        pvt->datalen = 0;
        pvt->session = ses;
        pvt->appkey = switch_core_session_strdup(ses, _appkey);
        pvt->nlsurl = switch_core_session_strdup(ses, _nlsurl);
        pvt->asr_dec_vol = _asr_dec_vol ? switch_core_session_strdup(ses, _asr_dec_vol) : nullptr;
        if (pvt->asr_dec_vol) {
            double db = atof(pvt->asr_dec_vol);
            pvt->vol_multiplier = pow(10, db / 20);
        }
        if ((status = switch_core_new_memory_pool(&pvt->pool)) != SWITCH_STATUS_SUCCESS) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Memory Error!\n");
            switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
        }
        switch_mutex_init(&pvt->mutex, SWITCH_MUTEX_NESTED, pvt->pool);
        switch_channel_set_private(channel, "asr", pvt);

        // hook cs state change
        if (switch_channel_add_state_handler(channel, &asr_cs_handlers) < 0) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "hook cs state change failed!\n");
        }

        load_pcm_from(pvt, _loadpcm);

        switch_threadattr_create(&thd_attr, pvt->pool);
        switch_threadattr_stacksize_set(thd_attr, SWITCH_THREAD_STACKSIZE);

        switch_thread_create(&thread, thd_attr, replay_thread, pvt, pvt->pool);

        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(ses), SWITCH_LOG_INFO, "%s Start Replay To ASR\n",
                          switch_channel_get_name(channel));
        unlock:
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }

    end:
    switch_core_destroy_memory_pool(&pool);
    return status;
}

// uuid_start_aliasr <uuid> appkey=<appkey> nls=<nlsurl> debug=<true/false> savepcm=<local path>
SWITCH_STANDARD_API(uuid_start_aliasr_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "uuid_start_aliasr: parameter missing.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_status_t status = SWITCH_STATUS_SUCCESS;
    switch_core_session_t *ses = nullptr;
    char *_appkey = nullptr;
    char *_nlsurl = nullptr;
    char *_asr_dec_vol = nullptr;
    char *_savepcm = nullptr;
//    bool        _debug = false;

    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *mycmd = switch_core_strdup(pool, cmd);

    char *argv[MAX_API_ARGC];
    memset(argv, 0, sizeof(char *) * MAX_API_ARGC);

    int argc = switch_split(mycmd, ' ', argv);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", mycmd, argc);

    if (argc < 1) {
        stream->write_function(stream, "uuid is required.\n");
        switch_goto_status(SWITCH_STATUS_SUCCESS, end);
    }

    for (int idx = 1; idx < MAX_API_ARGC; idx++) {
        if (argv[idx]) {
            char *ss[2] = {0, 0};
            int cnt = switch_split(argv[idx], '=', ss);
            if (cnt == 2) {
                char *var = ss[0];
                char *val = ss[1];
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "process arg: %s = %s\n", var, val);
                //strcasecmp：忽略大小写比较字符串（二进制）
                if (!strcasecmp(var, "appkey")) {
                    _appkey = val;
                    continue;
                }
                if (!strcasecmp(var, "nls")) {
                    _nlsurl = val;
                    continue;
                }
//                if (!strcasecmp(var, "debug")) {
//                    if (!strcasecmp(val, "true")) {
//                        _debug = true;
//                    }
//                    continue;
//                }
                if (!strcasecmp(var, "asr_dec_vol")) {
                    _asr_dec_vol = val;
                    continue;
                }
                if (!strcasecmp(var, "savepcm")) {
                    _savepcm = val;
                    continue;
                }
            }
        }
    }

    if (!_appkey || !_nlsurl) {
        stream->write_function(stream, "appkey and nls are required.\n");
        switch_goto_status(SWITCH_STATUS_SUCCESS, end);
    }

    ses = switch_core_session_force_locate(argv[0]);
    if (!ses) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "start aliasr failed, can't found session by %s\n",
                          argv[0]);
    } else {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "starting aliasr:%s\n",
                          switch_channel_get_name(channel));

        switch_da_t *pvt;
        if (!(pvt = (switch_da_t *) switch_core_session_alloc(ses, sizeof(switch_da_t)))) {
            switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
        }
        pvt->started = 0;
        pvt->stoped = 0;
        pvt->starting = 0;
        pvt->datalen = 0;
        pvt->session = ses;
        pvt->appkey = switch_core_session_strdup(ses, _appkey);
        pvt->nlsurl = switch_core_session_strdup(ses, _nlsurl);
        pvt->savepcm = _savepcm ? switch_core_session_strdup(ses, _savepcm) : nullptr;
        pvt->asr_dec_vol = _asr_dec_vol ? switch_core_session_strdup(ses, _asr_dec_vol) : nullptr;
        if (pvt->asr_dec_vol) {
            double db = atof(pvt->asr_dec_vol);
            pvt->vol_multiplier = pow(10, db / 20);
        }
        if ((status = switch_core_new_memory_pool(&pvt->pool)) != SWITCH_STATUS_SUCCESS) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Memory Error!\n");
            switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
        }
        switch_mutex_init(&pvt->mutex, SWITCH_MUTEX_NESTED, pvt->pool);
        //session添加media bug
        if ((status = switch_core_media_bug_add(ses, "asr", NULL,
                                                asr_callback, pvt, 0,
                // SMBF_READ_REPLACE | SMBF_WRITE_REPLACE |  SMBF_NO_PAUSE | SMBF_ONE_ONLY,
                                                SMBF_READ_STREAM | SMBF_NO_PAUSE,
                                                &(pvt->bug))) != SWITCH_STATUS_SUCCESS) {
            switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
        }
        switch_channel_set_private(channel, "asr", pvt);

        if (switch_channel_add_state_handler(channel, &asr_cs_handlers) < 0) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "hook cs state change failed!\n");
        } // hook cs state change

        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(ses), SWITCH_LOG_INFO, "%s Start ASR\n",
                          switch_channel_get_name(channel));
        unlock:
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }

    end:
    switch_core_destroy_memory_pool(&pool);
    return status;
}

// uuid_stop_aliasr <uuid>
SWITCH_STANDARD_API(uuid_stop_aliasr_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "uuid_stop_aliasr: parameter missing.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_status_t status = SWITCH_STATUS_SUCCESS;
    switch_core_session_t *ses = nullptr;
    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *mycmd = switch_core_strdup(pool, cmd);

    char *argv[1] = {0};
    int argc = switch_split(mycmd, ' ', argv);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", mycmd, argc);

    if (argc < 1) {
        stream->write_function(stream, "parameter number is invalid.\n");
        switch_goto_status(SWITCH_STATUS_SUCCESS, end);
    }

    ses = switch_core_session_force_locate(argv[0]);
    if (!ses) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "stop aliasr failed, can't found session by %s\n",
                          argv[0]);
    } else {
        switch_da_t *pvt;
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if ((pvt = (switch_da_t *) switch_channel_get_private(channel, "asr"))) {
            switch_channel_set_private(channel, "asr", nullptr);
            switch_core_media_bug_remove(ses, &pvt->bug);

            if (pvt->resampler) {
                switch_resample_destroy(&pvt->resampler);
            }
            switch_mutex_destroy(pvt->mutex);
            switch_core_destroy_memory_pool(&pvt->pool);

            switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(ses), SWITCH_LOG_DEBUG, "%s Stop ASR\n",
                              switch_channel_get_name(channel));
        }

        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }

    end:
    switch_core_destroy_memory_pool(&pool);
    return status;
}

/**
 *  定义load函数，加载时运行
 */
SWITCH_MODULE_LOAD_FUNCTION(mod_aliasr_load) {
    if (load_config() != SWITCH_STATUS_SUCCESS) {
        return SWITCH_STATUS_FALSE;
    }
    int ret = NlsClient::getInstance()->setLogConfig("log-transcriber", LogDebug);
    if (-1 == ret) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "set log failed\n");
        return SWITCH_STATUS_FALSE;
    }
    NlsClient::getInstance()->startWorkThread(4);

    switch_api_interface_t *api_interface = nullptr;
    *module_interface = switch_loadable_module_create_module_interface(pool, modname);

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "mod_aliasr_load start\n");

    // register API

    SWITCH_ADD_API(api_interface,
                   "uuid_replay_aliasr",
                   "uuid_replay_aliasr api",
                   uuid_replay_aliasr_function,
                   "<cmd><args>");

    SWITCH_ADD_API(api_interface,
                   "uuid_start_aliasr",
                   "uuid_start_aliasr api",
                   uuid_start_aliasr_function,
                   "<cmd><args>");

    SWITCH_ADD_API(api_interface,
                   "uuid_stop_aliasr",
                   "uuid_stop_aliasr api",
                   uuid_stop_aliasr_function,
                   "<cmd><args>");

    //注册终端命令自动补全
//        switch_console_set_complete("add tasktest1 [args]");
//        switch_console_set_complete("add tasktest2 [args]");

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " aliasr_load\n");

    return SWITCH_STATUS_SUCCESS;
}
/**
 *  定义shutdown函数，关闭时运行
 */
SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_aliasr_shutdown) {
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " mod_aliasr_shutdown called\n");

    return SWITCH_STATUS_SUCCESS;
}
