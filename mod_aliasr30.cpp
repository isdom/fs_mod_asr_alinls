#include <switch.h>
#include <cmath>
#include "nlsClient.h"
#include "nlsEvent.h"
#include "speechTranscriberRequest.h"
#include "nlsCommonSdk/Token.h"
//#include <fstream>
//#include <sys/time.h>
// #include <map>
//#include <queue>

#include <oss_c_sdk/oss_api.h>
#include <oss_c_sdk/aos_http_io.h>

// #include <libks/ks_thread_pool.h>

#define MAX_FRAME_BUFFER_SIZE (1024*1024) //1MB
#define SAMPLE_RATE 8000

using namespace AlibabaNlsCommon;
using AlibabaNls::NlsClient;
using AlibabaNls::NlsEvent;
using AlibabaNls::LogDebug;
using AlibabaNls::SpeechTranscriberRequest;

std::string g_akId = "";
std::string g_akSecret = "";
std::string g_token = "";
long g_expireTime = -1;
bool g_debug = false;


typedef struct  {
    char *caller;
    char *callee;
    char *unique_id;
} asr_callback_t;

//======================================== ali asr start ===============

#pragma pack(push, 1)

typedef struct {
    uint32_t actual_samples_per_second;
    int microseconds_per_packet;
} pcm_sample_t;

typedef struct pcm_slice_s {
    struct pcm_slice_s *_next;
    switch_time_t _from_answered;
    int32_t _raw_len;
    uint8_t _raw_data[];
} pcm_slice_t;

#define _OFFSET_OF(m,y) (uint8_t*)(&((m*)0)->y)
const static int SLICE_FIXED_LEN = _OFFSET_OF(pcm_slice_t, _raw_data) - _OFFSET_OF(pcm_slice_t, _from_answered);

const uint8_t PCM_VERSION[4] = {'P', 'C', 'M', '1'};
typedef struct {
    uint8_t version[4];
    uint32_t body_bytes;
    pcm_sample_t sample;
    uint8_t is_recv_bye;
    pcm_slice_t *header;
    pcm_slice_t *tail;
    char name[];
} pcm_track_t;

#pragma pack(pop)

const static int TRACK_FIXED_LEN = _OFFSET_OF(pcm_track_t, header) - _OFFSET_OF(pcm_track_t, version);

typedef std::map<std::string, pcm_track_t*> id2track_t;
id2track_t g_pcm_tracks;

const uint32_t MAX_TRACK_PENDING_UPLOAD = 1000;

switch_memory_pool_t  *g_mod_pool = nullptr;
switch_queue_t *g_tracks_to_upload = nullptr;
switch_thread_t *g_upload_to_oss_thread = nullptr;

typedef struct {
    switch_core_session_t *session;
    switch_media_bug_t *bug;
    SpeechTranscriberRequest *request;
    int started;
    int stopped;
    int starting;
    int data_len;
    switch_mutex_t *mutex;
    switch_memory_pool_t *pool;
    switch_audio_resampler_t *re_sampler;
    char *app_key;
    char *nls_url;
    char *save_pcm;
    char *asr_dec_vol;
    float vol_multiplier;
    pcm_track_t *track;
} switch_da_t;

typedef void (*asr_callback_func_t)(NlsEvent *, void *);

SpeechTranscriberRequest *generateAsrRequest(asr_callback_t *asr_callback, switch_da_t *pvt);

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
void onAsrTranscriptionStarted(NlsEvent *cbEvent, asr_callback_t *asr_callback) {
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: %s\n", asr_callback->unique_id);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: status code=%d, task id=%s\n",
                      cbEvent->getStatusCode(), cbEvent->getTaskId());
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: all response=%s\n",
                      cbEvent->getAllResponse());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(asr_callback->unique_id);
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
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", asr_callback->unique_id);
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
void onAsrSentenceBegin(NlsEvent *cbEvent, asr_callback_t *asr_callback) {
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceBegin: %s\n", asr_callback->unique_id);
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
void onAsrSentenceEnd(NlsEvent *cbEvent, asr_callback_t *asr_callback) {
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceEnd: %s\n", asr_callback->unique_id);
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
    switch_core_session_t *ses = switch_core_session_force_locate(asr_callback->unique_id);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
            event->subclass_name = (char *) malloc(strlen("start_asr_") + strlen(asr_callback->unique_id) + 1);
            strcpy(event->subclass_name, "start_asr_");
            strcat(event->subclass_name, asr_callback->unique_id);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", asr_callback->unique_id);

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
void onAsrTranscriptionResultChanged(NlsEvent *cbEvent, asr_callback_t *asr_callback) {
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionResultChanged: %s\n", asr_callback->unique_id);
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
    switch_core_session_t *ses = switch_core_session_force_locate(asr_callback->unique_id);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
            event->subclass_name = strdup("update_asr");
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", asr_callback->unique_id);

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
void onAsrTranscriptionCompleted(NlsEvent *cbEvent, asr_callback_t *asr_callback) {
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionCompleted: %s\n", asr_callback->unique_id);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrTranscriptionCompleted: status code=%d, task id=%s\n", cbEvent->getStatusCode(),
                      cbEvent->getTaskId());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(asr_callback->unique_id);
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
void onAsrTaskFailed(NlsEvent *cbEvent, asr_callback_t *asr_callback) {
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTaskFailed: %s\n", asr_callback->unique_id);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrTaskFailed: status code=%d, task id=%s, error message=%s\n", cbEvent->getStatusCode(),
                      cbEvent->getTaskId(), cbEvent->getErrorMessage());
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTaskFailed: all response=%s\n",
                      cbEvent->getAllResponse());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(asr_callback->unique_id);
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
void onAsrSentenceSemantics(NlsEvent *cbEvent, asr_callback_t *asr_callback) {
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceSemantics: %s\n", asr_callback->unique_id);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceSemantics: all response=%s\n",
                      cbEvent->getAllResponse());
}

/**
 * @brief 识别结束或发生异常时，会关闭连接通道, sdk内部线程上报ChannelCloseed事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void onAsrChannelClosed(NlsEvent *cbEvent, asr_callback_t *asr_callback) {
    switch_event_t *event = nullptr;
    if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
        event->subclass_name = strdup("stop_asr");
        switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
        switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Close", cbEvent->getResult());
        switch_event_fire(&event);
    }
    // release asr_callback_t
    switch_safe_free(asr_callback->callee);
    switch_safe_free(asr_callback->caller);
    switch_safe_free(asr_callback->unique_id);
    switch_safe_free(asr_callback);
}

/**
 * @brief asr请求构建
 * 
 * @param asr_callback
 * @return SpeechTranscriberRequest* 
 */
SpeechTranscriberRequest *generateAsrRequest(asr_callback_t *asr_callback, switch_da_t *pvt) {
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
    request->setOnTranscriptionStarted(reinterpret_cast<asr_callback_func_t>(onAsrTranscriptionStarted), asr_callback);
    // 设置识别启动回调函数
    request->setOnTranscriptionResultChanged(reinterpret_cast<asr_callback_func_t>(onAsrTranscriptionResultChanged), asr_callback);
    // 设置识别结果变化回调函数
    request->setOnTranscriptionCompleted(reinterpret_cast<asr_callback_func_t>(onAsrTranscriptionCompleted), asr_callback);
    // 设置语音转写结束回调函数
    request->setOnSentenceBegin(reinterpret_cast<asr_callback_func_t>(onAsrSentenceBegin), asr_callback);
    // 设置一句话开始回调函数
    request->setOnSentenceEnd(reinterpret_cast<asr_callback_func_t>(onAsrSentenceEnd), asr_callback);
    // 设置一句话结束回调函数
    request->setOnTaskFailed(reinterpret_cast<asr_callback_func_t>(onAsrTaskFailed), asr_callback);
    // 设置异常识别回调函数
    request->setOnChannelClosed(reinterpret_cast<asr_callback_func_t>(onAsrChannelClosed), asr_callback);
    // 设置识别通道关闭回调函数
    request->setOnSentenceSemantics(reinterpret_cast<asr_callback_func_t>(onAsrSentenceSemantics), asr_callback);
    //设置二次结果返回回调函数, 开启enable_nlp后返回
    request->setAppKey(pvt->app_key);
    // 设置AppKey, 必填参数, 请参照官网申请
    request->setUrl(pvt->nls_url);
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
                      pvt->nls_url, pvt->vol_multiplier);
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

void append_raw_pcm(pcm_track_t *track, pcm_slice_t *slice) {
    if (!track->header) {
        track->tail = track->header = slice;
    } else {
        track->tail->_next = slice;
        track->tail = slice;
    }
}

static void append_current_pcm(switch_da_t *pvt,
                               switch_channel_t *channel,
                               const switch_frame_t &frame) {
    switch_channel_timetable_t *times = switch_channel_get_timetable(channel);

    // channel->caller_profile->times->answered = switch_micro_time_now();
// https://github.com/signalwire/freeswitch/blob/792eee44d0611422cce3c3194f95125916a7d268/src/switch_channel.c#L3834C3-L3834C70
    auto slice = (pcm_slice_t*)malloc(sizeof(pcm_slice_t) + frame.datalen);
    slice->_next = nullptr;
    slice->_from_answered = switch_micro_time_now() - times->answered;
    slice->_raw_len = frame.datalen;
    memcpy(slice->_raw_data, frame.data, slice->_raw_len);
    append_raw_pcm(pvt->track, slice);
}

static void handleABCTypeRead(switch_media_bug_t *bug, switch_da_t *pvt, switch_channel_t *channel) {
    uint8_t data[SWITCH_RECOMMENDED_BUFFER_SIZE];
    switch_frame_t frame = {0};
    frame.data = data;
    frame.buflen = sizeof(data);
    if (switch_core_media_bug_read(bug, &frame, SWITCH_FALSE) != SWITCH_STATUS_FALSE) {
        switch_mutex_lock(pvt->mutex);

        if (pvt->save_pcm) {
            append_current_pcm(pvt, channel, frame);
        }

        uint32_t data_len = frame.datalen;
        auto *dp = (int16_t *) frame.data;

        if (pvt->re_sampler) {
            //====== resample ==== ///
            switch_resample_process(pvt->re_sampler, dp, (int) data_len / 2 / 1);
            memcpy(dp, pvt->re_sampler->to, pvt->re_sampler->to_len * 2 * 1);
            data_len = pvt->re_sampler->to_len * 2 * 1;
            if (g_debug) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "ASR new samples:%d\n", pvt->re_sampler->to_len);
            }
        }
        if (pvt->asr_dec_vol) {
            adjustVolume((int16_t *) dp, (size_t) data_len / 2, pvt->vol_multiplier);
        }
        if (pvt->request->sendAudio((uint8_t *) dp, (size_t) data_len) < 0) {
            pvt->stopped = 1;
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "send audio failed:%s\n",
                              switch_channel_get_name(channel));
            pvt->request->stop();
            NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
        }
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "SWITCH_ABC_TYPE_READ: send audio %d\n",
                              data_len);
        }
        switch_mutex_unlock(pvt->mutex);
    } else {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "switch_core_media_bug_read failed\n");
    }
}

static void handleABCTypeClose(switch_da_t *pvt, switch_channel_t *channel) {
    if (pvt->request) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Stop Succeed channel: %s\n",
                          switch_channel_get_name(channel));
        pvt->request->stop();
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr stopped:%s\n",
                          switch_channel_get_name(channel));
        //7: 识别结束, 释放request对象
        NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr released:%s\n",
                          switch_channel_get_name(channel));
    }
}

static void handleABCTypeInit(switch_da_t *pvt, switch_channel_t *channel) {
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Channel Init:%s\n",
              switch_channel_get_name(channel));

    switch_codec_t *read_codec = switch_core_session_get_read_codec(pvt->session);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "read_codec=[%s]!\n",
                      read_codec->implementation->iananame);

    if (pvt->stopped == 1) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "SWITCH_ABC_TYPE_INIT: pvt->stopped\n");
        return;
    }

    switch_mutex_lock(pvt->mutex);
    if (pvt->started == 0) {
        if (pvt->starting == 0) {
            pvt->starting = 1;
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Starting Transaction \n");
            auto cbParam = (asr_callback_t*)malloc(sizeof(asr_callback_t));
            cbParam->unique_id = strdup(switch_channel_get_uuid(channel));
            switch_caller_profile_t *profile = switch_channel_get_caller_profile(channel);
            cbParam->caller = strdup(profile->caller_id_number);
            cbParam->callee = strdup(profile->callee_id_number);
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Caller %s. Callee %s\n",
                              cbParam->caller, cbParam->callee);
            SpeechTranscriberRequest *request = generateAsrRequest(cbParam, pvt);
            if (!request) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "Asr Request init failed.%s\n",
                                  switch_channel_get_name(channel));
                goto unlock;
            }
            pvt->request = request;
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Init SpeechTranscriberRequest.%s\n",
                              switch_channel_get_name(channel));
            if (pvt->request->start() < 0) {
                pvt->stopped = 1;
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                                  "start() failed. may be can not connect server. please check network or firewalld:%s\n",
                                  switch_channel_get_name(channel));
                NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                // start()失败，释放request对象
            }
        }
    }

    unlock:
    switch_mutex_unlock(pvt->mutex);
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
        case SWITCH_ABC_TYPE_INIT:
            handleABCTypeInit(pvt, channel);
            break;
        case SWITCH_ABC_TYPE_CLOSE:
            handleABCTypeClose(pvt, channel);
            break;
        case SWITCH_ABC_TYPE_READ:
            handleABCTypeRead(bug, pvt, channel);
            break;
        default:
            break;
    }
    return SWITCH_TRUE;
}

void release_track(pcm_track_t *track) {
    pcm_slice_t *slice = track->header;
    while (slice) {
        pcm_slice_t *next_slice = slice->_next;
        free(slice);
        slice = next_slice;
    }
    track->header = track->tail = nullptr;
    free(track);
}

uint32_t calc_track_body_bytes(pcm_track_t *track) {
    uint32_t body_bytes = 0;
    pcm_slice_t *slice = track->header;
    while (slice) {
        body_bytes += SLICE_FIXED_LEN + slice->_raw_len;
        slice = slice->_next;
    }
    return body_bytes;
}

/*
void save_track_to(pcm_track_t *track, const char *filename) {
    FILE *output = fopen(filename, "wb");
    if (!output) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "failed to fopen %s\n", filename);
        return;
    }
    track->body_bytes = calc_track_body_bytes(track);
    fwrite(track, TRACK_FIXED_LEN, 1, output);

    pcm_slice_t *current = track->header;
    while (current) {
        fwrite(&(current->_from_answered), SLICE_FIXED_LEN + current->_raw_len, 1, output);
        current = current->_next;
    }
    fclose(output);
}
 */

pcm_track_t *load_track_from(const char *filename) {
    FILE *input = fopen(filename, "rb");
    if (!input) {
        return nullptr;
    }

    auto track = (pcm_track_t*) malloc(sizeof(pcm_track_t));
    if (!track) {
        fclose(input);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "malloc pcm_track_t failed, OOM\n");
        return nullptr;
    }
    memset(track, 0, sizeof(pcm_track_t));

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "try to read track fixed part: %d\n", TRACK_FIXED_LEN);
    if (fread(track, TRACK_FIXED_LEN, 1, input) <= 0) {
        free(track);
        fclose(input);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "read pcm_sample_t failed\n");
        return nullptr;
    }
    // check version: PCM1
    if (memcmp(track->version, PCM_VERSION, sizeof(PCM_VERSION)) != 0) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "track file has unsupport version: %c%c%c%c, Abort loading!\n",
                          track->version[0], track->version[1], track->version[2], track->version[3]);
        free(track);
        fclose(input);
        return nullptr;
    }

    int idx = 0;
    while (!feof(input)) {
        pcm_slice_t fixed;
        if (fread(&(fixed._from_answered), SLICE_FIXED_LEN, 1, input) <= 0) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "read pcm_slice_t fixed failed\n");
            break;
        } else {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "(%d)read pcm_slice_t fixed, _raw_len: %d\n",
                              idx++, fixed._raw_len);
            auto slice = (pcm_slice_t *) malloc(sizeof(pcm_slice_t) + fixed._raw_len);
            if (!slice) {
                release_track(track);
                fclose(input);
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "malloc sizeof(pcm_slice_t) + fixed._raw_len = (%ld) failed, OOM\n", sizeof(pcm_slice_t) + fixed._raw_len);
                return nullptr;
            }
            memset(slice, 0, sizeof(pcm_slice_t) + fixed._raw_len);
            slice->_next = nullptr;
            slice->_from_answered = fixed._from_answered;
            slice->_raw_len = fixed._raw_len;
            if (fread(slice->_raw_data, fixed._raw_len, 1, input) <= 0) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "read _raw_data(%d) failed\n",
                                  slice->_raw_len);
                break;
            } else {
                append_raw_pcm(track, slice);
            }
        }
    }

    // check body size
    uint32_t calculated_body_bytes = calc_track_body_bytes(track);
    if (track->body_bytes != calculated_body_bytes) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "mismatched body_bytes, %d(record in file) != %d(calculated)\n",
                          track->body_bytes, calculated_body_bytes);
    }

    fclose(input);
    return track;
}

static void *SWITCH_THREAD_FUNC replay_thread(switch_thread_t *thread, void *obj) {

    auto pvt = (switch_da_t *) obj;
    switch_channel_t *channel = switch_core_session_get_channel(pvt->session);

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "replay thread started for %s\n",
                      switch_channel_get_name(channel));

    switch_channel_timetable_t *times = switch_channel_get_timetable(channel);
    pcm_slice_t *current = pvt->track->header;

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Channel Init:%s\n",
                      switch_channel_get_name(channel));

    switch_codec_t *read_codec = switch_core_session_get_read_codec(pvt->session);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "read_codec=[%s]!\n",
                      read_codec->implementation->iananame);

    int idx = 0;
    switch_mutex_lock(pvt->mutex);
    if (pvt->started == 0) {
        if (pvt->starting == 0) {
            pvt->starting = 1;
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Starting Transaction \n");
            auto cbParam = (asr_callback_t*)malloc(sizeof(asr_callback_t));
            cbParam->unique_id = strdup(switch_channel_get_uuid(channel));
            switch_caller_profile_t *profile = switch_channel_get_caller_profile(channel);
            cbParam->caller = strdup(profile->caller_id_number);
            cbParam->callee = strdup(profile->callee_id_number);
            SpeechTranscriberRequest *request = generateAsrRequest(cbParam, pvt);
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Caller %s. Callee %s\n",
                              cbParam->caller, cbParam->callee);
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
                pvt->stopped = 1;
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                                  "start() failed. may be can not connect server. please check network or firewalld:%s\n",
                                  switch_channel_get_name(channel));
                switch_mutex_unlock(pvt->mutex);
                goto end;
            }
        }
    }
    switch_mutex_unlock(pvt->mutex);
    while (current) {
        const switch_time_t duration = switch_micro_time_now() - times->answered;
        if (current->_from_answered <= duration) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                              "(%d)-> org:%ld, replay:%ld\n", idx++, current->_from_answered, duration);
            // replay to asr
            switch_mutex_lock(pvt->mutex);

            if (pvt->re_sampler) {
                //====== resample ==== ///
                switch_resample_process(pvt->re_sampler, (int16_t *)current->_raw_data, (int) current->_raw_len / 2 / 1);
                memcpy(current->_raw_data, pvt->re_sampler->to, pvt->re_sampler->to_len * 2 * 1);
                current->_raw_len = pvt->re_sampler->to_len * 2 * 1;
                if (g_debug) {
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "ASR new samples:%d\n", pvt->re_sampler->to_len);
                }
            }
            if (pvt->asr_dec_vol) {
                adjustVolume((int16_t *) current->_raw_data, (size_t) current->_raw_len / 2, pvt->vol_multiplier);
            }
            if (pvt->request->sendAudio(current->_raw_data, (size_t) current->_raw_len) < 0) {
                pvt->stopped = 1;
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "send audio failed:%s\n",
                                  switch_channel_get_name(channel));
                switch_mutex_unlock(pvt->mutex);
                goto end;
            }
            if (g_debug) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "SWITCH_ABC_TYPE_READ: send audio %d\n",
                                  current->_raw_len);
            }
            switch_mutex_unlock(pvt->mutex);
            current = current->_next;
        } else {
            switch_yield(10 * 1000);
        }
    }

    end:
    if (pvt->request) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Stop Succeed channel: %s\n",
                          switch_channel_get_name(channel));
        pvt->request->stop();
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr stopped:%s\n",
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
//    release_track(pvt->track);
    return nullptr;
}

switch_status_t on_channel_hangup(switch_core_session_t *session) {
    switch_da_t *pvt;
    switch_channel_t *channel = switch_core_session_get_channel(session);
    if ((pvt = (switch_da_t *) switch_channel_get_private(channel, "asr"))) {
        const char *sip_hangup_disposition_value = switch_channel_get_variable(channel, "sip_hangup_disposition");
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                          "%s on_hangup, sip_hangup_disposition: %s\n",
                          switch_channel_get_name(channel), sip_hangup_disposition_value);
        if (pvt->track) {
            pvt->track->is_recv_bye = !strcmp("recv_bye", sip_hangup_disposition_value);
        }
    }
    return SWITCH_STATUS_SUCCESS;
}

switch_status_t on_channel_destroy(switch_core_session_t *session) {
    switch_da_t *pvt;
    switch_channel_t *channel = switch_core_session_get_channel(session);
    switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                      "%s on_destroy, release all resource for session\n",
                      switch_channel_get_name(channel));

    if ((pvt = (switch_da_t *) switch_channel_get_private(channel, "asr"))) {
        switch_channel_set_private(channel, "asr", nullptr);
        if (pvt->re_sampler) {
            switch_resample_destroy(&pvt->re_sampler);
            switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                              "%s on_destroy: switch_resample_destroy\n",
                              switch_channel_get_name(channel));
        }
        switch_mutex_destroy(pvt->mutex);
        switch_core_destroy_memory_pool(&pvt->pool);
        if (pvt->track) {
            // try to upload to oss
            // save_track_to(pvt->track, pvt->save_pcm);
            /**
             * @returns APR_EINTR the blocking operation was interrupted (try again)
             * @returns APR_EAGAIN the queue is full
             * @returns APR_EOF the queue has been terminated
             * @returns APR_SUCCESS on a successfull push
            */

            pvt->track->body_bytes = calc_track_body_bytes(pvt->track);
            if (g_upload_to_oss_thread) {
                if (APR_SUCCESS != switch_queue_trypush(g_tracks_to_upload, pvt->track)) {
                    // then destroy pcms
                    release_track(pvt->track);
                }
            } else {
                release_track(pvt->track);
            }
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
        on_channel_hangup,
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

typedef struct {
    const char *oss_ak_id;
    const char *oss_ak_secret;
    const char *endpoint;
    const char *bucket;
    switch_queue_t *upload_queue;
} upload_to_oss_t;

/* yourEndpoint填写Bucket所在地域对应的Endpoint。以华东1（杭州）为例，Endpoint填写为https://oss-cn-hangzhou.aliyuncs.com。*/
//const char *endpoint = "yourEndpoint";
///* 填写Bucket名称，例如examplebucket。*/
//const char *bucket_name = "examplebucket";
///* 填写Object完整路径，完整路径中不能包含Bucket名称，例如exampledir/exampleobject.txt。*/
//const char *object_name = "exampledir/exampleobject.txt";
//const char *object_content = "More than just cloud.";
void init_options(oss_request_options_t *options, const char *akid, const char *aksecret, const char *endpoint)
{
    options->config = oss_config_create(options->pool);
    /* 用char*类型的字符串初始化aos_string_t类型。*/
    aos_str_set(&options->config->endpoint, endpoint);
    /* 从环境变量中获取访问凭证。运行本代码示例之前，请确保已设置环境变量OSS_ACCESS_KEY_ID和OSS_ACCESS_KEY_SECRET。*/
    aos_str_set(&options->config->access_key_id, akid);
    aos_str_set(&options->config->access_key_secret, aksecret);
    /* 是否使用了CNAME。0表示不使用。*/
    options->config->is_cname = 0;
    /* 设置网络相关参数，比如超时时间等。*/
    options->ctl = aos_http_controller_create(options->pool, 0);
}

void upload_to_oss(pcm_track_t *track, upload_to_oss_t *uto) {
    /* 用于内存管理的内存池（pool），等价于apr_pool_t。其实现代码在apr库中。*/
    aos_pool_t *aos_pool;
    /* 创建并初始化options，该参数包括endpoint、access_key_id、acces_key_secret、is_cname、curl等全局配置信息。*/
    oss_request_options_t *oss_client_options;
    /* 初始化参数。*/
    aos_string_t bucket;
    aos_string_t object;
    aos_list_t buffer;
    aos_table_t *headers = nullptr;
    aos_table_t *resp_headers = nullptr;
    aos_status_t *resp_status = nullptr;

    /* 重新创建一个内存池，第二个参数是NULL，表示没有继承其它内存池。*/
    aos_pool_create(&aos_pool, nullptr);
    /* 在内存池中分配内存给options。*/
    oss_client_options = oss_request_options_create(aos_pool);
    /* 初始化Client的选项oss_client_options。*/
    init_options(oss_client_options, uto->oss_ak_id, uto->oss_ak_secret, uto->endpoint);

    aos_str_set(&bucket, uto->bucket);
    aos_str_set(&object, track->name);
    aos_list_init(&buffer);
    {
        aos_buf_t *content = nullptr;
        content = aos_buf_pack(oss_client_options->pool, track, TRACK_FIXED_LEN);
        aos_list_add_tail(&content->node, &buffer);

        pcm_slice_t *current = track->header;
        while (current) {
            content = aos_buf_pack(oss_client_options->pool, &(current->_from_answered), SLICE_FIXED_LEN + current->_raw_len);
            aos_list_add_tail(&content->node, &buffer);
            current = current->_next;
        }
    }
    /* 上传文件。*/
    resp_status = oss_put_object_from_buffer(oss_client_options, &bucket, &object, &buffer, headers, &resp_headers);
    /* 判断上传是否成功。*/
    if (aos_status_is_ok(resp_status)) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "put object (%s) from buffer succeeded\n", track->name);
    } else {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "put object (%s) from buffer failed\n", track->name);
    }
    /* 释放内存池，相当于释放了请求过程中各资源分配的内存。*/
    aos_pool_destroy(aos_pool);
}

static void *SWITCH_THREAD_FUNC upload_to_oss_thread(switch_thread_t *thread, upload_to_oss_t *uto) {

    pcm_track_t *track_to_upload = nullptr;

    while (true) {
        const switch_status_t status = switch_queue_pop_timeout(uto->upload_queue,
                                                                reinterpret_cast<void **>(&track_to_upload),
                                                                500 * 1000);
        switch (status) {
            // the request timed out
            case APR_TIMEUP:
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "upload_to_oss_thread: fetch timeout\n");
                break;
                //  the blocking was interrupted (try again)
            case APR_EINTR:
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "upload_to_oss_thread: fetch interrupted\n");
                break;
                // if the queue has been terminated
            case APR_EOF:
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "upload_to_oss_thread: terminated\n");
                goto end;
                //  on a successfull pop
            case APR_SUCCESS:
                if (track_to_upload) {
                    upload_to_oss(track_to_upload, uto);
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "upload_to_oss_thread: uto %s(%d) to oss\n",
                                      track_to_upload->name, track_to_upload->body_bytes);
                    release_track(track_to_upload);
                } else {
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "upload_to_oss_thread: APR_SUCCESS but track_to_upload is null\n");
                }
                break;
        }
    }

    end:
    return nullptr;
}

#define MAX_API_ARGC 10

// start_oss_upload akid=<akid> aksecret=<aksecret> endpoint=<endpoint> bucket=<bucket_name>
SWITCH_STANDARD_API(start_oss_upload_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "start_oss_upload: parameter missing.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    if (g_upload_to_oss_thread) {
        stream->write_function(stream, "start_oss_upload: upload_to_oss has already running.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_status_t status = SWITCH_STATUS_SUCCESS;
    char *_oss_ak_id = nullptr;
    char *_oss_ak_secret = nullptr;
    char *_endpoint = nullptr;
    char *_bucket = nullptr;

    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *mycmd = switch_core_strdup(pool, cmd);

    char *argv[MAX_API_ARGC];
    memset(argv, 0, sizeof(char *) * MAX_API_ARGC);

    int argc = switch_split(mycmd, ' ', argv);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", mycmd, argc);

    if (argc < 6) {
        stream->write_function(stream, "akid/aksecret/endpoint/bucket are required.\n");
        switch_goto_status(SWITCH_STATUS_SUCCESS, end);
    }

    for (int idx = 0; idx < MAX_API_ARGC; idx++) {
        if (argv[idx]) {
            char *ss[2] = {0, 0};
            int cnt = switch_split(argv[idx], '=', ss);
            if (cnt == 2) {
                char *var = ss[0];
                char *val = ss[1];
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "process arg: %s = %s\n", var, val);
                if (!strcasecmp(var, "akid")) {
                    _oss_ak_id = val;
                    continue;
                }
                if (!strcasecmp(var, "aksecret")) {
                    _oss_ak_secret = val;
                    continue;
                }
                if (!strcasecmp(var, "endpoint")) {
                    _endpoint = val;
                    continue;
                }
                if (!strcasecmp(var, "bucket")) {
                    _bucket = val;
                    continue;
                }
            }
        }
    }

    if (!_oss_ak_id || !_oss_ak_secret || !_endpoint || !_bucket ) {
        stream->write_function(stream, "akid/aksecret/endpoint/bucket are required.\n");
        switch_goto_status(SWITCH_STATUS_SUCCESS, end);
    }

    {
        switch_threadattr_t *thd_attr = nullptr;
        switch_threadattr_create(&thd_attr, g_mod_pool);
        switch_threadattr_stacksize_set(thd_attr, SWITCH_THREAD_STACKSIZE);

        auto uto = (upload_to_oss_t*)switch_core_alloc(g_mod_pool, sizeof(upload_to_oss_t));
        uto->oss_ak_id = switch_core_strdup(g_mod_pool, _oss_ak_id);
        uto->oss_ak_secret = switch_core_strdup(g_mod_pool, _oss_ak_secret);
        uto->endpoint = switch_core_strdup(g_mod_pool, _endpoint);
        uto->bucket = switch_core_strdup(g_mod_pool, _bucket);
        uto->upload_queue = g_tracks_to_upload;

        switch_thread_create(&g_upload_to_oss_thread, thd_attr,
                             reinterpret_cast<switch_thread_start_t>(upload_to_oss_thread), uto, g_mod_pool);
    }

    end:
    switch_core_destroy_memory_pool(&pool);
    return status;
}

// load_pcm_aliasr track_id file=<local path>
SWITCH_STANDARD_API(load_pcm_aliasr_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "load_pcm_aliasr: parameter missing.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    pcm_track_t *track = nullptr;
    switch_status_t status = SWITCH_STATUS_SUCCESS;
    char *_file = nullptr;

    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *mycmd = switch_core_strdup(pool, cmd);

    char *argv[MAX_API_ARGC];
    memset(argv, 0, sizeof(char *) * MAX_API_ARGC);

    int argc = switch_split(mycmd, ' ', argv);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", mycmd, argc);

    if (argc < 1) {
        stream->write_function(stream, "track_id is required.\n");
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
                if (!strcasecmp(var, "file")) {
                    _file = val;
                    continue;
                }
            }
        }
    }

    if (!_file) {
        stream->write_function(stream, "file are required.\n");
        switch_goto_status(SWITCH_STATUS_SUCCESS, end);
    }

    track = load_track_from(_file);
    if (track) {
        auto iter = g_pcm_tracks.find(argv[0]);

        if(iter != g_pcm_tracks.end()) {
            g_pcm_tracks.erase(iter);
            release_track(iter->second);
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "has been load track associate with %s，release old track\n", argv[0]);
        }
        auto result = g_pcm_tracks.insert(std::pair<std::string, pcm_track_t*>(argv[0], track));
        if (result.second) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "load %s to %s success\n", _file, argv[0]);
        } else {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "load %s to %s meet some error\n", _file, argv[0]);
        }
    } else {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "load %s to %s failed\n", _file, argv[0]);
    }

    end:
    switch_core_destroy_memory_pool(&pool);
    return status;
}

// uuid_replay_aliasr <uuid> appkey=<appkey> nls=<nls_url> trackid=<track id> debug=<true/false>
SWITCH_STANDARD_API(uuid_replay_aliasr_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "uuid_replay_aliasr: parameter missing.\n");
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "uuid_replay_aliasr: parameter missing.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_status_t status = SWITCH_STATUS_SUCCESS;
    switch_core_session_t *ses = nullptr;
    char *_appkey = nullptr;
    char *_nlsurl = nullptr;
    char *_asr_dec_vol = nullptr;
    char *_trackid = nullptr;
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
                if (!strcasecmp(var, "trackid")) {
                    _trackid = val;
                    continue;
                }
            }
        }
    }

    if (!_appkey || !_nlsurl || !_trackid) {
        stream->write_function(stream, "appkey/nls/trackid are required.\n");
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "appkey/nls/trackid are required.\n");
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
        pvt->stopped = 0;
        pvt->starting = 0;
        pvt->data_len = 0;
        pvt->session = ses;
        pvt->app_key = switch_core_session_strdup(ses, _appkey);
        pvt->nls_url = switch_core_session_strdup(ses, _nlsurl);
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
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "hook channel state change failed!\n");
        }

        {
            auto iter = g_pcm_tracks.find(_trackid);
            if (iter == g_pcm_tracks.end() || !iter->second) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "replay to aliasr failed, can't found track by %s\n",
                                  _trackid);
                switch_goto_status(SWITCH_STATUS_SUCCESS, end);
            }
            pvt->track = iter->second;
        }

        if (pvt->track->sample.actual_samples_per_second != SAMPLE_RATE) {
            if (switch_resample_create(&pvt->re_sampler,
                                       pvt->track->sample.actual_samples_per_second,
                                       SAMPLE_RATE,
                                       8 * (pvt->track->sample.microseconds_per_packet / 1000) * 2,
                                       SWITCH_RESAMPLE_QUALITY,
                                       1) != SWITCH_STATUS_SUCCESS) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "Unable to allocate re_sampler\n");
                switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
            }
        }

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

static void init_track(const switch_codec_implementation_t &read_impl, switch_da_t *pvt) {
    pvt->track = (pcm_track_t*)malloc(sizeof(pcm_track_t) + strlen(pvt->save_pcm)+1);
    memset(pvt->track, 0, sizeof(pcm_track_t));
    memcpy(pvt->track->version, PCM_VERSION, sizeof(PCM_VERSION));
    strcpy(pvt->track->name, pvt->save_pcm);
    pvt->track->sample.actual_samples_per_second = read_impl.actual_samples_per_second;
    pvt->track->sample.microseconds_per_packet = read_impl.microseconds_per_packet;
}

// uuid_start_aliasr <uuid> appkey=<appkey> nls=<nls_url> debug=<true/false> savepcm=<local path>
SWITCH_STANDARD_API(uuid_start_aliasr_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "uuid_start_aliasr: parameter missing.\n");
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "uuid_start_aliasr: parameter missing.\n");
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
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "appkey and nls are required.\n");
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
        pvt->stopped = 0;
        pvt->starting = 0;
        pvt->data_len = 0;
        pvt->session = ses;
        pvt->app_key = switch_core_session_strdup(pvt->session, _appkey);
        pvt->nls_url = switch_core_session_strdup(pvt->session, _nlsurl);
        pvt->save_pcm = _savepcm ? switch_core_session_strdup(pvt->session, _savepcm) : nullptr;
        pvt->asr_dec_vol = _asr_dec_vol ? switch_core_session_strdup(pvt->session, _asr_dec_vol) : nullptr;
        if (pvt->asr_dec_vol) {
            double db = atof(pvt->asr_dec_vol);
            pvt->vol_multiplier = pow(10, db / 20);
        }

        if ((status = switch_core_new_memory_pool(&pvt->pool)) != SWITCH_STATUS_SUCCESS) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Memory Error!\n");
            switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
        }
        switch_mutex_init(&pvt->mutex, SWITCH_MUTEX_NESTED, pvt->pool);

        switch_codec_implementation_t read_impl;
        memset(&read_impl, 0, sizeof(switch_codec_implementation_t));
        switch_core_session_get_read_impl(pvt->session, &read_impl);

        if (read_impl.actual_samples_per_second != SAMPLE_RATE) {
            if (switch_resample_create(&pvt->re_sampler,
                                       read_impl.actual_samples_per_second,
                                       SAMPLE_RATE,
                                       8 * (read_impl.microseconds_per_packet / 1000) * 2,
                                       SWITCH_RESAMPLE_QUALITY,
                                       1) != SWITCH_STATUS_SUCCESS) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "Unable to allocate re_sampler\n");
                switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
            }
        }

        if (pvt->save_pcm) {
            init_track(read_impl, pvt);
        }

        //session添加media bug
        if ((status = switch_core_media_bug_add(ses, "asr", nullptr,
                                                asr_callback, pvt, 0,
                // SMBF_READ_REPLACE | SMBF_WRITE_REPLACE |  SMBF_NO_PAUSE | SMBF_ONE_ONLY,
                                                SMBF_READ_STREAM | SMBF_NO_PAUSE,
                                                &(pvt->bug))) != SWITCH_STATUS_SUCCESS) {
            switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
        }
        switch_channel_set_private(channel, "asr", pvt);

        if (switch_channel_add_state_handler(channel, &asr_cs_handlers) < 0) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "hook channel state change failed!\n");
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

            if (pvt->re_sampler) {
                switch_resample_destroy(&pvt->re_sampler);
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
                   "start_oss_upload",
                   "start_oss_upload api",
                   start_oss_upload_function,
                   "<cmd><args>");

    SWITCH_ADD_API(api_interface,
                   "load_pcm_aliasr",
                   "load_pcm_aliasr api",
                   load_pcm_aliasr_function,
                   "<cmd><args>");

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

    switch_core_new_memory_pool(&g_mod_pool) ;
    switch_queue_create (&g_tracks_to_upload, MAX_TRACK_PENDING_UPLOAD, g_mod_pool);

    /* 在程序入口调用aos_http_io_initialize方法来初始化网络、内存等全局资源。*/
    if (aos_http_io_initialize(NULL, 0) != AOSE_OK) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "failed to aos_http_io_initialize\n");
    }

    switch_thread_t *thread = nullptr;
    switch_threadattr_t *thd_attr = nullptr;

    switch_threadattr_create(&thd_attr, g_mod_pool);
    switch_threadattr_stacksize_set(thd_attr, SWITCH_THREAD_STACKSIZE);

    switch_thread_create(&g_upload_to_oss_thread, thd_attr, reinterpret_cast<switch_thread_start_t>(upload_to_oss_thread),
                         g_tracks_to_upload, g_mod_pool);

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "mod_aliasr_load\n");

    return SWITCH_STATUS_SUCCESS;
}
/**
 *  定义shutdown函数，关闭时运行
 */
SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_aliasr_shutdown) {

    switch_queue_term(g_tracks_to_upload);

    /* 释放之前分配的全局资源。*/
    aos_http_io_deinitialize();

    switch_core_destroy_memory_pool(&g_mod_pool);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " mod_aliasr_shutdown called\n");

    return SWITCH_STATUS_SUCCESS;
}
