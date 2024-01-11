#include <switch.h>
#include <cmath>
#include <sys/time.h>
#include <sstream>
#include "nlsClient.h"
#include "nlsEvent.h"
#include "speechTranscriberRequest.h"
#include "speechSynthesizerRequest.h"
#include "nlsToken.h"

#define SAMPLE_RATE 8000

using namespace AlibabaNlsCommon;
using AlibabaNls::NlsClient;
using AlibabaNls::NlsEvent;
using AlibabaNls::SpeechTranscriberRequest;


const char *g_ak_id = nullptr;
const char *g_ak_secret = nullptr;
std::string g_token = "";
long g_expireTime = -1;
bool g_debug = false;

typedef struct {
    switch_atomic_t ali_asr_concurrent_cnt;
} ali_asr_global_t;

ali_asr_global_t *ali_asr_globals;

// public declare

typedef void (*on_asr_started_func_t) (void *);
typedef void (*on_asr_sentence_begin_func_t) (void *);
typedef void (*on_asr_sentence_end_func_t) (void *, const char *sentence);
typedef void (*on_asr_result_changed_func_t) (void *, const char *result);
typedef void (*on_asr_stopped_func_t) (void *);

typedef struct {
    void *asr_caller;
    on_asr_started_func_t on_asr_started_func;
    on_asr_sentence_begin_func_t on_asr_sentence_begin_func;
    on_asr_sentence_end_func_t on_asr_sentence_end_func;
    on_asr_result_changed_func_t on_asr_result_changed_func;
    on_asr_stopped_func_t on_asr_stopped_func;
} asr_callback_t;

typedef void *(*asr_init_func_t) (switch_core_session_t *, const switch_codec_implementation_t *, const char *);
typedef bool (*asr_start_func_t) (void *asr_data, asr_callback_t *asr_callback);
typedef bool (*asr_send_audio_func_t) (void *asr_data, void *data, uint32_t data_len);
typedef void (*asr_stop_func_t) (void *asr_data);
typedef void (*asr_destroy_func_t) (void *asr_data);

typedef struct {
    asr_init_func_t asr_init_func;
    asr_start_func_t asr_start_func;
    asr_send_audio_func_t asr_send_audio_func;
    asr_stop_func_t asr_stop_func;
    asr_destroy_func_t asr_destroy_func;
} asr_provider_t;

// public declare end

//======================================== ali asr start ===============

typedef struct {
    switch_core_session_t *session;
    SpeechTranscriberRequest *request;
    int started;
    int stopped;
    int starting;
    switch_mutex_t *mutex;
    switch_audio_resampler_t *re_sampler;
    char *app_key;
    char *nls_url;
    char *vad_threshold;
    char *asr_dec_vol;
    float vol_multiplier;
    asr_callback_t *asr_callback;
} ali_asr_context_t;

typedef void (*asr_callback_func_t)(NlsEvent *, void *);

SpeechTranscriberRequest *generateAsrRequest(ali_asr_context_t *pvt);

/**
 * 根据AccessKey ID和AccessKey Secret重新生成一个token，
 * 并获取其有效期时间戳
 */
int generateToken(const char *akId, const char *akSecret, std::string *token, long *expireTime) {
    NlsToken nlsTokenRequest;
    nlsTokenRequest.setAccessKeyId(akId);
    nlsTokenRequest.setKeySecret(akSecret);
    //打印请求token的参数    
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "begin send generate token rquest: akId=%s, akSecret=%s\n",
                      akId, akSecret);
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
 * @param ali_asr_context_t 回调自定义参数，默认为 nullptr, 可以根据需求自定义参数
 */
void onAsrTranscriptionStarted(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrTranscriptionStarted 调用时，确认 ali_asr_context_t * 有效

    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: status code=%d, task id=%s\n",
                          cbEvent->getStatusCode(), cbEvent->getTaskId());
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: all response=%s\n",
                          cbEvent->getAllResponse());
    }
    switch_mutex_lock(pvt->mutex);
    pvt->started = 1;
    pvt->starting = 0;
    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "I need lock!!!!!!!!!!!! \n");
    }
    switch_mutex_unlock(pvt->mutex);

    if (pvt->asr_callback) {
        pvt->asr_callback->on_asr_started_func(pvt->asr_callback->asr_caller);
    }
}

/**
 * @brief 服务端检测到了一句话的开始, sdk内部线程上报SentenceBegin事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param ali_asr_context_t 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrSentenceBegin(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrSentenceBegin 调用时，确认 ali_asr_context_t * 有效

    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                          "onAsrSentenceBegin: status code=%d, task id=%s, index=%d, time=%d\n",
                          cbEvent->getStatusCode(),
                          cbEvent->getTaskId(),
                          cbEvent->getSentenceIndex(),
                          cbEvent->getSentenceTime());
    }
    if (pvt->asr_callback) {
        pvt->asr_callback->on_asr_sentence_begin_func(pvt->asr_callback->asr_caller);
    }
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
 * @param ali_asr_context_t 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrSentenceEnd(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrSentenceEnd 调用时，确认 ali_asr_context_t * 有效

    if (g_debug) {
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
    }
    if (pvt->asr_callback) {
        char *sentence = dupAsrResult(cbEvent->getAllResponse());
        pvt->asr_callback->on_asr_sentence_end_func(pvt->asr_callback->asr_caller, sentence);
        free(sentence);
    }
}

/**
 * @brief 识别结果发生了变化, sdk在接收到云端返回到最新结果时,
 *        sdk内部线程上报ResultChanged事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param ali_asr_context_t 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrTranscriptionResultChanged(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrTranscriptionResultChanged 调用时，确认 ali_asr_context_t * 有效

    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                          "onAsrTranscriptionResultChanged: status code=%d, task id=%s, index=%d, time=%d, result=%s\n",
                          cbEvent->getStatusCode(), cbEvent->getTaskId(),
                          cbEvent->getSentenceIndex(),
                          cbEvent->getSentenceTime(),
                          cbEvent->getResult()
        );
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTranscriptionResultChanged: all response=%s\n",
                          cbEvent->getAllResponse());
    }
    if (pvt->asr_callback) {
        char *result = dupAsrResult(cbEvent->getAllResponse());
        pvt->asr_callback->on_asr_result_changed_func(pvt->asr_callback->asr_caller, result);
        free(result);
    }
}

/**
 * @brief 服务端停止实时音频流识别时, sdk内部线程上报Completed事件
 * @note 上报Completed事件之后，SDK内部会关闭识别连接通道.
         此时调用sendAudio会返回负值, 请停止发送.
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param ali_asr_context_t 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void onAsrTranscriptionCompleted(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrTranscriptionCompleted 调用时，确认 ali_asr_context_t * 有效

    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                          "onAsrTranscriptionCompleted: status code=%d, task id=%s\n", cbEvent->getStatusCode(),
                          cbEvent->getTaskId());
    }
}

/**
 * @brief 识别过程(包含start(), sendAudio(), stop())发生异常时, sdk内部线程上报TaskFailed事件
 * @note 上报TaskFailed事件之后, SDK内部会关闭识别连接通道. 此时调用sendAudio会返回负值, 请停止发送
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param ali_asr_context_t 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void onAsrTaskFailed(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrTaskFailed 调用时，确认 ali_asr_context_t * 有效

    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                          "onAsrTaskFailed: status code=%d, task id=%s, error message=%s\n", cbEvent->getStatusCode(),
                          cbEvent->getTaskId(), cbEvent->getErrorMessage());
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTaskFailed: all response=%s\n",
                          cbEvent->getAllResponse());
    }

    switch_mutex_lock(pvt->mutex);
    pvt->started = 0;
    switch_mutex_unlock(pvt->mutex);
}

/**
 * @brief 二次结果返回回调函数, 开启enable_nlp后返回
 * 
 * @param cbEvent 
 * @param ali_asr_context_t
 */
void onAsrSentenceSemantics(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrSentenceSemantics 调用时，确认 ali_asr_context_t * 有效

    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceSemantics: all response=%s\n",
                          cbEvent->getAllResponse());
    }
}

/**
 * @brief 识别结束或发生异常时，会关闭连接通道, sdk内部线程上报ChannelCloseed事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param asr_context 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void onAsrChannelClosed(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrChannelClosed 调用时，确认 ali_asr_context_t * 有效

    if (pvt->asr_callback) {
        pvt->asr_callback->on_asr_stopped_func(pvt->asr_callback->asr_caller);
    }
}

/**
 * @brief asr请求构建
 * 
 * @param asr_context
 * @return SpeechTranscriberRequest* 
 */
SpeechTranscriberRequest *generateAsrRequest(ali_asr_context_t *pvt) {
    time_t now;
    time(&now);
    if (g_expireTime - now < 10) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                          "the token will be expired, please generate new token by AccessKey-ID and AccessKey-Secret\n");
        if (-1 == generateToken(g_ak_id, g_ak_secret, &g_token, &g_expireTime)) {
            return nullptr;
        }
    }
    SpeechTranscriberRequest *request = NlsClient::getInstance()->createTranscriberRequest("cpp", false);
    if (request == nullptr) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "createTranscriberRequest failed.\n");
        return nullptr;
    }
    request->setOnTranscriptionStarted(reinterpret_cast<asr_callback_func_t>(onAsrTranscriptionStarted), pvt);
    // 设置识别启动回调函数
    request->setOnTranscriptionResultChanged(reinterpret_cast<asr_callback_func_t>(onAsrTranscriptionResultChanged), pvt);
    // 设置识别结果变化回调函数
    request->setOnTranscriptionCompleted(reinterpret_cast<asr_callback_func_t>(onAsrTranscriptionCompleted), pvt);
    // 设置语音转写结束回调函数
    request->setOnSentenceBegin(reinterpret_cast<asr_callback_func_t>(onAsrSentenceBegin), pvt);
    // 设置一句话开始回调函数
    request->setOnSentenceEnd(reinterpret_cast<asr_callback_func_t>(onAsrSentenceEnd), pvt);
    // 设置一句话结束回调函数
    request->setOnTaskFailed(reinterpret_cast<asr_callback_func_t>(onAsrTaskFailed), pvt);
    // 设置异常识别回调函数
    request->setOnChannelClosed(reinterpret_cast<asr_callback_func_t>(onAsrChannelClosed), pvt);
    // 设置识别通道关闭回调函数
    request->setOnSentenceSemantics(reinterpret_cast<asr_callback_func_t>(onAsrSentenceSemantics), pvt);
    //设置二次结果返回回调函数, 开启enable_nlp后返回
    request->setAppKey(pvt->app_key);
    // 设置AppKey, 必填参数, 请参照官网申请
    request->setTimeout(500);

    // 使缺省用 url
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
    if (pvt->vad_threshold) {
        request->setSemanticSentenceDetection(false);
        // 可选参数. 静音时长超过该阈值会被认为断句,
        // 合法参数范围200～2000(ms), 默认值800ms.
        // vad断句与语义断句为互斥关系, 不能同时使用.
        // 调用此设置前, 请将语义断句setSemanticSentenceDetection设置为false.
        request->setMaxSentenceSilence(atoi(pvt->vad_threshold));
    }
    // 设置是否在后处理中执行数字转写, 可选参数. 默认false
    request->setToken(g_token.c_str());
    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "nls url is:%s, vol multiplier is:%f\n",
                          pvt->nls_url, pvt->vol_multiplier);
    }
    return request;
}
//======================================== ali asr end ===============
//======================================== freeswitch module start ===============
SWITCH_MODULE_LOAD_FUNCTION(mod_aliasr_load);

SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_aliasr_shutdown);

extern "C"
{
SWITCH_MODULE_DEFINITION(mod_aliasr, mod_aliasr_load, mod_aliasr_shutdown, nullptr);
};

static void *init_ali_asr(switch_core_session_t *session, const switch_codec_implementation_t *read_impl, const char *cmd);

static bool start_ali_asr(ali_asr_context_t *pvt, asr_callback_t *asr_callback);

static bool send_audio_to_ali_asr(ali_asr_context_t *pvt, void *data, uint32_t data_len);

static void stop_ali_asr(ali_asr_context_t *pvt);

static void cancel_ali_asr(ali_asr_context_t *pvt);

static void destroy_ali_asr(ali_asr_context_t *pvt);

static const asr_provider_t ali_asr_provider_funcs = {
        init_ali_asr,
        reinterpret_cast<asr_start_func_t>(start_ali_asr),
        reinterpret_cast<asr_send_audio_func_t>(send_audio_to_ali_asr),
        reinterpret_cast<asr_stop_func_t>(cancel_ali_asr),
        reinterpret_cast<asr_destroy_func_t>(destroy_ali_asr)
};

static switch_status_t attach_ali_asr_provider_on_channel_init(switch_core_session_t *session) {
    switch_channel_t *channel = switch_core_session_get_channel(session);
    switch_channel_set_private(channel, "ali_asr", &ali_asr_provider_funcs);
    return SWITCH_STATUS_SUCCESS;
}

switch_state_handler_table_t ali_asr_cs_handlers = {
        /*! executed when the state changes to init */
        // switch_state_handler_t on_init;
        attach_ali_asr_provider_on_channel_init,
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
        nullptr,
        // int flags;
        0
};

/**
 * 配置加载 aliyun的appkey，akid，aksecret
 * 
 * @return switch_status_t 执行状态：
 */
static switch_status_t load_config(switch_memory_pool_t *pool) {
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
            g_ak_id = switch_core_strdup(pool, val);
            continue;
        }
        if (!strcasecmp(var, "aksecret")) {
            g_ak_secret = switch_core_strdup(pool, val);
            continue;
        }
        if (!strcasecmp(var, "debug")) {
            if (!strcasecmp(val, "true")) {
                g_debug = true;
            }
            continue;
        }
    }
    switch_xml_free(xml);
    return SWITCH_STATUS_SUCCESS;
}

static void adjustVolume(int16_t *pcm, size_t pcm_len, float vol_multiplier) {
    int32_t pcm_val;
    for (size_t ctr = 0; ctr < pcm_len; ctr++) {
        pcm_val = pcm[ctr] * vol_multiplier;
        if (pcm_val < 32767 && pcm_val > -32768) {
            pcm[ctr] = (int16_t)pcm_val;
        } else if (pcm_val > 32767) {
            pcm[ctr] = 32767;
        } else if (pcm_val < -32768) {
            pcm[ctr] = -32768;
        }
    }
}

#define MAX_API_ARGC 10

static void *init_ali_asr(switch_core_session_t *session, const switch_codec_implementation_t *read_impl, const char *cmd) {
    char *_app_key = nullptr;
    char *_nls_url = nullptr;
    char *_asr_dec_vol = nullptr;
    char *_vad_threshold = nullptr;

    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *my_cmd = switch_core_strdup(pool, cmd);

    char *argv[MAX_API_ARGC];
    memset(argv, 0, sizeof(char *) * MAX_API_ARGC);

    int argc = switch_split(my_cmd, ' ', argv);
    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", my_cmd, argc);
    }

    for (int idx = 1; idx < MAX_API_ARGC; idx++) {
        if (argv[idx]) {
            char *ss[2] = {nullptr, nullptr};
            int cnt = switch_split(argv[idx], '=', ss);
            if (cnt == 2) {
                char *var = ss[0];
                char *val = ss[1];
                if (g_debug) {
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "process arg: %s = %s\n", var, val);
                }
                if (!strcasecmp(var, "appkey")) {
                    _app_key = val;
                    continue;
                }
                if (!strcasecmp(var, "nls")) {
                    _nls_url = val;
                    continue;
                }
                if (!strcasecmp(var, "asr_dec_vol")) {
                    _asr_dec_vol = val;
                    continue;
                }
                if (!strcasecmp(var, "vad_threshold")) {
                    _vad_threshold = val;
                    continue;
                }
            }
        }
    }

    // switch_channel_t *channel = switch_core_session_get_channel(session);
    ali_asr_context_t *pvt;
    if (!(pvt = (ali_asr_context_t *) switch_core_session_alloc(session, sizeof(ali_asr_context_t)))) {
        goto end;
    }
    pvt->started = 0;
    pvt->stopped = 0;
    pvt->starting = 0;
    pvt->session = session;
    pvt->app_key = switch_core_session_strdup(session, _app_key);
    pvt->nls_url = switch_core_session_strdup(session, _nls_url);
    pvt->vad_threshold = _vad_threshold ? switch_core_session_strdup(session, _vad_threshold) : nullptr;
    pvt->asr_dec_vol = _asr_dec_vol ? switch_core_session_strdup(session, _asr_dec_vol) : nullptr;
    if (pvt->asr_dec_vol) {
        double db = atof(pvt->asr_dec_vol);
        pvt->vol_multiplier = pow(10, db / 20);
    }
    switch_mutex_init(&pvt->mutex, SWITCH_MUTEX_NESTED, switch_core_session_get_pool(session));

    if (read_impl->actual_samples_per_second != SAMPLE_RATE) {
        if (switch_resample_create(&pvt->re_sampler,
                                   read_impl->actual_samples_per_second,
                                   SAMPLE_RATE,
                                   8 * (read_impl->microseconds_per_packet / 1000) * 2,
                                   SWITCH_RESAMPLE_QUALITY,
                                   1) != SWITCH_STATUS_SUCCESS) {
            // release all resource alloc before
            switch_mutex_destroy(pvt->mutex);

            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "Unable to allocate re_sampler\n");
            pvt = nullptr;
            goto end;
        }
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT,
                              "create re-sampler bcs of media sampler/s is %d but ali asr support: %d, while ms/p: %d\n",
                              read_impl->actual_samples_per_second, SAMPLE_RATE, read_impl->microseconds_per_packet);
        }
    }
    // increment aliasr concurrent count
    switch_atomic_inc(&ali_asr_globals->ali_asr_concurrent_cnt);

    end:
    switch_core_destroy_memory_pool(&pool);
    return pvt;
}

static bool start_ali_asr(ali_asr_context_t *pvt, asr_callback_t *asr_callback) {
    bool  ret_val = false;
    if (pvt->stopped == 1) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "start_ali_asr: pvt->stopped\n");
        return ret_val;
    }

    switch_mutex_lock(pvt->mutex);
    if (pvt->started == 0) {
        if (pvt->starting == 0) {
            pvt->starting = 1;
            if (g_debug) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Starting Transaction \n");
            }
            switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
            pvt->asr_callback = asr_callback;
            SpeechTranscriberRequest *request = generateAsrRequest(pvt);
            if (!request) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "Asr Request init failed.%s\n",
                                  switch_channel_get_name(channel));
                ret_val = false;
                goto unlock;
            }
            pvt->request = request;
            if (g_debug) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Init SpeechTranscriberRequest.%s\n",
                                  switch_channel_get_name(channel));
            }
            if (pvt->request->start() < 0) {
                pvt->stopped = 1;
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                                  "start() failed. may be can not connect server. please check network or firewalld:%s\n",
                                  switch_channel_get_name(channel));
                NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                pvt->request = nullptr;
                ret_val = false;
                goto unlock;
            } else {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "start() success for %s\n",
                                  switch_channel_get_name(channel));
            }
            ret_val = true;
        }
    }

    unlock:
    switch_mutex_unlock(pvt->mutex);
    return ret_val;
}

static bool send_audio_to_ali_asr(ali_asr_context_t *pvt, void *data, uint32_t data_len) {
    bool  ret_val = false;
    // send audio to asr
    switch_mutex_lock(pvt->mutex);

    if (pvt->request) {
        if (pvt->re_sampler) {
            //====== resample ==== ///
            switch_resample_process(pvt->re_sampler, (int16_t *)data, (int) data_len / 2 / 1);
            memcpy(data, pvt->re_sampler->to, pvt->re_sampler->to_len * 2 * 1);
            data_len = pvt->re_sampler->to_len * 2 * 1;
            if (g_debug) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "ASR new samples:%d\n", pvt->re_sampler->to_len);
            }
        }
        if (pvt->asr_dec_vol) {
            adjustVolume((int16_t *) data, (size_t) data_len / 2, pvt->vol_multiplier);
        }
        if (pvt->started) {
            if (pvt->request->sendAudio((uint8_t*)data, (size_t) data_len) < 0) {
                pvt->stopped = 1;
                // 直接关闭实时音频流识别过程,调用cancel之后不会再上报任何回调事件
                pvt->request->cancel();
                // 释放request对象
                NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                pvt->request = nullptr;
                switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "cancel asr bcs of send audio failed -> on channel: %s\n",
                                  switch_channel_get_name(channel));
                ret_val = false;
                goto unlock;
            }
        } else {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "send_audio_to_ali_asr: aliasr starting, ignore send audio\n");
        }
        ret_val = true;
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "send_audio_to_ali_asr: send audio %d\n",
                              data_len);
        }
    } else {
        switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "try send audio but SpeechTranscriberRequest has been released -> on channel: %s\n",
                          switch_channel_get_name(channel));
        ret_val = false;
    }

    unlock:
    switch_mutex_unlock(pvt->mutex);
    return ret_val;
}

static void stop_ali_asr(ali_asr_context_t *pvt) {
    switch_mutex_lock(pvt->mutex);
    switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
    if (pvt->request) {
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Stop Succeed channel: %s\n",
                              switch_channel_get_name(channel));
        }
        pvt->request->stop();
        // ASR 识别结束, 释放 request 对象
        NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
        pvt->request = nullptr;
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                              "stop ali asr and request is released on channel: %s\n",
                              switch_channel_get_name(channel));
        }
    } else {
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING,
                              "ali asr has already stopped and released on channel:%s\n",
                              switch_channel_get_name(channel));
        }
    }
    switch_mutex_unlock(pvt->mutex);
}

static void cancel_ali_asr(ali_asr_context_t *pvt) {
    switch_mutex_lock(pvt->mutex);
    switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
    if (pvt->request) {
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "try to cancel ali asr on channel: %s\n",
                              switch_channel_get_name(channel));
        }
        // 直接关闭实时音频流识别过程,调用cancel之后不会再上报任何回调事件
        pvt->request->cancel();
        // 释放request对象
        NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
        pvt->request = nullptr;
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                              "cancel ali asr and request is released on channel: %s\n",
                              switch_channel_get_name(channel));
        }
    } else {
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING,
                              "ali asr has already cancelled and released on channel:%s\n",
                              switch_channel_get_name(channel));
        }
    }
    switch_mutex_unlock(pvt->mutex);
}

static void destroy_ali_asr(ali_asr_context_t *pvt) {
    switch_core_session_t *session = pvt->session;
    switch_channel_t *channel = switch_core_session_get_channel(session);
    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(pvt->session), SWITCH_LOG_NOTICE,
                          "destroy_ali_asr: release all resource for session -> on channel: %s\n",
                          switch_channel_get_name(channel));
    }

    cancel_ali_asr(pvt);
    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                          "destroy_ali_asr: cancel_ali_asr -> channel: %s\n",
                          switch_channel_get_name(channel));
    }

    // decrement aliasr concurrent count
    switch_atomic_dec(&ali_asr_globals->ali_asr_concurrent_cnt);

    if (pvt->re_sampler) {
        switch_resample_destroy(&pvt->re_sampler);
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                              "destroy_ali_asr: switch_resample_destroy -> on channel: %s\n",
                              switch_channel_get_name(channel));
        }
    }
    switch_mutex_destroy(pvt->mutex);
    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                          "destroy_ali_asr: switch_mutex_destroy -> on channel: %s\n",
                          switch_channel_get_name(channel));
    }
}

SWITCH_STANDARD_API(aliasr_concurrent_cnt_function) {
    const uint32_t concurrent_cnt = switch_atomic_read (&ali_asr_globals->ali_asr_concurrent_cnt);
    stream->write_function(stream, "%d\n", concurrent_cnt);
    switch_event_t *event = nullptr;
    if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
        event->subclass_name = strdup("aliasr_concurrent_cnt");
        switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
        switch_event_add_header(event, SWITCH_STACK_BOTTOM, "Aliasr-Concurrent-Cnt", "%d", concurrent_cnt);
        switch_event_fire(&event);
    }

    return SWITCH_STATUS_SUCCESS;
}

// ====================================================== TTS ======================================================

typedef struct {
    const char *_save_path;
    const char *_format;
    pthread_mutex_t mtxWord;
    pthread_cond_t cvWord;
    switch_memory_pool_t *pool;
} ali_tts_context_t;

typedef void (*tts_callback_func_t)(NlsEvent *, void *);

std::string timestamp_str() {
    char buf[64];
    struct timeval tv = {0};
    struct tm ltm = {0};

    gettimeofday(&tv, nullptr);
    localtime_r(&tv.tv_sec, &ltm);
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%06ld",
             ltm.tm_year + 1900, ltm.tm_mon + 1, ltm.tm_mday,
             ltm.tm_hour, ltm.tm_min, ltm.tm_sec,
             tv.tv_usec);
    buf[63] = '\0';
    std::string tmp = buf;
    return tmp;
}

/**
 * @brief sdk在接收到云端返回合成结束消息时, sdk内部线程上报Completed事件
 * @note 上报Completed事件之后，SDK内部会关闭识别连接通道.
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void OnSynthesisCompleted(AlibabaNls::NlsEvent* cbEvent, ali_tts_context_t* pvt) {
    std::string ts = timestamp_str();
    if (g_debug) {
        // 获取消息的状态码，成功为0或者20000000，失败时对应失败的错误码
        // 当前任务的task id，方便定位问题，建议输出
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "OnSynthesisCompleted: %s status code=%d, task id=%s\n",
                          ts.c_str(), cbEvent->getStatusCode(), cbEvent->getTaskId());
    }
}

/**
 * @brief 合成过程发生异常时, sdk内部线程上报TaskFailed事件
 * @note 上报TaskFailed事件之后，SDK内部会关闭识别连接通道.
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void OnSynthesisTaskFailed(AlibabaNls::NlsEvent* cbEvent, ali_tts_context_t* pvt) {
    /*
    FILE *failed_stream = fopen("synthesisTaskFailed.log", "a+");
    if (failed_stream) {
        std::string ts = timestamp_str();
        char outbuf[1024] = {0};
        snprintf(outbuf, sizeof(outbuf),
                 "%s status code:%d task id:%s error mesg:%s\n",
                 ts.c_str(),
                 cbEvent->getStatusCode(),
                 cbEvent->getTaskId(),
                 cbEvent->getErrorMessage()
        );
        fwrite(outbuf, strlen(outbuf), 1, failed_stream);
        fclose(failed_stream);
    }
     */


    if (g_debug) {
        // 获取消息的状态码，成功为0或者20000000，失败时对应失败的错误码
        // 当前任务的task id，方便定位问题，建议输出
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "OnSynthesisTaskFailed:status code=%d, task id=%s, error message: %s\n",
                          cbEvent->getStatusCode(), cbEvent->getTaskId(), cbEvent->getErrorMessage());
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "OnSynthesisTaskFailed: All response: %s\n",
                          cbEvent->getAllResponse());
    }
}

/**
 * @brief 识别结束或发生异常时，会关闭连接通道, sdk内部线程上报ChannelCloseed事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void OnSynthesisChannelClosed(AlibabaNls::NlsEvent* cbEvent, ali_tts_context_t* pvt) {
    std::string ts = timestamp_str();
    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "OnSynthesisChannelClosed: %s, All response: %s\n",
                          ts.c_str(), cbEvent->getAllResponse());
    }

    if (pvt) {
        //通知发送线程, 最终识别结果已经返回, 可以调用stop()
        pthread_mutex_lock(&(pvt->mtxWord));
        pthread_cond_signal(&(pvt->cvWord));
        pthread_mutex_unlock(&(pvt->mtxWord));
    }
}

/**
 * @brief 文本上报服务端之后, 收到服务端返回的二进制音频数据, SDK内部线程通过BinaryDataRecved事件上报给用户
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 * @notice 此处切记不可做block操作,只可做音频数据转存. 若在此回调中做过多操作,
 *         会阻塞后续的数据回调和completed事件回调.
 */
void OnBinaryDataRecved(AlibabaNls::NlsEvent* cbEvent, ali_tts_context_t* pvt) {
    std::vector<unsigned char> data = cbEvent->getBinaryData(); // getBinaryData() 获取文本合成的二进制音频数据

    if (data.size() > 0) {
        // 以追加形式将二进制音频数据写入文件
        // std::string dir = pvt->_save_path;
//        if (access(dir.c_str(), 0) == -1) {
//            mkdir(dir.c_str(), S_IRWXU);
//        }
        char file_name[256] = {0};
        snprintf(file_name, 256, "%s/%s.%s", pvt->_save_path, cbEvent->getTaskId(), pvt->_format);

        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "OnBinaryDataRecved: file_name: %s, data.size() %ld\n",
                          file_name, data.size());

        switch_file_handle_t tts_fh = {0};
        if (switch_core_file_open(&tts_fh, file_name, 0, 0, SWITCH_FILE_FLAG_WRITE | SWITCH_FILE_WRITE_APPEND, pvt->pool) != SWITCH_STATUS_SUCCESS) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "OnBinaryDataRecved: can't open file_name: %s for append\n",
                              file_name);
        } else {
            switch_size_t nbytes = data.size();
            switch_core_file_write(&tts_fh, (void*)&data[0], &nbytes);
            switch_core_file_close(&tts_fh);
        }
    }
}

/**
 * @brief 返回 tts 文本对应的日志信息，增量返回对应的字幕信息
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
*/
void OnMetaInfo(AlibabaNls::NlsEvent* cbEvent, ali_tts_context_t* pvt) {
    if (g_debug) {
        //switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "OnMetaInfo: All response: %s\n",
        //                  cbEvent->getAllResponse());
    }
}

/**
 * @brief 服务端返回的所有信息会通过此回调反馈,
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param cbParam 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
*/
void onMessage(AlibabaNls::NlsEvent* cbEvent, ali_tts_context_t* pvt) {
    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onMessage: All response: %s\n",
                          cbEvent->getAllResponse());
    }
}

std::string to_utf8(uint32_t cp) {
    /*
    if using C++11 or later, you can do this:

    std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
    return conv.to_bytes( (char32_t)cp );

    Otherwise...
    */

    std::string result;

    int count;
    if (cp <= 0x007F) {
        count = 1;
    }
    else if (cp <= 0x07FF) {
        count = 2;
    }
    else if (cp <= 0xFFFF) {
        count = 3;
    }
    else if (cp <= 0x10FFFF) {
        count = 4;
    }
    else {
        return result; // or throw an exception
    }

    result.resize(count);

    if (count > 1)
    {
        for (int i = count-1; i > 0; --i)
        {
            result[i] = (char) (0x80 | (cp & 0x3F));
            cp >>= 6;
        }

        for (int i = 0; i < count; ++i)
            cp |= (1 << (7-i));
    }

    result[0] = (char) cp;

    return result;
}

void ues_to_utf8(std::string &ues) {
    std::string::size_type startIdx = 0;
    do {
        startIdx = ues.find("\\u", startIdx);
        if (startIdx == std::string::npos) break;

        std::string::size_type endIdx = ues.find_first_not_of("0123456789abcdefABCDEF", startIdx+2);
        if (endIdx == std::string::npos) {
            endIdx = ues.length() + 1;
        }

        std::string tmpStr = ues.substr(startIdx+2, endIdx-(startIdx+2));
        std::istringstream iss(tmpStr);

        uint32_t cp;
        if (iss >> std::hex >> cp)
        {
            std::string utf8 = to_utf8(cp);
            ues.replace(startIdx, 2+tmpStr.length(), utf8);
            startIdx += utf8.length();
        }
        else {
            startIdx += 2;
        }
    }
    while (true);
}

// uuid_alitts <uuid> text=XXXXX saveto=<path> appkey=<key> url=<url> playback_id=<id>
SWITCH_STANDARD_API(uuid_alitts_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "uuid_alitts: parameter missing.\n");
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "uuid_alitts: parameter missing.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_status_t status = SWITCH_STATUS_SUCCESS;
    char *_text = nullptr;
    char *_saveto = nullptr;
    char *_app_key = nullptr;
    char *_url = nullptr;
    char *_playback_id = nullptr;

    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *my_cmd = switch_core_strdup(pool, cmd);

    char *argv[MAX_API_ARGC];
    memset(argv, 0, sizeof(char *) * MAX_API_ARGC);

    int argc = switch_split(my_cmd, ' ', argv);
    if (g_debug) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", my_cmd, argc);
    }

    if (argc < 1) {
        stream->write_function(stream, "uuid is required.\n");
        switch_goto_status(SWITCH_STATUS_SUCCESS, end);
    }

    for (int idx = 1; idx < MAX_API_ARGC; idx++) {
        if (argv[idx]) {
            char *ss[2] = {nullptr, nullptr};
            int cnt = switch_split(argv[idx], '=', ss);
            if (cnt == 2) {
                char *var = ss[0];
                char *val = ss[1];
                if (g_debug) {
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "process arg: %s = %s\n", var, val);
                }
                if (!strcasecmp(var, "text")) {
                    std::string ues(val);
                    ues_to_utf8(ues);
                    _text = switch_core_strdup(pool, ues.c_str());
                    continue;
                }
                if (!strcasecmp(var, "saveto")) {
                    _saveto = val;
                    continue;
                }
                if (!strcasecmp(var, "appkey")) {
                    _app_key = val;
                    continue;
                }
                if (!strcasecmp(var, "url")) {
                    _url = val;
                    continue;
                }
                if (!strcasecmp(var, "playback_id")) {
                    _playback_id = val;
                    continue;
                }
            }
        }
    }

    {
        time_t now;
        time(&now);
        if (g_expireTime - now < 10) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                              "uuid_alitts: the token will be expired, please generate new token by AccessKey-ID and AccessKey-Secret\n");
            if (-1 == generateToken(g_ak_id, g_ak_secret, &g_token, &g_expireTime)) {
                switch_goto_status(SWITCH_STATUS_SUCCESS, end);
            }
        }

        /*
         * 1. 创建语音识别SpeechSynthesizerRequest对象.
         *
         * 默认为实时短文本语音合成请求, 支持一次性合成300字符以内的文字,
         * 其中1个汉字、1个英文字母或1个标点均算作1个字符,
         * 超过300个字符的内容将会报错(或者截断).
         * 一次性合成超过300字符可考虑长文本语音合成功能.
         *
         * 实时短文本语音合成文档详见: https://help.aliyun.com/document_detail/84435.html
         * 长文本语音合成文档详见: https://help.aliyun.com/document_detail/130509.html
         */
        int chars_cnt = AlibabaNls::NlsClient::getInstance()->calculateUtf8Chars(_text);
        if (g_debug) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " this text contains %d chars\n", chars_cnt);
        }

        AlibabaNls::SpeechSynthesizerRequest* request = nullptr;
        if (chars_cnt > 300) {
            // 长文本语音合成
            request = AlibabaNls::NlsClient::getInstance()->createSynthesizerRequest(AlibabaNls::LongTts);
        } else {
            // 短文本语音合成
            request = AlibabaNls::NlsClient::getInstance()->createSynthesizerRequest();
        }
        if (request == nullptr) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "createSynthesizerRequest failed.\n");
            switch_goto_status(SWITCH_STATUS_SUCCESS, end);
        }

        /*
         * 2. 设置用于接收结果的回调
         */
        // 设置音频合成结束回调函数
        ali_tts_context_t pvt = {0};

        pvt.pool = pool;
        pvt._save_path = _saveto;
        pvt._format = "wav";
        pthread_mutex_init(&pvt.mtxWord, nullptr);
        pthread_cond_init(&pvt.cvWord, nullptr);

        request->setOnSynthesisCompleted(reinterpret_cast<tts_callback_func_t>(OnSynthesisCompleted), &pvt);
        // 设置音频合成通道关闭回调函数
        request->setOnChannelClosed(reinterpret_cast<tts_callback_func_t>(OnSynthesisChannelClosed), &pvt);
        // 设置异常失败回调函数
        request->setOnTaskFailed(reinterpret_cast<tts_callback_func_t>(OnSynthesisTaskFailed), &pvt);
        // 设置文本音频数据接收回调函数
        request->setOnBinaryDataReceived(reinterpret_cast<tts_callback_func_t>(OnBinaryDataRecved), &pvt);
        // 设置字幕信息
        request->setOnMetaInfo(reinterpret_cast<tts_callback_func_t>(OnMetaInfo), &pvt);
        // 设置所有服务端返回信息回调函数
        //request->setOnMessage(onMessage, &cbParam);
        // 开启所有服务端返回信息回调函数, 其他回调(除了OnBinaryDataRecved)失效
        //request->setEnableOnMessage(true);

        /*
         * 3. 设置request的相关参数
         */
        // 设置待合成文本, 必填参数. 文本内容必须为UTF-8编码
        // 一次性合成超过300字符可考虑长文本语音合成功能.
        // 长文本语音合成文档详见: https://help.aliyun.com/document_detail/130509.html
        request->setText(_text);
        // 发音人, 包含"xiaoyun", "ruoxi", "xiaogang"等. 可选参数, 默认是xiaoyun
        // 使用项目中设置的人声配置模型
        // request->setVoice("siqi");
        // 访问个性化音色，访问的Voice必须是个人定制音色
        //request->setPayloadParam("{\"enable_ptts\":true}");
        // 音量, 范围是0~100, 可选参数, 默认50
        request->setVolume(50);
        // 音频编码格式, 可选参数, 默认是wav. 支持的格式pcm, wav, mp3
        request->setFormat("wav");
        // 音频采样率, 包含8000, 16000. 可选参数, 默认是16000
        request->setSampleRate(16000);
        // 语速, 范围是-500~500, 可选参数, 默认是0
        request->setSpeechRate(0);
        // 语调, 范围是-500~500, 可选参数, 默认是0
        request->setPitchRate(0);
        // 开启字幕
        request->setEnableSubtitle(true);

        // 设置AppKey, 必填参数, 请参照官网申请
        request->setAppKey(_app_key);
        // 设置账号校验token, 必填参数
        request->setToken(g_token.c_str());

        if (_url != nullptr) {
            request->setUrl(_url);
        }
        // 设置链接超时500ms
        request->setTimeout(500);
        // 获取返回文本的编码格式
        const char* output_format = request->getOutputFormat();
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "text format: %s\n", output_format);

        /*
         * 4. start()为异步操作。成功则开始返回BinaryRecv事件。失败返回TaskFailed事件。
         */
        int ret = request->start();
        if (ret < 0) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "start failed.\n");
            // start()失败，释放request对象
            AlibabaNls::NlsClient::getInstance()->releaseSynthesizerRequest(request);
            switch_goto_status(SWITCH_STATUS_SUCCESS, end);
        } else {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "start success.\n");
        }
        /*
         * 5. start成功，开始等待接收完所有合成数据。
         *    stop()为无意义接口，调用与否都会跑完全程.
         *    cancel()立即停止工作, 且不会有回调返回, 失败返回TaskFailed事件。
         */
//    ret = request->cancel();
        ret = request->stop();  // always return 0

        /*
         * 开始等待接收完所有合成数据。
         */
        struct timeval tv_now = {0};
        struct timespec tv_outtime = {0};
        if (ret == 0) {
            gettimeofday(&tv_now, nullptr);
            if (g_debug) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "wait closed callback: tv %ld\n", tv_now.tv_sec);
            }
            /*
             * 根据文本长短、接收速度和网络环境，接收完所有合成数据的时间无法确定，
             * 这里设定30s超时只是展示一种超时机制。
             */
            tv_outtime.tv_sec = tv_now.tv_sec + 30;
            tv_outtime.tv_nsec = tv_now.tv_usec * 1000;
            // 等待closed事件后再进行释放, 否则会出现崩溃
            pthread_mutex_lock(&(pvt.mtxWord));
            if (ETIMEDOUT == pthread_cond_timedwait(&(pvt.cvWord), &(pvt.mtxWord), &tv_outtime)) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "synthesis timeout.\n");
            }
            pthread_mutex_unlock(&(pvt.mtxWord));
        } else {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "stop return value : %d.\n", ret);
        }
        gettimeofday(&tv_now, nullptr);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "current request task_id: %s, stop finished, tv: %ld\n",
                          request->getTaskId(), tv_now.tv_sec);

        switch_core_session_t *ses = switch_core_session_force_locate(argv[0]);
        if (!ses) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "alitts failed, can't found session by %s\n", argv[0]);
        } else {
            char *app_arg = switch_core_sprintf(switch_core_session_get_pool(ses),
                                                "{vars_playback_id=%s}%s/%s.%s",
                                                _playback_id, _saveto, request->getTaskId(), "wav" );
            switch_core_session_execute_application_async(ses, "playback", app_arg);
            // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
            //  We meet : ... Locked, Waiting on external entities
            switch_core_session_rwunlock(ses);
        }
        /*
         * 6. 完成所有工作后释放当前请求。
         *    请在closed事件(确定完成所有工作)后再释放, 否则容易破坏内部状态机, 会强制卸载正在运行的请求。
         */
        AlibabaNls::NlsClient::getInstance()->releaseSynthesizerRequest(request);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "release Synthesizer success.\n");
    }

end:
    switch_core_destroy_memory_pool(&pool);
    return status;
}

#define ALIASR_DEBUG_SYNTAX "<on|off>"
SWITCH_STANDARD_API(mod_aliasr_debug)
{
    if (zstr(cmd)) {
        stream->write_function(stream, "-USAGE: %s\n", ALIASR_DEBUG_SYNTAX);
    } else {
        if (!strcasecmp(cmd, "on")) {
            g_debug = true;
            stream->write_function(stream, "Aliasr Debug: on\n");
        } else if (!strcasecmp(cmd, "off")) {
            g_debug = false;
            stream->write_function(stream, "Aliasr Debug: off\n");
        } else {
            stream->write_function(stream, "-USAGE: %s\n", ALIASR_DEBUG_SYNTAX);
        }
    }
    return SWITCH_STATUS_SUCCESS;
}

/**
 *  定义load函数，加载时运行
 */
SWITCH_MODULE_LOAD_FUNCTION(mod_aliasr_load) {
    if (load_config(pool) != SWITCH_STATUS_SUCCESS) {
        return SWITCH_STATUS_FALSE;
    }
    int ret = AlibabaNls::NlsClient::getInstance()->setLogConfig(
            "/usr/local/freeswitch/log/nls-transcriber", AlibabaNls::LogDebug, 400, 50);
    if (-1 == ret) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "set log failed\n");
        return SWITCH_STATUS_FALSE;
    }
    NlsClient::getInstance()->startWorkThread(4);


    switch_api_interface_t *api_interface;
    *module_interface = switch_loadable_module_create_module_interface(pool, modname);

    ali_asr_globals = (ali_asr_global_t *)switch_core_alloc(pool, sizeof(ali_asr_global_t));

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "mod_aliasr_load start\n");

    // register global state handlers
    switch_core_add_state_handler(&ali_asr_cs_handlers);

    SWITCH_ADD_API(api_interface,
                   "aliasr_concurrent_cnt",
                   "aliasr_concurrent_cnt api",
                   aliasr_concurrent_cnt_function,
                   "<cmd><args>");

    SWITCH_ADD_API(api_interface, "aliasr_debug", "Set aliasr debug", mod_aliasr_debug, ALIASR_DEBUG_SYNTAX);

    SWITCH_ADD_API(api_interface, "uuid_alitts", "Invoke Ali TTS", uuid_alitts_function, "<uuid> text=XXXXX saveto=<path> appkey=<key>");

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "mod_aliasr_load\n");

    return SWITCH_STATUS_SUCCESS;
}
/**
 *  定义shutdown函数，关闭时运行
 */
SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_aliasr_shutdown) {

    // unregister global state handlers
    switch_core_remove_state_handler(&ali_asr_cs_handlers);

    NlsClient::releaseInstance();

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " mod_aliasr_shutdown called\n");

    return SWITCH_STATUS_SUCCESS;
}
