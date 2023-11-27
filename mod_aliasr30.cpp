#include <switch.h>
#include <cmath>
#include "nlsClient.h"
#include "nlsEvent.h"
#include "speechTranscriberRequest.h"
#include "nlsCommonSdk/Token.h"

#define MAX_FRAME_BUFFER_SIZE (1024*1024) //1MB
#define SAMPLE_RATE 8000

using namespace AlibabaNlsCommon;
using AlibabaNls::NlsClient;
using AlibabaNls::NlsEvent;
using AlibabaNls::LogDebug;
using AlibabaNls::SpeechTranscriberRequest;


const char *g_ak_id = nullptr;
const char *g_ak_secret = nullptr;
std::string g_token = "";
long g_expireTime = -1;
bool g_debug = false;

// public declare

typedef void *(*rar_init_func_t) (switch_core_session_t *, const switch_codec_implementation_t *, const char *);
typedef void (*rar_record_func_t) (void *, switch_time_t from_answered, const switch_frame_t *frame);

typedef struct {
    rar_init_func_t rar_init_func;
    rar_record_func_t rar_record_func;
} record_replay_t;

typedef void (*on_asr_started_func_t) (void *);
typedef void (*on_asr_sentence_func_t) (void *, const char *sentence);
typedef void (*on_asr_result_changed_func_t) (void *, const char *result);
typedef void (*on_asr_stopped_func_t) (void *);

typedef struct {
    void *asr_caller;
    on_asr_started_func_t on_asr_started_func;
    on_asr_sentence_func_t on_asr_sentence_func;
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

#if 0
typedef struct  {
    char *caller;
    char *callee;
    char *unique_id;
    asr_callback_t *asr_callback;
} asr_context_t;
#endif

//======================================== ali asr start ===============

typedef struct {
    switch_core_session_t *session;
    switch_media_bug_t *bug;
    SpeechTranscriberRequest *request;
    int started;
    int stopped;
    int starting;
    switch_mutex_t *mutex;
    switch_audio_resampler_t *re_sampler;
    char *app_key;
    char *nls_url;
    char *asr_dec_vol;
    float vol_multiplier;
    void *rar_data;
    record_replay_t *rar_funcs;
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
 * @param asr_context 回调自定义参数，默认为 nullptr, 可以根据需求自定义参数
 */
void onAsrTranscriptionStarted(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrTranscriptionStarted 调用时，确认 ali_asr_context_t * 有效

//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: %s\n", asr_context->unique_id);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: status code=%d, task id=%s\n",
                      cbEvent->getStatusCode(), cbEvent->getTaskId());
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: all response=%s\n",
                      cbEvent->getAllResponse());

    switch_mutex_lock(pvt->mutex);
    pvt->started = 1;
    pvt->starting = 0;
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "I need lock!!!!!!!!!!!! \n");
    switch_mutex_unlock(pvt->mutex);

    if (pvt->asr_callback) {
        pvt->asr_callback->on_asr_started_func(pvt->asr_callback->asr_caller);
    }
}

/**
 * @brief 服务端检测到了一句话的开始, sdk内部线程上报SentenceBegin事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param asr_context 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrSentenceBegin(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrSentenceBegin 调用时，确认 ali_asr_context_t * 有效

//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceBegin: %s\n", asr_context->unique_id);
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
 * @param asr_context 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrSentenceEnd(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrSentenceEnd 调用时，确认 ali_asr_context_t * 有效

//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceEnd: %s\n", asr_context->unique_id);
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
    if (pvt->asr_callback) {
        char *sentence = dupAsrResult(cbEvent->getAllResponse());
        pvt->asr_callback->on_asr_sentence_func(pvt->asr_callback->asr_caller, sentence);
        free(sentence);
    }
#if 0
    else {
        switch_event_t *event = nullptr;
        switch_core_session_t *ses = switch_core_session_force_locate(asr_context->unique_id);
        if (ses) {
            switch_channel_t *channel = switch_core_session_get_channel(ses);
            if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
                event->subclass_name = (char *) malloc(strlen("start_asr_") + strlen(asr_context->unique_id) + 1);
                strcpy(event->subclass_name, "start_asr_");
                strcat(event->subclass_name, asr_context->unique_id);
                switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
                switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", asr_context->unique_id);

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
#endif
}

/**
 * @brief 识别结果发生了变化, sdk在接收到云端返回到最新结果时,
 *        sdk内部线程上报ResultChanged事件
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param asr_context 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 */
void onAsrTranscriptionResultChanged(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrTranscriptionResultChanged 调用时，确认 ali_asr_context_t * 有效

//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionResultChanged: %s\n", asr_context->unique_id);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrTranscriptionResultChanged: status code=%d, task id=%s, index=%d, time=%d, result=%s\n",
                      cbEvent->getStatusCode(), cbEvent->getTaskId(),
                      cbEvent->getSentenceIndex(),
                      cbEvent->getSentenceTime(),
                      cbEvent->getResult()
    );
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTranscriptionResultChanged: all response=%s\n",
                      cbEvent->getAllResponse());
    if (pvt->asr_callback) {
        char *result = dupAsrResult(cbEvent->getAllResponse());
        pvt->asr_callback->on_asr_result_changed_func(pvt->asr_callback->asr_caller, result);
        free(result);
    }
#if 0
    else {
        switch_event_t *event = nullptr;
        switch_core_session_t *ses = switch_core_session_force_locate(asr_context->unique_id);
        if (ses) {
            switch_channel_t *channel = switch_core_session_get_channel(ses);
            if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
                event->subclass_name = strdup("update_asr");
                switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
                switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", asr_context->unique_id);

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
#endif
}

/**
 * @brief 服务端停止实时音频流识别时, sdk内部线程上报Completed事件
 * @note 上报Completed事件之后，SDK内部会关闭识别连接通道.
         此时调用sendAudio会返回负值, 请停止发送.
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param asr_context 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void onAsrTranscriptionCompleted(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrTranscriptionCompleted 调用时，确认 ali_asr_context_t * 有效

//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionCompleted: %s\n", asr_context->unique_id);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrTranscriptionCompleted: status code=%d, task id=%s\n", cbEvent->getStatusCode(),
                      cbEvent->getTaskId());
#if 0
    ali_asr_context_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(asr_context->unique_id);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if ((pvt = (ali_asr_context_t *) switch_channel_get_private(channel, "asr"))) {
            //        if(pvt->frameDataBuffer){
            //            free(pvt->frameDataBuffer);
            //        }
        }
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }
#endif
}

/**
 * @brief 识别过程(包含start(), sendAudio(), stop())发生异常时, sdk内部线程上报TaskFailed事件
 * @note 上报TaskFailed事件之后, SDK内部会关闭识别连接通道. 此时调用sendAudio会返回负值, 请停止发送
 * @param cbEvent 回调事件结构, 详见nlsEvent.h
 * @param asr_context 回调自定义参数，默认为NULL, 可以根据需求自定义参数
 * @return
 */
void onAsrTaskFailed(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrTaskFailed 调用时，确认 ali_asr_context_t * 有效

//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTaskFailed: %s\n", asr_context->unique_id);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                      "onAsrTaskFailed: status code=%d, task id=%s, error message=%s\n", cbEvent->getStatusCode(),
                      cbEvent->getTaskId(), cbEvent->getErrorMessage());
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTaskFailed: all response=%s\n",
                      cbEvent->getAllResponse());

    switch_mutex_lock(pvt->mutex);
    pvt->started = 0;
    switch_mutex_unlock(pvt->mutex);
#if 0
    ali_asr_context_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(asr_context->unique_id);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if ((pvt = (ali_asr_context_t *) switch_channel_get_private(channel, "asr"))) {
            switch_mutex_lock(pvt->mutex);
            pvt->started = 0;
            switch_mutex_unlock(pvt->mutex);
        }
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    }
#endif
}

/**
 * @brief 二次结果返回回调函数, 开启enable_nlp后返回
 * 
 * @param cbEvent 
 * @param asr_context
 */
void onAsrSentenceSemantics(NlsEvent *cbEvent, ali_asr_context_t *pvt) {
    // cancel_and_release_ali_asr_on_channel_destroy 回调中，会调用 cancel_ali_asr 来确保 session 结束后，
    // 不会有任何 ASR 事件上报（SpeechTranscriberRequest.cancel: 直接关闭实时音频流识别过程,调用 cancel 之后不会再上报任何回调事件）
    // 因此，当 onAsrSentenceSemantics 调用时，确认 ali_asr_context_t * 有效

//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceSemantics: %s\n", asr_context->unique_id);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrSentenceSemantics: all response=%s\n",
                      cbEvent->getAllResponse());
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
#if 0
    else {
        switch_event_t *event = nullptr;
        if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS) {
            event->subclass_name = strdup("stop_asr");
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Close", cbEvent->getResult());
            switch_event_fire(&event);
        }
    }
    // release asr_context_t
    switch_safe_free(asr_context->callee);
    switch_safe_free(asr_context->caller);
    switch_safe_free(asr_context->unique_id);
    switch_safe_free(asr_context);
#endif
}

/**
 * @brief asr请求构建
 * 
 * @param asr_context
 * @return SpeechTranscriberRequest* 
 */
SpeechTranscriberRequest *generateAsrRequest(/*asr_context_t *asr_context, */ali_asr_context_t *pvt) {
    time_t now;
    time(&now);
    if (g_expireTime - now < 10) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,
                          "the token will be expired, please generate new token by AccessKey-ID and AccessKey-Secret\n");
        if (-1 == generateToken(g_ak_id, g_ak_secret, &g_token, &g_expireTime)) {
            return nullptr;
        }
    }
    SpeechTranscriberRequest *request = NlsClient::getInstance()->createTranscriberRequest();
    if (request == nullptr) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "createTranscriberRequest failed.\n");
        return nullptr;
    }
#if 0
    request->setOnTranscriptionStarted(reinterpret_cast<asr_callback_func_t>(onAsrTranscriptionStarted), asr_context);
    // 设置识别启动回调函数
    request->setOnTranscriptionResultChanged(reinterpret_cast<asr_callback_func_t>(onAsrTranscriptionResultChanged), asr_context);
    // 设置识别结果变化回调函数
    request->setOnTranscriptionCompleted(reinterpret_cast<asr_callback_func_t>(onAsrTranscriptionCompleted), asr_context);
    // 设置语音转写结束回调函数
    request->setOnSentenceBegin(reinterpret_cast<asr_callback_func_t>(onAsrSentenceBegin), asr_context);
    // 设置一句话开始回调函数
    request->setOnSentenceEnd(reinterpret_cast<asr_callback_func_t>(onAsrSentenceEnd), asr_context);
    // 设置一句话结束回调函数
    request->setOnTaskFailed(reinterpret_cast<asr_callback_func_t>(onAsrTaskFailed), asr_context);
    // 设置异常识别回调函数
    request->setOnChannelClosed(reinterpret_cast<asr_callback_func_t>(onAsrChannelClosed), asr_context);
    // 设置识别通道关闭回调函数
    request->setOnSentenceSemantics(reinterpret_cast<asr_callback_func_t>(onAsrSentenceSemantics), asr_context);
    //设置二次结果返回回调函数, 开启enable_nlp后返回
#else
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
#endif
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
SWITCH_MODULE_DEFINITION(mod_aliasr, mod_aliasr_load, mod_aliasr_shutdown, nullptr);
};

static void *init_ali_asr(switch_core_session_t *session, const switch_codec_implementation_t *read_impl, const char *cmd);

static bool start_ali_asr(ali_asr_context_t *pvt, asr_callback_t *asr_callback);

static bool send_audio_to_ali_asr(ali_asr_context_t *pvt, void *data, uint32_t data_len);

static void stop_ali_asr(ali_asr_context_t *pvt);

static void cancel_ali_asr(ali_asr_context_t *pvt);

static void destroy_ali_asr(ali_asr_context_t *pvt);

static const asr_provider_t g_funcs = {
        init_ali_asr,
        reinterpret_cast<asr_start_func_t>(start_ali_asr),
        reinterpret_cast<asr_send_audio_func_t>(send_audio_to_ali_asr),
        reinterpret_cast<asr_stop_func_t>(cancel_ali_asr),
        reinterpret_cast<asr_destroy_func_t>(destroy_ali_asr)
};

static switch_status_t attach_ali_asr_provider_on_channel_init(switch_core_session_t *session) {
    switch_channel_t *channel = switch_core_session_get_channel(session);
    switch_channel_set_private(channel, "ali_asr", &g_funcs);
    return SWITCH_STATUS_SUCCESS;
}

switch_state_handler_table_t global_cs_handlers = {
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

static void adjustVolume(int16_t *pcm, size_t pcmlen, float vol_multiplier) {
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

#if 0
static void handleABCTypeRead(switch_media_bug_t *bug, ali_asr_context_t *pvt, switch_channel_t *channel) {
    uint8_t data[SWITCH_RECOMMENDED_BUFFER_SIZE];
    switch_frame_t frame = {0};
    frame.data = data;
    frame.buflen = sizeof(data);
    if (switch_core_media_bug_read(bug, &frame, SWITCH_FALSE) != SWITCH_STATUS_FALSE) {
        switch_mutex_lock(pvt->mutex);

        if (pvt->rar_funcs) {
            switch_channel_timetable_t *times = switch_channel_get_timetable(channel);

            // channel->caller_profile->times->answered = switch_micro_time_now();
            // https://github.com/signalwire/freeswitch/blob/792eee44d0611422cce3c3194f95125916a7d268/src/switch_channel.c#L3834C3-L3834C70
            pvt->rar_funcs->rar_record_func(pvt->rar_data, switch_micro_time_now() - times->answered, &frame);
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

static void handleABCTypeClose(ali_asr_context_t *pvt, switch_channel_t *channel) {
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

static void handleABCTypeInit(ali_asr_context_t *pvt, switch_channel_t *channel) {
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
#if 0
            auto asr_context = (asr_context_t*)malloc(sizeof(asr_context_t));
            asr_context->unique_id = strdup(switch_channel_get_uuid(channel));
            switch_caller_profile_t *profile = switch_channel_get_caller_profile(channel);
            asr_context->caller = strdup(profile->caller_id_number);
            asr_context->callee = strdup(profile->callee_id_number);
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Caller %s. Callee %s\n",
                              asr_context->caller, asr_context->callee);
            SpeechTranscriberRequest *request = generateAsrRequest(asr_context, pvt);
#else
            SpeechTranscriberRequest *request = generateAsrRequest(nullptr, pvt);
#endif
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
static switch_bool_t asr_context(switch_media_bug_t *bug, void *user_data, switch_abc_type_t type) {
    auto *pvt = (ali_asr_context_t *) user_data;
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
#endif

#if 0
static switch_status_t cancel_and_release_ali_asr_on_channel_destroy(switch_core_session_t *session) {
    ali_asr_context_t *pvt;
    switch_channel_t *channel = switch_core_session_get_channel(session);
    switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                      "%s on_destroy, release all resource for session\n",
                      switch_channel_get_name(channel));

    if ((pvt = (ali_asr_context_t *) switch_channel_get_private(channel, ASR_PVT_NAME))) {
        switch_channel_set_private(channel, ASR_PVT_NAME, nullptr);

        cancel_ali_asr(pvt);
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                          "on_destroy: cancel_ali_asr -> channel: %s\n",
                          switch_channel_get_name(channel));

        if (pvt->re_sampler) {
            switch_resample_destroy(&pvt->re_sampler);
            switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                              "%s on_destroy: switch_resample_destroy\n",
                              switch_channel_get_name(channel));
        }
        switch_mutex_destroy(pvt->mutex);
        switch_core_destroy_memory_pool(&pvt->pool);
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                          "%s on_destroy: switch_mutex_destroy & switch_core_destroy_memory_pool\n",
                          switch_channel_get_name(channel));
    }
    return SWITCH_STATUS_SUCCESS;
}

switch_state_handler_table_t session_asr_handlers = {
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
        cancel_and_release_ali_asr_on_channel_destroy,
        // int flags;
        0
};
#endif

#define MAX_API_ARGC 10

static void *init_ali_asr(switch_core_session_t *session, const switch_codec_implementation_t *read_impl, const char *cmd) {
    char *_app_key = nullptr;
    char *_nls_url = nullptr;
    char *_asr_dec_vol = nullptr;

    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *my_cmd = switch_core_strdup(pool, cmd);

    char *argv[MAX_API_ARGC];
    memset(argv, 0, sizeof(char *) * MAX_API_ARGC);

    int argc = switch_split(my_cmd, ' ', argv);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", my_cmd, argc);

    for (int idx = 1; idx < MAX_API_ARGC; idx++) {
        if (argv[idx]) {
            char *ss[2] = {nullptr, nullptr};
            int cnt = switch_split(argv[idx], '=', ss);
            if (cnt == 2) {
                char *var = ss[0];
                char *val = ss[1];
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "process arg: %s = %s\n", var, val);
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
    }

    end:
    switch_core_destroy_memory_pool(&pool);
    return pvt;
}

static bool start_ali_asr(ali_asr_context_t *pvt, asr_callback_t *asr_callback) {
    bool  ret_val = false;
    switch_mutex_lock(pvt->mutex);
    if (pvt->started == 0) {
        if (pvt->starting == 0) {
            pvt->starting = 1;
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Starting Transaction \n");
            switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
            pvt->asr_callback = asr_callback;
#if 0
            auto asr_context = (asr_context_t*)malloc(sizeof(asr_context_t));
            asr_context->unique_id = strdup(switch_channel_get_uuid(channel));
            switch_caller_profile_t *profile = switch_channel_get_caller_profile(channel);
            asr_context->caller = strdup(profile->caller_id_number);
            asr_context->callee = strdup(profile->callee_id_number);
            asr_context->asr_callback = asr_callback;
            SpeechTranscriberRequest *request = generateAsrRequest(asr_context, pvt);
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Caller %s. Callee %s\n",
                              asr_context->caller, asr_context->callee);
#else
            SpeechTranscriberRequest *request = generateAsrRequest(pvt);
#endif
            if (!request) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "Asr Request init failed.%s\n",
                                  switch_channel_get_name(channel));
                ret_val = false;
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
                pvt->request = nullptr;
                ret_val = false;
                goto unlock;
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
        if (pvt->request->sendAudio((uint8_t*)data, (size_t) data_len) < 0) {
            pvt->stopped = 1;
            switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "send audio failed -> on channel: %s\n",
                              switch_channel_get_name(channel));
            ret_val = false;
            goto unlock;
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
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Stop Succeed channel: %s\n",
                          switch_channel_get_name(channel));
        pvt->request->stop();
        //7: 识别结束, 释放request对象
        NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
        pvt->request = nullptr;
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "stop ali asr and request is released on channel: %s\n",
                          switch_channel_get_name(channel));
    } else {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "ali asr has already stopped and released on channel:%s\n",
                          switch_channel_get_name(channel));
    }
    switch_mutex_unlock(pvt->mutex);
}

static void cancel_ali_asr(ali_asr_context_t *pvt) {
    switch_mutex_lock(pvt->mutex);
    switch_channel_t *channel = switch_core_session_get_channel(pvt->session);
    if (pvt->request) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "try to cancel ali asr on channel: %s\n",
                          switch_channel_get_name(channel));
        // 直接关闭实时音频流识别过程,调用cancel之后不会再上报任何回调事件
        pvt->request->cancel();
        // 释放request对象
        NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
        pvt->request = nullptr;
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "cancel ali asr and request is released on channel: %s\n",
                          switch_channel_get_name(channel));
    } else {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "ali asr has already cancelled and released on channel:%s\n",
                          switch_channel_get_name(channel));
    }
    switch_mutex_unlock(pvt->mutex);
}

static void destroy_ali_asr(ali_asr_context_t *pvt) {
    switch_core_session_t *session = pvt->session;
    switch_channel_t *channel = switch_core_session_get_channel(session);
    switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(pvt->session), SWITCH_LOG_NOTICE,
                      "destroy_ali_asr: release all resource for session -> on channel: %s\n",
                      switch_channel_get_name(channel));

    cancel_ali_asr(pvt);
    switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                      "destroy_ali_asr: cancel_ali_asr -> channel: %s\n",
                      switch_channel_get_name(channel));

    if (pvt->re_sampler) {
        switch_resample_destroy(&pvt->re_sampler);
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                          "destroy_ali_asr: switch_resample_destroy -> on channel: %s\n",
                          switch_channel_get_name(channel));
    }
    switch_mutex_destroy(pvt->mutex);
    switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_NOTICE,
                      "destroy_ali_asr: switch_mutex_destroy -> on channel: %s\n",
                      switch_channel_get_name(channel));
}

#if 0
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

        ali_asr_context_t *pvt;
        if (!(pvt = (ali_asr_context_t *) switch_core_session_alloc(ses, sizeof(ali_asr_context_t)))) {
            switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
        }
        pvt->started = 0;
        pvt->stopped = 0;
        pvt->starting = 0;
//        pvt->data_len = 0;
        pvt->session = ses;
        pvt->app_key = switch_core_session_strdup(pvt->session, _appkey);
        pvt->nls_url = switch_core_session_strdup(pvt->session, _nlsurl);
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

        if (_savepcm) {
            pvt->rar_funcs = (record_replay_t*)switch_channel_get_private(channel, "record_and_replay");
            if (pvt->rar_funcs) {
                pvt->rar_data = pvt->rar_funcs->rar_init_func(pvt->session, &read_impl, _savepcm);
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "enable record and replay feature by %s\n", _savepcm);
            } else {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "can't found rar_funcs, Disable record and replay feature!\n");
            }
        } else {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "can't found savepcm, Disable record and replay feature!\n");
        }

        //session添加media bug
        if ((status = switch_core_media_bug_add(ses, "asr", nullptr,
                                                asr_context, pvt, 0,
                // SMBF_READ_REPLACE | SMBF_WRITE_REPLACE |  SMBF_NO_PAUSE | SMBF_ONE_ONLY,
                                                SMBF_READ_STREAM | SMBF_NO_PAUSE,
                                                &(pvt->bug))) != SWITCH_STATUS_SUCCESS) {
            switch_goto_status(SWITCH_STATUS_SUCCESS, unlock);
        }
        switch_channel_set_private(channel, "asr", pvt);

        if (switch_channel_add_state_handler(channel, &session_asr_handlers) < 0) {
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
        ali_asr_context_t *pvt;
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if ((pvt = (ali_asr_context_t *) switch_channel_get_private(channel, "asr"))) {
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
#endif

/**
 *  定义load函数，加载时运行
 */
SWITCH_MODULE_LOAD_FUNCTION(mod_aliasr_load) {
    if (load_config(pool) != SWITCH_STATUS_SUCCESS) {
        return SWITCH_STATUS_FALSE;
    }
    int ret = NlsClient::getInstance()->setLogConfig("log-transcriber", LogDebug);
    if (-1 == ret) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "set log failed\n");
        return SWITCH_STATUS_FALSE;
    }
    NlsClient::getInstance()->startWorkThread(4);

//    switch_api_interface_t *api_interface = nullptr;
    *module_interface = switch_loadable_module_create_module_interface(pool, modname);

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "mod_aliasr_load start\n");
#if 0
    // register API
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
#endif

    // register global state handlers
    switch_core_add_state_handler(&global_cs_handlers);

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "mod_aliasr_load\n");

    return SWITCH_STATUS_SUCCESS;
}
/**
 *  定义shutdown函数，关闭时运行
 */
SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_aliasr_shutdown) {

    // unregister global state handlers
    switch_core_remove_state_handler(&global_cs_handlers);

    NlsClient::releaseInstance();

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " mod_aliasr_shutdown called\n");

    return SWITCH_STATUS_SUCCESS;
}
