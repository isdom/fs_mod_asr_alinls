#include <switch.h>
#include <fstream>
#include <math.h>
#include <sys/time.h>

#define ASIO_STANDALONE 1
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/config/asio_client.hpp>
#include <atomic>
#include <thread>

#include "nlohmann/json.hpp"

template <typename T> class WebsocketClient;

typedef WebsocketClient<websocketpp::config::asio_tls_client> funasr_client;

struct AsrParamCallBack
{
    std::string caller;
    std::string callee;
    char *sUUID ;
};

typedef struct
{
    switch_core_session_t   *session;
    switch_media_bug_t      *bug;
    funasr_client           *fac;
    int                     started;
    int                     stoped;
    int                     starting;
    int                     datalen;
    switch_mutex_t          *mutex;
    switch_memory_pool_t    *pool;
    switch_audio_resampler_t *resampler;
    char                    *asrurl;
} switch_da_t;

/**
 * 识别启动回调函数
 *
 * @param cbEvent
 * @param cbParam
 */
void onAsrTranscriptionStarted(AsrParamCallBack *cbParam)
{
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "onAsrTranscriptionStarted: %s\n", cbParam->sUUID);
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionStarted: status code=%d, task id=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId());
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionStarted: all response=%s\n", cbEvent->getAllResponse());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(cbParam->sUUID);
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
void onAsrSentenceBegin(AsrParamCallBack *cbParam)
{
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceBegin: %s\n", cbParam->sUUID);
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceBegin: status code=%d, task id=%s, index=%d, time=%d\n", cbEvent->getStatusCode(), cbEvent->getTaskId(),
//                    cbEvent->getSentenceIndex(),
//                    cbEvent->getSentenceTime());
}

/**
 * @brief 一句话结束回调函数
 *
 * @param cbEvent
 * @param cbParam
 */
void onAsrSentenceEnd(AsrParamCallBack *cbParam, const std::string &text)
{
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceEnd: %s\n", cbParam->sUUID);
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrSentenceEnd: status code=%d, task id=%s, index=%d, time=%d, begin_time=%d, result=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId(),
//                    cbEvent->getSentenceIndex(),
//                    cbEvent->getSentenceTime(),
//                    cbEvent->getSentenceBeginTime(),
//                    cbEvent->getResult()
//                    );
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrSentenceEnd: all response=%s\n", cbEvent->getAllResponse());
    switch_event_t *event = NULL;
    switch_core_session_t *ses = switch_core_session_force_locate(cbParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if(switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS)
        {
            event->subclass_name = (char*)malloc(strlen("start_asr_") + strlen(cbParam->sUUID) + 1);
            strcpy(event->subclass_name, "start_asr_");
            strcat(event->subclass_name, cbParam->sUUID);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", cbParam->sUUID);
    
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Response", text.c_str());
    
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Channel", switch_channel_get_name(channel));
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
void onAsrTranscriptionResultChanged(AsrParamCallBack *cbParam, const std::string &text)
{
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionResultChanged: %s\n", cbParam->sUUID);
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionResultChanged: status code=%d, task id=%s, index=%d, time=%d, result=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId(),
//                    cbEvent->getSentenceIndex(),
//                    cbEvent->getSentenceTime(),
//                    cbEvent->getResult()
//                    );
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTranscriptionResultChanged: all response=%s\n", cbEvent->getAllResponse());
    switch_event_t *event = NULL;
    switch_core_session_t *ses = switch_core_session_force_locate(cbParam->sUUID);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if (switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS)
        {
            event->subclass_name = strdup("update_asr");
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Unique-ID", cbParam->sUUID);
    
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Response", text.c_str());
            
            switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Channel", switch_channel_get_name(channel));
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
void onAsrTranscriptionCompleted(AsrParamCallBack *cbParam)
{
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionCompleted: %s\n", cbParam->sUUID);
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTranscriptionCompleted: status code=%d, task id=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(cbParam->sUUID);
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
void onAsrTaskFailed(AsrParamCallBack *cbParam)
{
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTaskFailed: %s\n", cbParam->sUUID);
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrTaskFailed: status code=%d, task id=%s, error message=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId(), cbEvent->getErrorMessage());
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "onAsrTaskFailed: all response=%s\n", cbEvent->getAllResponse());
    switch_da_t *pvt;
    switch_core_session_t *ses = switch_core_session_force_locate(cbParam->sUUID);
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
 * @brief 识别通道关闭回调函数
 *
 * @param cbEvent
 * @param cbParam
 */
void onAsrChannelClosed(AsrParamCallBack *cbParam)
{
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE,"onAsrChannelClosed: %s\n", cbParam->sUUID);
    switch_event_t *event = NULL;
    if(switch_event_create(&event, SWITCH_EVENT_CUSTOM) == SWITCH_STATUS_SUCCESS)
    {
        event->subclass_name = strdup("stop_asr");
        switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "Event-Subclass", event->subclass_name);
//        switch_event_add_header_string(event, SWITCH_STACK_BOTTOM, "ASR-Close", cbEvent->getResult());
        switch_event_fire(&event);
    }
    delete cbParam;
}

/**
 * Define a semi-cross platform helper method that waits/sleeps for a bit.
 */
void WaitABit() {
#ifdef WIN32
  Sleep(1000);
#else
  usleep(1000);
#endif
}

typedef websocketpp::config::asio_client::message_type::ptr message_ptr;
typedef websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context> context_ptr;

using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;

context_ptr OnTlsInit(websocketpp::connection_hdl) {
    context_ptr ctx = websocketpp::lib::make_shared<asio::ssl::context>(asio::ssl::context::sslv23);

    try {
        ctx->set_options(
            asio::ssl::context::default_workarounds | asio::ssl::context::no_sslv2 |
            asio::ssl::context::no_sslv3 | asio::ssl::context::single_dh_use);

    } catch (std::exception& e) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "OnTlsInit asio::ssl::context::set_options exception: %s\n", e.what());
    }
    return ctx;
}

// template for tls or not config
template <typename T>
class WebsocketClient {
public:
    // typedef websocketpp::client<T> client;
    // typedef websocketpp::client<websocketpp::config::asio_tls_client>
    // wss_client;
    typedef websocketpp::lib::lock_guard<websocketpp::lib::mutex> scoped_lock;

    WebsocketClient(int is_ssl, AsrParamCallBack *cbParam) : m_open(false), m_done(false) {
        m_cbParam = cbParam;
        
        // set up access channels to only log interesting things
        m_client.clear_access_channels(websocketpp::log::alevel::all);
        m_client.set_access_channels(websocketpp::log::alevel::connect);
        m_client.set_access_channels(websocketpp::log::alevel::disconnect);
        m_client.set_access_channels(websocketpp::log::alevel::app);

        // Initialize the Asio transport policy
        m_client.init_asio();
        m_client.start_perpetual();

        // Bind the handlers we are using
        using websocketpp::lib::bind;
        using websocketpp::lib::placeholders::_1;
        m_client.set_open_handler(bind(&WebsocketClient::on_open, this, _1));
        m_client.set_close_handler(bind(&WebsocketClient::on_close, this, _1));

        m_client.set_message_handler(
            [this](websocketpp::connection_hdl hdl, message_ptr msg) {
              on_message(hdl, msg);
            });

        m_client.set_fail_handler(bind(&WebsocketClient::on_fail, this, _1));
        m_client.clear_access_channels(websocketpp::log::alevel::all);
    }

    std::string getThreadIdOfString(const std::thread::id & id)
    {
        std::stringstream sin;
        sin << id;
        return sin.str();
    }
    
    void on_message(websocketpp::connection_hdl hdl, message_ptr msg) {
        const std::string& payload = msg->get_payload();
        switch (msg->get_opcode()) {
        case websocketpp::frame::opcode::text:
            nlohmann::json asrresult = nlohmann::json::parse(payload);
            std::string id_str = getThreadIdOfString(std::this_thread::get_id());
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "thread: %s, on_message = %s\n", id_str.c_str(), payload.c_str());

            if (asrresult["mode"] == "2pass-online") {
                onAsrTranscriptionResultChanged(m_cbParam, asrresult["text"]);
            } else if (asrresult["mode"] == "2pass-offline") {
                onAsrSentenceEnd(m_cbParam, asrresult["text"]);
            }
                
            if (asrresult["is_final"] == true) {
                websocketpp::lib::error_code ec;
       
                m_client.close(hdl, websocketpp::close::status::going_away, "", ec);

                if (ec) {
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Error closing connection: %s\n", ec.message().c_str());
                }
            }
        }
    }

    // This method will block until the connection is complete
    int start(const std::string& uri, std::string asr_mode, std::vector<int> chunk_vector) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "start wsc with: %s mode: %s\n", uri.c_str(), asr_mode.c_str());
        
        {
            // Create a new connection to the given URI
            websocketpp::lib::error_code ec;
            typename websocketpp::client<T>::connection_ptr con = m_client.get_connection(uri, ec);
            if (ec) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "Get Connection Error: %s\n", ec.message().c_str());
                return -1;
            }
            // Grab a handle for this connection so we can talk to it in a thread
            // safe manor after the event loop starts.
            m_hdl = con->get_handle();

            // Queue the connection. No DNS queries or network connections will be
            // made until the io_service event loop is run.
            m_client.connect(con);
        }

        // Create a thread to run the ASIO io_service event loop
        m_thread.reset(new websocketpp::lib::thread(&websocketpp::client<T>::run, &m_client));
        
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "start send wsc first msg\n");
        // first message
        bool wait = false;
        while (1) {
          {
            scoped_lock guard(m_lock);
            // If the connection has been closed, stop generating data
            if (m_done) {
//              break;
                return -1;
            }
            // If the connection hasn't been opened yet wait a bit and retry
            if (!m_open) {
              wait = true;
            } else {
              break;
            }
          }

          if (wait) {
            // LOG(INFO) << "wait.." << m_open;
            WaitABit();
            continue;
          }
        }
        
        {
            nlohmann::json jsonbegin;
            nlohmann::json chunk_size = nlohmann::json::array();
            chunk_size.push_back(chunk_vector[0]);
            chunk_size.push_back(chunk_vector[1]);
            chunk_size.push_back(chunk_vector[2]);
            jsonbegin["mode"] = asr_mode;
            jsonbegin["chunk_size"] = chunk_size;
            jsonbegin["wav_name"] = "asr";
            jsonbegin["wav_format"] = "pcm";
            jsonbegin["is_speaking"] = true;
            
            websocketpp::lib::error_code ec;
            m_client.send(m_hdl, jsonbegin.dump(), websocketpp::frame::opcode::text, ec);
            if (ec) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "funasr send begin msg failed: %s\n", ec.message().c_str());
            } else {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "funasr send begin msg success\n");
            }
        }
        
        return 0;
    }
    
    void stop() {
        {
            nlohmann::json jsonend;
            jsonend["is_speaking"] = false;
            websocketpp::lib::error_code ec;
            m_client.send(m_hdl, jsonend.dump(), websocketpp::frame::opcode::text, ec);
            if (ec) {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "funasr send end msg failed: %s\n", ec.message().c_str());
            } else {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "funasr send end msg success\n");
            }
        }

        m_client.stop_perpetual();
        m_thread->join();
        
        onAsrChannelClosed(m_cbParam);
    }

    // The open handler will signal that we are ready to start sending data
    void on_open(websocketpp::connection_hdl) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Connection opened, starting data!\n");

        {
            scoped_lock guard(m_lock);
            m_open = true;
        }
        onAsrTranscriptionStarted(m_cbParam);
    }

    // The close handler will signal that we should stop sending data
    void on_close(websocketpp::connection_hdl) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Connection closed, stopping data!\n");
        
        {
            scoped_lock guard(m_lock);
            m_done = true;
        }
        onAsrTranscriptionCompleted(m_cbParam);
    }

    // The fail handler will signal that we should stop sending data
    void on_fail(websocketpp::connection_hdl) {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_ERROR, "Connection failed, stopping data!\n");
        
        {
            scoped_lock guard(m_lock);
            m_done = true;
        }
        onAsrTaskFailed(m_cbParam);
    }
    
    void sendAudio(uint8_t *dp, size_t datalen, websocketpp::lib::error_code &ec) {
        m_client.send(m_hdl, dp, datalen, websocketpp::frame::opcode::binary, ec);
    }
    
    websocketpp::client<T> m_client;
    websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;

private:
    
    AsrParamCallBack            *m_cbParam;
    websocketpp::connection_hdl m_hdl;
    websocketpp::lib::mutex m_lock;
    bool m_open;
    bool m_done;
};

// typedef WebsocketClient<websocketpp::config::asio_tls_client> funasr_client;

#define MAX_FRAME_BUFFER_SIZE (1024*1024) //1MB
#define SAMPLE_RATE 16000

bool        g_debug = false;
float       g_vol_multiplier = 1.0f;

funasr_client *generateAsrClient(AsrParamCallBack * cbParam)
{
    funasr_client* fac = new funasr_client(1, cbParam);
    if (fac == NULL)
    {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "generateAsrClient failed.\n" );
        return NULL;
    }
    
    fac->m_client.set_tls_init_handler(bind(&OnTlsInit, ::_1));
    
    /*
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
     */
    
//    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "vol multiplier is:%f\n", g_asrurl.c_str(), g_vol_multiplier);
    return fac;
}


//======================================== freeswitch module start ===============
SWITCH_MODULE_LOAD_FUNCTION(mod_funasr_load);
SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_funasr_shutdown);
extern "C"
{
    SWITCH_MODULE_DEFINITION(mod_funasr, mod_funasr_load, mod_funasr_shutdown, NULL);
}
;

#if 0
/**
 * 配置加载 aliyun的appkey，akid，aksecret
 *
 * @return switch_status_t 执行状态：
 */
static switch_status_t load_config()
{
    const char *cf = "funasr.conf";
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
#endif

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
                    funasr_client* fac = generateAsrClient(cbParam);
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Caller %s. Callee %s\n",cbParam->caller.c_str() , cbParam->callee.c_str() );
                    if(fac == NULL)
                    {
                        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "Asr Client init failed.%s\n", switch_channel_get_name(channel));
                        switch_mutex_unlock(pvt->mutex);
                        return SWITCH_TRUE;
                    }
                    pvt->fac = fac;
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "Init Asr Client.%s\n", switch_channel_get_name(channel));
                    
                    std::vector<int> chunk_size;
                    chunk_size.push_back(5);
                    chunk_size.push_back(10);
                    chunk_size.push_back(5);

                    if (pvt->fac->start(std::string(pvt->asrurl), "2pass", chunk_size) < 0)
                    {
                        pvt->stoped = 1;
                        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "start() failed. may be can not connect server. please check network or firewalld:%s\n", switch_channel_get_name(channel));
                        pvt->fac->stop();
                        delete pvt->fac;
                        pvt->fac = NULL;
                        // NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
                        // start()失败，释放request对象
                    }
                }
            }
            switch_mutex_unlock(pvt->mutex);
        }
        break;
        case SWITCH_ABC_TYPE_CLOSE:
        {
            if (pvt->fac)
            {
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "ASR Stop Succeed channel: %s\n", switch_channel_get_name(channel));
                pvt->fac->stop();
                switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "asr stoped:%s\n", switch_channel_get_name(channel));
                //7: 识别结束, 释放request对象
                delete pvt->fac;
                pvt->fac = NULL;
                // NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
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
                if (read_impl.actual_samples_per_second != 16000)
                {
                    if (!pvt->resampler)
                    {
                        if (switch_resample_create(&pvt->resampler,
                           read_impl.actual_samples_per_second,
                           16000,
                           16 * (read_impl.microseconds_per_packet / 1000) * 2,
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
//                if (g_vol_multiplier != 1.0f) {
//                    adjustVolume((int16_t*)dp, (size_t)datalen / 2);
//                }

                websocketpp::lib::error_code ec;

                pvt->fac->sendAudio((uint8_t *)dp, (size_t)datalen, ec);
                
                if (ec) {
                    pvt->stoped =1;
                    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_NOTICE, "Send Audio Error: %s, channel %s\n", ec.message().c_str(), switch_channel_get_name(channel));
                    pvt->fac->stop();
                    delete pvt->fac;
                    pvt->fac = NULL;
                    // NlsClient::getInstance()->releaseTranscriberRequest(pvt->request);
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

// uuid_start_funasr <uuid> <uri>
SWITCH_STANDARD_API(uuid_start_funasr_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "uuid_start_funasr: parameter missing.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *mycmd = switch_core_strdup(pool, cmd);

    char *argv[2] = {0, 0};
    int argc = switch_split(mycmd, ' ', argv);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", mycmd, argc);

    if (argc < 2) {
        stream->write_function(stream, "parameter number is invalid.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_core_session_t *ses = switch_core_session_force_locate(argv[0]);
    if (ses) {
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_INFO, "starting funasr:%s\n", switch_channel_get_name(channel));
        
        switch_status_t status;
        switch_da_t *pvt;
        switch_codec_implementation_t read_impl;
        
        memset(&read_impl, 0, sizeof(switch_codec_implementation_t));
        
        //获取读媒体编码实现方法
        switch_core_session_get_read_impl(ses, &read_impl);
        if (!(pvt = (switch_da_t*)switch_core_session_alloc(ses, sizeof(switch_da_t)))) {
            goto lab_end;
        }
        pvt->started = 0;
        pvt->stoped = 0;
        pvt->starting = 0;
        pvt->datalen = 0;
        pvt->session = ses;
        pvt->asrurl = strdup(argv[1]);
        if ((status = switch_core_new_memory_pool(&pvt->pool)) != SWITCH_STATUS_SUCCESS) {
            switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_CRIT, "Memory Error!\n");
            goto lab_end;
        }
        switch_mutex_init(&pvt->mutex,SWITCH_MUTEX_NESTED,pvt->pool);
        //session添加media bug
        if ((status = switch_core_media_bug_add(ses, "asr", NULL,
                asr_callback, pvt, 0,
                // SMBF_READ_REPLACE | SMBF_WRITE_REPLACE |  SMBF_NO_PAUSE | SMBF_ONE_ONLY,
                SMBF_READ_STREAM | SMBF_NO_PAUSE,
                &(pvt->bug))) != SWITCH_STATUS_SUCCESS) {
            goto lab_end;
        }
        switch_channel_set_private(channel, "asr", pvt);
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(ses), SWITCH_LOG_INFO, "%s Start ASR\n", switch_channel_get_name(channel));
lab_end:
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    } else {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "start funasr failed, can't found session by %s\n", argv[0]);
    }
    
    return SWITCH_STATUS_SUCCESS;
}

// uuid_stop_funasr <uuid>
SWITCH_STANDARD_API(uuid_stop_funasr_function) {
    if (zstr(cmd)) {
        stream->write_function(stream, "uuid_stop_funasr: parameter missing.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_memory_pool_t *pool;
    switch_core_new_memory_pool(&pool);
    char *mycmd = switch_core_strdup(pool, cmd);

    char *argv[1] = {0};
    int argc = switch_split(mycmd, ' ', argv);
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, "cmd:%s, args count: %d\n", mycmd, argc);

    if (argc < 1) {
        stream->write_function(stream, "parameter number is invalid.\n");
        return SWITCH_STATUS_SUCCESS;
    }

    switch_core_session_t *ses = switch_core_session_force_locate(argv[0]);
    if (ses) {
        switch_da_t *pvt;
        switch_channel_t *channel = switch_core_session_get_channel(ses);
        if ((pvt = (switch_da_t*)switch_channel_get_private(channel, "asr")))  {
            switch_channel_set_private(channel, "asr", NULL);
            switch_core_media_bug_remove(session, &pvt->bug);
            switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(ses), SWITCH_LOG_DEBUG, "%s Stop ASR\n", switch_channel_get_name(channel));
        }
        
        // add rwunlock for BUG: un-released channel, ref: https://blog.csdn.net/xxm524/article/details/125821116
        //  We meet : ... Locked, Waiting on external entities
        switch_core_session_rwunlock(ses);
    } else {
        switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_WARNING, "stop funasr failed, can't found session by %s\n", argv[0]);
    }

    return SWITCH_STATUS_SUCCESS;
}

#if 0
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
 *  定义添加的函数
 */
SWITCH_STANDARD_APP(stop_asr_session_function)
{
    switch_da_t *pvt;
    switch_channel_t *channel = switch_core_session_get_channel(session);
    if ((pvt = (switch_da_t*)switch_channel_get_private(channel, "asr"))) {
        switch_channel_set_private(channel, "asr", NULL);
        switch_core_media_bug_remove(session, &pvt->bug);
        switch_log_printf(SWITCH_CHANNEL_SESSION_LOG(session), SWITCH_LOG_DEBUG, "%s Stop ASR\n", switch_channel_get_name(channel));
    }
}
#endif

/**
 *  定义load函数，加载时运行
 */
SWITCH_MODULE_LOAD_FUNCTION(mod_funasr_load)
{
    switch_api_interface_t* api_interface = NULL;
    *module_interface =switch_loadable_module_create_module_interface(pool, modname);

    switch_log_printf(SWITCH_CHANNEL_LOG,SWITCH_LOG_INFO, "mod_funasr_load start\n");
     
    // register API
    SWITCH_ADD_API( api_interface,
                    "uuid_start_funasr",
                    "uuid_start_funasr api",
                    uuid_start_funasr_function,
                    "<cmd><args>");
    
    SWITCH_ADD_API( api_interface,
                    "uuid_stop_funasr",
                    "uuid_stop_funasr api",
                    uuid_stop_funasr_function,
                    "<cmd><args>");
    

        //注册终端命令自动补全
//        switch_console_set_complete("add tasktest1 [args]");
//        switch_console_set_complete("add tasktest2 [args]");

    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " funasr_load\n");
    
    return SWITCH_STATUS_SUCCESS;
}

/**
 *  定义shutdown函数，关闭时运行
 */
SWITCH_MODULE_SHUTDOWN_FUNCTION(mod_funasr_shutdown)
{
    switch_log_printf(SWITCH_CHANNEL_LOG, SWITCH_LOG_DEBUG, " mod_funasr_shutdown called\n");
    return SWITCH_STATUS_SUCCESS;
}
