//
// Created by kukdh1 on 2015-11-22.
//

#ifndef POINT_CLOUD_JNI_EXAMPLE_ASYNC_SOCKET_CLIENT_H
#define POINT_CLOUD_JNI_EXAMPLE_ASYNC_SOCKET_CLIENT_H

#include <jni.h>
#include <boost/bind.hpp>
#include <asio.hpp>
#include <deque>

#include <tango_client_api.h>  // NOLINT
#include <tango-gl/util.h>

#define PORT            "9000"
#define SERVER_ADDR     "1.233.237.176"

namespace tango_point_cloud {
    struct Data {
        char *pcData;
        unsigned int uiLength;
    };

    class Session {
        private:
            asio::io_context &_io_context;
            asio::ip::tcp::socket _socket;

            std::deque<Data> sdQueue;

            bool bConnected;

            void handle_connect(const asio::error_code &error);
            void handle_write(const asio::error_code &error);

        public:
            Session(asio::io_context &io_context);
            void do_write(char *pcData, const unsigned int uiLength);
    };

    class NetworkClient {
        private:
            asio::io_context io_context;
            asio::io_service::work work;
            Session *pSession;
            bool bFirstData;

            void write(const char *pcData, const unsigned int uiLength);

        public:
            NetworkClient();
            void SendPointCloudToServer(const std::vector<float> &points, const std::vector<uint8_t> &colors, const glm::mat4 &transform);
    };
}

#endif //POINT_CLOUD_JNI_EXAMPLE_ASYNC_SOCKET_CLIENT_H
