//
// Created by kukdh1 on 2015-11-22.
//

#include <tango-point-cloud/async_socket_client.h>

namespace tango_point_cloud {
    Session::Session(asio::io_context &io_context)
            : _io_context(io_context), _socket(io_context)
    {
        char filename[1024];
        char ip[16];

        sprintf(filename, "/storage/emulated/0/ip.txt");
        memset(ip, 0, 16);

        FILE *in = fopen(filename, "r");

        if (in)
        {
            fread(ip, 1, 15, in);
            fclose(in);
        }
        else
        {
            strncpy(ip, SERVER_ADDR, strlen(SERVER_ADDR));

            FILE *out = fopen(filename, "w");
            fwrite(ip, 1, strlen(ip), out);
            fclose(out);
        }

        LOGI("[Net-Info] Session Created (%s:%s)", ip, PORT);
        bConnected = false;

        asio::ip::tcp::resolver resolver(_io_context);
        asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(ip, PORT);

        asio::async_connect(_socket, endpoints, boost::bind(&Session::handle_connect, this, asio::placeholders::error));
    }

    void Session::handle_connect(const asio::error_code &error)
    {
        if (!error)
        {
            LOGI("[Net-Info] Connect Succeed");
            bConnected = true;
        }
        else
        {
            LOGE("[Net-Error] Connect Failed");
            delete this;
        }
    }

    void Session::do_write(char *pcData, const unsigned int uiLength)
    {
        bool writing = !sdQueue.empty();
        Data dData;

        if (!bConnected) {
            LOGE("[Net-Error] Not Connedted");

            free(pcData);

            return;
        }

        dData.pcData = pcData;
        dData.uiLength = uiLength;

        sdQueue.push_back(dData);

        if (!writing)
            asio::async_write(_socket, asio::buffer(sdQueue.front().pcData, sdQueue.front().uiLength), boost::bind(&Session::handle_write, this, asio::placeholders::error));
        else
            LOGE("[Net-Info] Not Transfered %d", writing);
    }

    void Session::handle_write(const asio::error_code &error)
    {
        if (!error)
        {
            LOGI("[Net-Info] Transfer Succeed");

            free(sdQueue.front().pcData);
            sdQueue.pop_front();
            if (!sdQueue.empty())
            {
                asio::async_write(_socket, asio::buffer(sdQueue.front().pcData, sdQueue.front().uiLength), boost::bind(&Session::handle_write, this, asio::placeholders::error));
            }
        }
        else
        {
            LOGE("[Net-Error] Transfer Failed");
            delete this;
        }
    }

    NetworkClient::NetworkClient()
            : work(io_context)
    {
        LOGI("[Net-Info] Client Begin");
        bFirstData = true;

        pSession = new Session(io_context);

        asio::thread t(boost::bind(&asio::io_service::run, &io_context));
    //    io_context.poll();
    }

    void NetworkClient::write(const char *pcData, const unsigned int uiLength)
    {
        char *pcTemp;

        pcTemp = (char *)calloc(uiLength, sizeof(char));
        memcpy(pcTemp, pcData, uiLength * sizeof(char));

        asio::post(io_context, boost::bind(&Session::do_write, pSession, pcTemp, uiLength));
    }

    void NetworkClient::SendPointCloudToServer(const std::vector<float> &points, const std::vector<uint8_t> &colors, const glm::mat4 &transform)
    {
        LOGI("[Net-Debug] Make Data");

        if (bFirstData)
        {
            bFirstData = false;
            return;
        }

        unsigned int length;
        unsigned int bytelength;
        unsigned char *pBuffer;

        bytelength = 8 + 16 * 4 + colors.size() * 4;

        pBuffer = (unsigned char *)calloc(bytelength, sizeof(unsigned char));

        if (pBuffer)
        {
            length = colors.size() / 4;

            memcpy(pBuffer, "PCD2", 4);
            memcpy(pBuffer + 4, &length, 4);
            memcpy(pBuffer + 8, glm::value_ptr(transform), 16 * 4);

            unsigned char *pPointBuf;

            pPointBuf = pBuffer + 72;

            for (unsigned int i = 0; i < length; i++) {
                memcpy(pPointBuf +i * 16, &points.at(i * 3), 4);
                memcpy(pPointBuf +i * 16 + 4, &points.at(i * 3 + 1), 4);
                memcpy(pPointBuf +i * 16 + 8, &points.at(i * 3 + 2), 4);
                pPointBuf[i * 16 + 12] = colors.at(i * 4);
                pPointBuf[i * 16 + 13] = colors.at(i * 4 + 1);
                pPointBuf[i * 16 + 14] = colors.at(i * 4 + 2);
                pPointBuf[i * 16 + 15] = colors.at(i * 4 + 3);
            }

            write((char *)pBuffer, bytelength);

            free(pBuffer);
        }
    }
}