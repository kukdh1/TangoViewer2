#include "NetworkManager.h"

namespace kukdh1
{
	class Session
	{
		private:
			boost::asio::io_service &_io_service;
			boost::asio::ip::tcp::socket _socket;

			HWND hWndParent;
			LogWindow *lWindow;

			NMDataCallback nmcCallback;

			char cHeader[8];
			char *pcData;

			void handle_read(const boost::system::error_code &error, const size_t byte_transferred)
			{
				if (!error)
				{
#ifdef PRINT_DEBUG_MSG
					if (lWindow)
						lWindow->PrintLog(L"[Net-Debug] Data In\r\n");
#endif

					if (byte_transferred != 8)
						return;

					unsigned int uiLength;
					unsigned int uiByteLength;

					if (strncmp(cHeader, "PCD2", 4) == 0)
					{
						//Recv Ready
						memcpy(&uiLength, cHeader + 4, 4);
						uiByteLength = 64 + uiLength * 16;	//Matrix(4by4, float) + Point(X,Y,Z,RGBA)

						pcData = (char *)calloc(uiByteLength + 8, sizeof(char));
						memcpy(pcData, cHeader, 8);
						boost::asio::async_read(_socket, boost::asio::buffer(pcData + 8, uiByteLength), boost::asio::transfer_exactly(uiByteLength), boost::bind(&Session::handle_read_2, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
					}
					else
						read_header();
				}
				else
				{
					if (lWindow)
						lWindow->PrintLog(L"[Net-Error] Receive Failed\r\n");

					delete this;
				}
			}

			void handle_read_2(const boost::system::error_code &error, const size_t byte_transferred)
			{
				if (!error)
				{
#ifdef PRINT_DEBUG_MSG
					if (lWindow)
						lWindow->PrintLog(L"[Net-Debug] Data In\r\n");

					if (lWindow)
						lWindow->PrintLog(L"[Net-Debug] Add Thread Called\r\n");
#endif

					nmcCallback(hWndParent, (unsigned int)byte_transferred + 8, (unsigned char *)pcData);
					
					free(pcData);
				}
				else
				{
					if (lWindow)
						lWindow->PrintLog(L"[Net-Error] Receive Failed\r\n");

					delete this;
				}

				read_header();
			}

			void read_header()
			{
				boost::asio::async_read(_socket, boost::asio::buffer(cHeader, 8), boost::asio::transfer_exactly(8), boost::bind(&Session::handle_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
			}

		public:
			Session(boost::asio::io_service &io_service, LogWindow *plWindow, HWND hWndParent, NMDataCallback function)
				: _io_service(io_service), _socket(io_service)
			{
				lWindow = plWindow;
				nmcCallback = function;
				this->hWndParent = hWndParent;
			}

			void Start()
			{
				read_header();
			}

			boost::asio::ip::tcp::socket &GetSocket()
			{
				return _socket;
			}
	};

	class Server
	{
		private:
			boost::asio::io_service &_io_service;
			boost::asio::ip::tcp::acceptor _acceptor;

			HWND hWndParent;
			LogWindow *lWindow;
			NMDataCallback nmcCallback;
			
			Session *pSession;

			void handle_accept(const boost::system::error_code &error)
			{
				if (!error)
				{
					if (lWindow)
					{
						boost::asio::ip::tcp::endpoint remote_ep = pSession->GetSocket().remote_endpoint();
						boost::asio::ip::address remote_ad = remote_ep.address();
						unsigned short remote_port = remote_ep.port();

						lWindow->PrintLog(L"[Net-Info] Client Connected (%S:%d)\r\n", remote_ad.to_string().c_str(), remote_port);

						pSession->Start();
					}
				}
				else
				{
					if (lWindow)
						lWindow->PrintLog(L"[Net-Error] Accept Failed\r\n");

					delete pSession;

					start_accept();
				}
			}

		public:
			Server(boost::asio::io_service &io_service, LogWindow *plWindow, HWND hWndParent, NMDataCallback function)
				: _io_service(io_service), _acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), PORT))
			{
				lWindow = plWindow;
				nmcCallback = function;

				if (lWindow)
					lWindow->PrintLog(L"[Net-Info] Server Begin\r\n");

				this->hWndParent = hWndParent;
				start_accept();
			}

			void start_accept()
			{
				pSession = new Session(_io_service, lWindow, hWndParent, nmcCallback);
				_acceptor.async_accept(pSession->GetSocket(), boost::bind(&Server::handle_accept, this, boost::asio::placeholders::error));
			}
	};

	void NetworkManager(HWND hWnd, NMDataCallback function, LogWindow *lWindow)
	{
		static boost::asio::io_service io_context;
		static Server s(io_context, lWindow, hWnd, function);

		boost::thread t(boost::bind(&boost::asio::io_service::run, &io_context));
	}
}