/**
 * @file  : TCP.cpp
 *
 * @date  : 03.10.2012
 * @author: m1ch1
 */

#include "TCP.h"

namespace apps
{
using boost::asio::ip::tcp;
using namespace std;

TCP::TCP(unsigned int port)		//server
{
	_choosenMode = server;
	_port = port;

	_delay = 2000000;

	_requestNewConnection = false;

	_server = new tcp::acceptor(_ioService, tcp::endpoint(tcp::v4(), _port));
	_socket = NULL;

	_timer = new boost::asio::deadline_timer(_ioService);

	_targetServer = NULL;
}

TCP::TCP(std::string ip, unsigned int port)		//client
{
	_choosenMode = client;
	_port = port;
	_ip = ip;

	_delay = 2000000;

	_requestNewConnection = false;

	_targetServer = new boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(_ip),_port);
	_socket = NULL;

	_timer = new boost::asio::deadline_timer(_ioService);

	_server = NULL;
}

TCP::~TCP()
{
    if(!_server)        delete _server;
    if(!_timer)         delete _timer;
    if(!_socket)        delete _socket;
    if(!_targetServer)  delete _targetServer;
}

int TCP::read(void* data, unsigned int size)
{
    int ret = -1;
	try{
	    ret = _socket->read_some(boost::asio::buffer(data,size));
	}
	catch(boost::system::system_error& e)
    {
	    //get new connection
	    this->connectOnce();
	    ret = -1;
    }
	return ret;
}

int TCP::readAll(void* data, unsigned int size)
{
    int tmp_read = 0;
    int ret = 0;
    unsigned char* tmp_data = (unsigned char*)data;

    while(tmp_read < size)
    {
        void* tmp_data_void = (void*)&tmp_data[tmp_read];
        try{
            ret = _socket->read_some(boost::asio::buffer(tmp_data_void, size - tmp_read));
            tmp_read += ret;
        }
        catch(boost::system::system_error& e)
        {
            this->connectOnce();
            return -1;
        }
    }
    return 0;
}


int TCP::read(void* data, unsigned int size, unsigned int delay_us)
{
    bool data_available = false;
    _socket->async_read_some(boost::asio::buffer(data,size),
                             boost::bind(&TCP::read_callback,
                                         this,
                                         boost::ref(data_available),
                                         boost::ref(*_timer),
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred)
                            );

    boost::posix_time::microsec delay_boost(delay_us);

    _timer->expires_from_now(delay_boost);

    _timer->async_wait(boost::bind(&TCP::wait_callback,
                                 this,
                                 boost::ref(*_socket),
                                 boost::asio::placeholders::error));

    _ioService.run();  // will block until async callbacks are finished

    _ioService.reset();

    if(_requestNewConnection)
    {
        _requestNewConnection = false;
        this->connectOnce();
        return -1;
    }

    if (!data_available)
    {
        //no data;
        //std::cout << "debug: receive timed out or failed" << std::endl;
        return -2;
    }
    return 0;
}


int TCP::connectOnce()
{
    if(_choosenMode == server)
    {
        delete _socket;
        _socket = new tcp::socket(_ioService);
        _server->accept(*_socket);  //wait for connection....
    }
    else if(_choosenMode == client)
    {

        bool ok = true;
        do{
            try{
                delete _socket;
                _socket = new boost::asio::ip::tcp::socket(_ioService);
                _socket->connect(*_targetServer);
                ok = true;
            }
            catch(boost::system::system_error& e)
            {
                ok = false;
                usleep(_delay);
            }
        }while(!ok);
    }
    return 0;
}

int TCP::write(void* data, unsigned int size)
{
    int size_ = 0;
    try{
	size_ = _socket->write_some(boost::asio::buffer(data,size));
    }
    catch(boost::system::system_error& e)
    {
        //get new connection
        std::cerr << "Got Boost sytem Error while transmitting" << std::endl;
        this->connectOnce();
        return -1;
    }
	return size_;
}

void TCP::read_callback(bool& data_available,
        boost::asio::deadline_timer& timeout,
        const boost::system::error_code& error, std::size_t bytes_transferred)
{

    //std::cout << "debug: called -> read_callback(...):->" << error.value() << "<-" << std::endl;
    if(error.value() == ERROR_CODE_LOST_SOCKET)
    {
        _requestNewConnection = true;
    }
    if (error || !bytes_transferred)
    {
        // No data was read!
        data_available = false;
        return;
    }

    timeout.cancel();  // will cause wait_callback to fire with an error
    data_available = true;
}

void TCP::wait_callback(boost::asio::ip::tcp::socket& ser_port,
        const boost::system::error_code& error)
{
    //std::cout << "debug: called -> SerialCom::wait_callback(...)" << std::endl;
    if(error)
    {
        // Data was read and this timeout was canceled
        return;
    }

    ser_port.cancel();  // will cause read_callback to fire with an error
}

} /* namespace apps */


