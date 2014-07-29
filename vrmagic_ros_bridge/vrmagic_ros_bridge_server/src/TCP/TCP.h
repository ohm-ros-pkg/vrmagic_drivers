/**
 * @file  : TCP.h
 *
 * @date  : 03.10.2012
 * @author: m1ch1
 *
 * @todo add Enums for returnvalues.... maybe with exeptions
 * @todo add std::cerr output on broken connection
 *
 */

#ifndef TCP_H_
#define TCP_H_

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>

#define ERROR_CODE_LOST_SOCKET  2


namespace apps
{

using boost::asio::ip::tcp;

class TCP
{
private:	//data elements

	enum _mode {server,client};
	_mode _choosenMode;

	unsigned int _delay;

	bool _requestNewConnection;

	boost::asio::io_service _ioService;
	unsigned int _port;
	std::string _ip;

	boost::asio::ip::tcp::endpoint* _targetServer;	//used when client

	boost::asio::ip::tcp::socket* _socket;

	tcp::acceptor* _server;							//used when server

    // -- async_read + timeout --
    boost::asio::deadline_timer* _timer;

public:
	TCP(unsigned int port);						//server
	TCP(std::string ip, unsigned int port);		//client
	virtual ~TCP();

	int connectOnce();

	/**
     * @brief reads data with given size from socket
     *
     * @note It is possible that this function does not read all given data
     *
     * @param[in,out]  arg	->	description todo
     *
     * @return 		   number of read bytes
     */
	int read(void* data, unsigned int size);

	/**
     * @brief reads data with given size from socket
     *
     * @note this function will block until all data is read. It is
     *       forced to read all given data
     *
     * @param[in,out]  arg	->	description
     *
     * @return 		   0 on succes and -1 on error(brocken connection)
     */
	int readAll(void* data, unsigned int size);

	/**
     * @brief return after given delay with -1 else 0 and blocks
     *        endless (wait for new connection) whenn connection
     *        is broken
     *
     * @param[in,out]  arg	->	description
     *
     * @return 		   0 on succes and -1 on timeout and broken connection
     */
	int read(void* data, unsigned int size, unsigned int delay_us);

	int write(void* data, unsigned int size);

private:    //functions
    /**
     * @fn read_callback( bool& data_available,
     *                    boost::asio::deadline_timer& timeout,
     *                    const boost::system::error_code& error,
     *                    std::size_t bytes_transferred );
     *
     * @brief callback function for reading asynchron serial data with timeout
     *
     *
     * @param[in,out]
     *
     *
     * @return void
     */
    void read_callback(bool& data_available, boost::asio::deadline_timer& timeout,
                       const boost::system::error_code& error,
                       std::size_t bytes_transferred);

    /**
     * @fn void wait_callback( boost::asio::serial_port& ser_port,
     *                         const boost::system::error_code& error );
     *
     * @brief callback function for reading asynchron serial data with timeout
     *
     *
     * @param[in,out]
     *
     *
     * @return void
     */
    void wait_callback(boost::asio::ip::tcp::socket& ser_port,
                       const boost::system::error_code& error);

};

} /* namespace apps */
#endif /* TCP_H_ */
