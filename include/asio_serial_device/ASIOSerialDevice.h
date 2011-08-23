/*
  This file is part of asio-serial-device, a class wrapper to
  use the boost::asio serial functionality.

  asio-serial-device is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Nathan Michael, Aug. 2011
*/

#ifndef __ASIOSERIALDEVICE__
#define __ASIOSERIALDEVICE__

#include <deque>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>

// Class version of example ASIO over serial with boost::asio:
// http://groups.google.com/group/boost-list/browse_thread/thread/5cc7dcc7b90d41fc

#define MAX_READ_LENGTH 512

class ASIOSerialDevice
{
 public:
  ASIOSerialDevice();
  ASIOSerialDevice(const std::string &device, unsigned int baud);
  ~ASIOSerialDevice();

  void SetReadCallback(const boost::function<void (const unsigned char*, size_t)>& handler);

  void Start();
  void Stop();

  bool Write(const std::vector<unsigned char>& msg);
  void Open(const std::string& device, unsigned int baud);

  bool Active();

 private:
  void Close();
  void CloseCallback(const boost::system::error_code& error);

  void ReadStart();
  void ReadComplete(const boost::system::error_code& error,
                    size_t bytes_transferred);

  void WriteCallback(const std::vector<unsigned char>& msg);
  void WriteStart();
  void WriteComplete(const boost::system::error_code& error);

  std::string device;
  unsigned int baud;
  bool active, open;
  std::deque< std::vector<unsigned char> > write_msgs;

  boost::thread thread;
  boost::asio::io_service io_service;
  boost::asio::serial_port* serial_port;
  boost::function<void (const unsigned char*, size_t)> read_callback;

  unsigned char read_msg[MAX_READ_LENGTH];
};
#endif
