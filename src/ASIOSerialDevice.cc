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

#include <asio_serial_device/ASIOSerialDevice.h>

using namespace std;

ASIOSerialDevice::ASIOSerialDevice()
{
  active = false;
  open = false;
  serial_port = 0;
}

ASIOSerialDevice::ASIOSerialDevice(const string &device,
                                   unsigned int baud)
{
  active = false;
  open = false;
  serial_port = 0;

  Open(device, baud);
}

ASIOSerialDevice::~ASIOSerialDevice()
{
  if (active)
    Stop();

  if (serial_port != 0)
    delete serial_port;
}

void ASIOSerialDevice::Open(const string &device_,
                            unsigned int baud_,
                            ba::serial_port_base::parity parity,
                            ba::serial_port_base::character_size csize,
                            ba::serial_port_base::flow_control flow,
                            ba::serial_port_base::stop_bits stop)
{
  device = device_;
  baud = baud_;

  if (!open)
    {
      try
        {
          serial_port = new ba::serial_port(io_service, device);
        }
      catch (std::exception e)
        {
          cerr << "Unable to open device: " << device << endl;
          throw;
        }

      if (!serial_port->is_open())
        throw runtime_error("Failed to open serial port");

      ba::serial_port_base::baud_rate baud(baud_);
      serial_port->set_option(baud);

      serial_port->set_option(parity);
      serial_port->set_option(csize);
      serial_port->set_option(flow);
      serial_port->set_option(stop);

      open = true;
    }
}

void ASIOSerialDevice::Close()
{
  if (open)
    io_service.post(boost::bind(&ASIOSerialDevice::CloseCallback, this,
                                boost::system::error_code()));
}

void ASIOSerialDevice::Start()
{
  if (!open)
    throw runtime_error("Serial port interface not open");

  ReadStart();

  thread = boost::thread(boost::bind(&ba::io_service::run, &io_service));

  active = true;

  return;
}

void ASIOSerialDevice::ReadStart()
{
  if (open)
    serial_port->async_read_some(ba::buffer(read_msg, MAX_READ_LENGTH),
                                 boost::bind(&ASIOSerialDevice::ReadComplete,
                                             this, ba::placeholders::error,
                                             ba::placeholders::bytes_transferred));
}

void ASIOSerialDevice::ReadComplete(const boost::system::error_code& error,
                                       size_t bytes_transferred)
{
  if (!error)
    {
      if (!read_callback.empty())
        read_callback(const_cast<unsigned char *>(read_msg), bytes_transferred);
      ReadStart();
    }
  else
    CloseCallback(error);
}

void ASIOSerialDevice::SetReadCallback(const boost::function<void (const unsigned char*, size_t)>& handler)
{
  read_callback = handler;
}

void ASIOSerialDevice::Stop()
{
  Close();
  thread.join();
  active = false;
}

void ASIOSerialDevice::CloseCallback(const boost::system::error_code& error)
{
  if (error && (error != ba::error::operation_aborted))
    cerr << "Error: " << error.message() << endl;

  serial_port->close();
  open = false;
}

bool ASIOSerialDevice::Write(const vector<unsigned char>& msg)
{
  if (!open)
    return false;

  io_service.post(boost::bind(&ASIOSerialDevice::WriteCallback, this, msg));

  return true;
}

void ASIOSerialDevice::WriteCallback(const vector<unsigned char>& msg)
{
  bool write_in_progress = !write_msgs.empty();
  write_msgs.push_back(msg);
  if (!write_in_progress)
    WriteStart();
}

void ASIOSerialDevice::WriteStart()
{
  ba::async_write(*serial_port,
                           ba::buffer(&(write_msgs.front()[0]),
                                               write_msgs.front().size()),
                           boost::bind(&ASIOSerialDevice::WriteComplete,
                                       this, ba::placeholders::error));
}

void ASIOSerialDevice::WriteComplete(const boost::system::error_code& error)
{
  if (!error)
    {
      write_msgs.pop_front();
      if (!write_msgs.empty())
        WriteStart();
    }
  else
    CloseCallback(error);
}

bool ASIOSerialDevice::Active()
{
  return active;
}
