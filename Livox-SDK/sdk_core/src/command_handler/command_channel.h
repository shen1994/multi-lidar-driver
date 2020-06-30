//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_COMMAND_CHANNEL_H_
#define LIVOX_COMMAND_CHANNEL_H_
#include <memory>
#include <map>
#include <list>
#include <string>
#include "apr_network_io.h"
#include "base/io_loop.h"
#include "comm/comm_port.h"

namespace livox {

typedef struct TagCommand {
  uint8_t handle;
  CommPacket packet;
  std::shared_ptr<CommandCallback> cb;
  uint32_t time_out;
  TagCommand() : packet(), time_out(0) {}
  TagCommand(uint8_t _handle,
             uint8_t _cmd_type,
             uint8_t _cmd_set,
             uint8_t _cmd_code,
             uint16_t _seq_num,
             uint8_t *data,
             uint16_t length,
             uint32_t _time_out,
             const std::shared_ptr<CommandCallback> &_cb)
      : handle(_handle), packet(), cb(_cb) {
    packet.packet_type = _cmd_type;
    packet.cmd_set = _cmd_set;
    packet.cmd_code = _cmd_code;
    packet.seq_num = _seq_num;
    packet.data = data;
    packet.data_len = length;
    time_out = _time_out;
  }
} Command;

class CommandChannelDelegate {
 public:
  virtual void OnCommand(uint8_t handle, const Command &command) = 0;
  virtual void OnHeartbeatStateUpdate(uint8_t handle, const HeartbeatResponse &state) = 0;
};

/**
 * CommandChannel implements the sending/receiving commands with a specific device.
 */
class CommandChannel : public IOLoop::IOLoopDelegate {
 public:
  CommandChannel(apr_port_t port,
                 uint8_t handle,
                 const std::string &remote_ip,
                 CommandChannelDelegate *cb,
                 apr_pool_t *pool);
  virtual ~CommandChannel() { Uninit(); }

  /** Uninitialize CommandChannel. */
  void Uninit();

  /**
   * Bind a CommandChannel with a IOLoop.
   * @param loop the IOLoop to bind.
   * @return true on successfully.
   */
  bool Bind(IOLoop *loop);

  /**
   * Send a command asynchronously.
   * @param command the command to send.
   */
  void SendAsync(const Command &command);

  void OnData(apr_socket_t *, void *);
  void OnTimer(apr_time_t now);

  static uint16_t GenerateSeq();

 private:
  void Send(const Command &cmd);
  void HeartBeat(apr_time_t t);
  void SendInternal(const Command &command);
  Command DeepCopy(const Command &cmd);
  void OnHeartbeatAck(const CommPacket &packet);
  void DeviceDisconnect(uint8_t handle);

 private:
  static const int kHeartbeatTimer = 800;
  uint8_t handle_;
  apr_port_t port_;
  apr_socket_t *sock_;
  apr_pool_t *mem_pool_;
  IOLoop *loop_;
  CommandChannelDelegate *callback_;
  std::map<uint16_t, std::pair<Command, apr_time_t> > commands_;
  std::unique_ptr<CommPort> comm_port_;
  apr_time_t heartbeat_time_;
  std::string remote_ip_;
  apr_time_t last_heartbeat_;
};

}  // namespace livox

#endif  // LIVOX_COMMAND_CHANNEL_H_
