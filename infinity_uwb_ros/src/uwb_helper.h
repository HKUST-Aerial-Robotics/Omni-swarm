#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <set>
#include <stdio.h>
#include <map>


#define REMOTE_NODE_HEADER_LEN 16
#define NODE_HEADER_LEN 16
typedef std::vector<uint8_t> Buffer;
struct RemoteNodeInfo {
    double distance = -1;
    int dis_time;
    double rssi = 0;
    Buffer msg;
    bool active = false;
    int id = 0;
    std::string to_str()
    {
        char str[100] = {0};
        sprintf (str, " id:%d dis:%f rssi:%f active:%d msg:%s",
            id, distance, rssi, active, (char*)msg.data()
        );
        std::string _str(str);
        return _str;
    }
};


class UWBHelperNode
{
public:

    enum {
        WAIT_FOR_HEADER,
        WAIT_FOR_NODE_DETAIL,
        WAIT_FOR_NODE_CHECKSUM,
    };

    UWBHelperNode(std::string serial_name, int baudrate, bool enable_debug_output=false);
    std::vector<uint8_t> buf;
    int self_id;
    int sys_time;
    int active_node_num = 0;

    void read_and_parse();

    void send_broadcast_data(std::vector<uint8_t> msg);

protected:
    void delete_first_n_buf(int _len);
    bool parse_data(); 

    bool enable_debug_output;

    virtual void on_broadcast_data_recv(int _id, int _recv_time, Buffer _msg);
    void on_node_data_recv(int _id, int _dis, int _dis_time, int _rssi, Buffer msg = Buffer(0));

    virtual void on_node_data_updated();
    std::map<int,RemoteNodeInfo> nodes_info;
private:
    int serial_fd;
    void configure_port(int baudrate);
    bool open_port(std::string serial_name, int baudrate);

    uint8_t read_byte_from_serial();
    int serial_available_bytes();

    int read_status = WAIT_FOR_HEADER;
    int read_wait_remote_node_num  = 0;

    std::set<int> active_node_set;


    int parse_node_header();

    bool is_node_msg_header();

    void parse_header();

    bool parse_remote_node_details();


    void serial_write(uint8_t* data, int len);

};