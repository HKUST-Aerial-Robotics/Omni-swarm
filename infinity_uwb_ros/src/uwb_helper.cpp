#include "uwb_helper.h"

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>


UWBHelperNode::UWBHelperNode(std::string serial_name, int baudrate, bool enable_debug_output) {
    this->enable_debug_output = enable_debug_output;
    if (open_port(serial_name, baudrate))
    {
        printf("Open port %s[%d] successful!\n\n", serial_name.c_str(), baudrate);
    }
    else {
        printf("Can't open serial port;exit\n");
        exit(-1);
    }
}

bool UWBHelperNode::open_port(std::string serial_name, int baudrate)
{
    printf("Trying to open port %s\n", serial_name.c_str());
    serial_fd = open(serial_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1)
    {
        perror("open_port: Unable to open port ");
        return false;
    }
    else
        fcntl(serial_fd, F_SETFL, 0);

    configure_port(baudrate);

    return true;
}

void UWBHelperNode::configure_port(int baudrate)
{
    struct termios tty;

    if (tcgetattr(serial_fd, &tty) < 0) {
        printf("Error from tcgetattr");
        exit(-1);
        return ;
    }

#if defined(__MACH__)
    speed_t spd = baudrate;
#else
    speed_t spd = B921600;
    switch (baudrate)
    {
        case 3000000:
            spd = B3000000;
            break;
        case 1000000:
            spd = B1000000;
            break;
        case 230400:
            spd = B230400;
            break;
        case 460800:
            spd = B460800;
            break;
        case 921600:
            spd = B921600;                
            break;
        case 115200:
        default:
            spd = B115200;
            break;
    }
#endif
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);
    
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr");
        exit(-1);
        return;
    }
    printf("Successful set port\n");
    return;
}


void UWBHelperNode::delete_first_n_buf(int _len)
{
    buf.erase(buf.begin(), buf.begin()+_len);
}


void UWBHelperNode::read_and_parse() {
    while (serial_available_bytes() > 0)
    {
        uint8_t c = read_byte_from_serial();
        // printf("%c",c);
        buf.push_back(c);
        bool ret = this->parse_data();
        if (ret) {
            this->on_node_data_updated();
        }
    }
}


bool UWBHelperNode::parse_data()
{
    if(this->read_status == WAIT_FOR_HEADER)
    {
        this->parse_header();
    }

    if (this->read_status == WAIT_FOR_NODE_DETAIL)
    {
        if (this->read_wait_remote_node_num > 0)
        {
            bool ret = parse_remote_node_details();
            if (ret)
            {
                this->read_wait_remote_node_num --;
            }
        }
        else {
            this->read_status = WAIT_FOR_NODE_CHECKSUM;
        }
    }

    if (this->read_status == WAIT_FOR_NODE_CHECKSUM)
    {
        if (buf.size() > 0)
        {
            uint8_t checksum = buf[0];
            this->delete_first_n_buf(1);
            this->read_status = WAIT_FOR_HEADER;
            return true;
        }
    }
    return false;
}

void UWBHelperNode::send_broadcast_data(std::vector<uint8_t> msg)
{
    this->serial_write((uint8_t*)msg.data(), msg.size());   
}

void UWBHelperNode::on_node_data_updated()
{
    for (auto it : nodes_info)
    {
        int _id = it.first;
        if (active_node_set.find(_id) != active_node_set.end())
            nodes_info[_id].active = true;
        else
            nodes_info[_id].active = false;
    }
       
    if(this->enable_debug_output)
    {
        printf("\rSYSTIME %d NODES %ld",this->sys_time, this->nodes_info.size());
        for (auto it : nodes_info)
        {
            std::string str = it.second.to_str();
            printf("%s;;",str.c_str());
        }
    }

    active_node_num = active_node_set.size();
    this->active_node_set.clear();

}

int UWBHelperNode::serial_available_bytes()
{
    int bytes = 0;
    ioctl(serial_fd, FIONREAD, &bytes);
    return bytes;
}

uint8_t UWBHelperNode::read_byte_from_serial()
{
    char c;
    int n = read (serial_fd, &c, sizeof(char));
    // printf("%2x ", c);
    return c;
}

#pragma pack(push, 1)
struct NodeHeader{
    char head[2];
    uint8_t id;
    uint32_t lps_time;
    char reserved[8];
    uint8_t remote_num;
};
#pragma pack(pop)

int UWBHelperNode::parse_node_header()
{
    NodeHeader nh;
    memcpy(&nh, buf.data(), sizeof(nh));
    // printf("id %d t %d n %d\n",
        // nh.id,
        // nh.lps_time,
        // nh.remote_num
    // );
    this->sys_time = nh.lps_time;
    this->self_id = nh.id;
    this->active_node_num = nh.remote_num;
    this->delete_first_n_buf(NODE_HEADER_LEN);
    return this->active_node_num;
}

bool UWBHelperNode::is_node_msg_header()
{
    return (buf[0] == 0x55 && buf[1] == 0xA0);
}


void UWBHelperNode::parse_header()
{
    while(buf.size() > 2 && ! is_node_msg_header())
    {
        delete_first_n_buf(1);
    }

    if (buf.size()<16)
    {
        return;
    }

    // printf("Found node header\n");
    int node_num = this->parse_node_header();
    if (node_num > 10)
    {
        node_num = 0;
    }
    this->read_wait_remote_node_num = node_num;
    this->read_status = WAIT_FOR_NODE_DETAIL;
    // printf("node_num %d\n", node_num);
}

#pragma pack(push, 1)
struct RemoteNodeHeader{
    uint8_t id;
    uint32_t distance : 24;
    uint16_t rssi;
    uint32_t dis_sys_time;
    char reserved[4];
    uint16_t data_len;
};
#pragma pack(pop)
bool UWBHelperNode::parse_remote_node_details()
{
    if (buf.size() < REMOTE_NODE_HEADER_LEN)
        return false;
    RemoteNodeHeader nh;
    memcpy(&nh, buf.data(), sizeof(nh));

    if (nh.data_len == 0)
    {
        //No data msg recved
        delete_first_n_buf(REMOTE_NODE_HEADER_LEN);
        this->on_node_data_recv(nh.id, nh.distance, nh.dis_sys_time, nh.rssi);
        return true;
    }
    else {
        if (buf.size() < REMOTE_NODE_HEADER_LEN + nh.data_len)
        {
            return false;
        }
        else {
            Buffer msg(nh.data_len);
            std::copy(buf.begin() + REMOTE_NODE_HEADER_LEN,
                    buf.begin() + REMOTE_NODE_HEADER_LEN + nh.data_len
            ,msg.begin());

            delete_first_n_buf(REMOTE_NODE_HEADER_LEN + nh.data_len);
            this->on_node_data_recv(nh.id, nh.distance, nh.dis_sys_time, nh.rssi, msg);
            return true;
        }
    }
    return false;
}

void UWBHelperNode::on_broadcast_data_recv(int _id, int _recv_time, std::vector<uint8_t> _msg)
{

    char * str = (char *) _msg.data();
    if (this->enable_debug_output)
    {
        printf("RECV DATA @ %d on %d, systime %d, msg\n%s\n",
            _id, _recv_time,  this->sys_time, str
        );
    }

}

void UWBHelperNode::serial_write(uint8_t* data, int len)
{
    write(serial_fd, data, len);
}

void UWBHelperNode::on_node_data_recv(int _id, int _dis, int _dis_time, int _rssi, std::vector<uint8_t> msg)
{
    this->active_node_set.insert(_id);
    // printf("Insert to active node set%d\n", _id);
    if (nodes_info.find(_id) == nodes_info.end())
    { 
        RemoteNodeInfo info;
        nodes_info[_id] = info;
        nodes_info[_id].id = _id;
    }

    nodes_info[_id].distance = ((double)_dis)/1000;
    nodes_info[_id].rssi = (double)_rssi;
    nodes_info[_id].msg = msg;
    nodes_info[_id].active = true;
    nodes_info[_id].dis_time = _dis_time;

    if (msg.size() != 0)
    {
        on_broadcast_data_recv(_id, _dis_time, msg);
    }
}

