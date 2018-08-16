from __future__ import print_function, unicode_literals

import serial
import struct
import numpy as np

class UWBHelperBase(object):
    def __init__(self, serial_name, baud_rate):
        self.serial = serial.Serial(serial_name, baud_rate)
        print("Opened port ", self.serial)
        self.buf = b''
        self.self_id = None
        self.sys_time = -1

    def delete_first_n_buf(self, _len):
        self.buf = self.buf[_len:]
        return self.buf

    def parse_data(self, recv_buffer):
        #Nor imply yet
        pass

    def on_data_updated(self):
        pass

    def update_from_buf(self, read_all=False):
        try:
            while self.serial.in_waiting > 0:
                t = self.serial.read()
                self.buf = self.buf + t
                ret = self.parse_data(self.buf)
                if ret:
                    self.on_data_updated()

        except Exception as inst:
            # print(inst)
            print("Fuck", inst)
            raise

    def close(self):
        self.serial.close()


class UWBHelperNode(UWBHelperBase):
    WAIT_FOR_HEADER = 0
    WAIT_FOR_NODE_DETAIL = 1
    WAIT_FOR_NODE_CHECKSUM = 2
    REMOTE_NODE_HEADER_LEN = 16
    NODE_HEADER_LEN = 16

    def __init__(self, serial_name, baud_rate=921600, debug_output = False, update_cb=None, data_cb=None):
        super(UWBHelperNode, self).__init__(serial_name,baud_rate)

        self.read_status = UWBHelperNode.WAIT_FOR_HEADER
        self.read_wait_remote_node_num = 0


        self.nodes_info = {}

        self.debug_output = debug_output

        self.active_node_set = set()

        self.data_updated_cb = update_cb

        self.data_cb = data_cb
        self.available_node_num = 0
        self.count = 0



    def parse_node_header(self, recv_buf):
        struct_fmt = "<2xBI8xB"
        _id, _lps_time, _num = struct.unpack(struct_fmt, recv_buf[:UWBHelperNode.NODE_HEADER_LEN])
        #TODO:check valid
        self.sys_time = _lps_time
        self.self_id = _id
        self.node_num_now = _num
        recv_buf = self.delete_first_n_buf(UWBHelperNode.NODE_HEADER_LEN)

        for k in self.nodes_info:
            self.nodes_info[k]["active"] = False
        return self.node_num_now

    def is_node_msg_header(self, buf):
        return (buf[0] == 0x55 and buf[1] == 0xA0) or (ord(buf[0]) == 0x55 and ord(buf[1]) == 0xA0)

    def parse_header(self, buf):
        # print(''.join('{:02x}'.format(ord(x)) for x in buf))
        while len(buf) > 2 and not self.is_node_msg_header(buf):
            buf = self.delete_first_n_buf(1)
        if self.is_node_msg_header(buf):
            print("Catch header", len(buf))
        
        #Now msg is start with 0x55 and 0xA0
        if len(buf) < 16:
            # If less than 16, then can't get the node length
            return
        print("Catch header")
        if len(buf) > 10000:
            self.delete_first_n_buf(len(buf))
        node_num = self.parse_node_header(buf)
        self.read_wait_remote_node_num = node_num
        self.read_status = UWBHelperNode.WAIT_FOR_NODE_DETAIL

        print("Found header self id {}, wait for node_num {}".format(self.self_id, node_num))

    def parse_remote_node_details(self, buf):
        HEADER_LEN = UWBHelperNode.REMOTE_NODE_HEADER_LEN
        if len(buf) < HEADER_LEN:
            return
        struct_fmt = "<B3sHL4xH"
        _id, _dis, _rssi, _dis_time, _data_len = struct.unpack(struct_fmt, buf[0:HEADER_LEN])

        #Python3
        # _dis = int.from_bytes(_dis, byteorder="little")
        
        #Python2
        struct_fm2 = "<L"
        _dis =  _dis + chr(0)
        _dis = float(struct.unpack(struct_fm2, _dis)[0])


        if _data_len == 0:
            #No data msg recv
            buf = self.delete_first_n_buf(HEADER_LEN)
            self.on_node_data_recv(_id, _dis,_dis_time, _rssi)
            return True
        else:
            if len(buf) < HEADER_LEN + _data_len:
                return False
            else:
                _msg = buf[HEADER_LEN:HEADER_LEN + _data_len]
                buf = self.delete_first_n_buf(HEADER_LEN + _data_len)
                self.on_node_data_recv(_id, _dis, _dis_time, _rssi, _msg)
                return True

    def parse_data(self, recv_buffer):
        if self.read_status == UWBHelperNode.WAIT_FOR_HEADER:
            self.parse_header(recv_buffer)

        if self.read_status == UWBHelperNode.WAIT_FOR_NODE_DETAIL:
            if self.read_wait_remote_node_num > 0:
                ret = self.parse_remote_node_details(recv_buffer)
                if ret:
                    self.read_wait_remote_node_num = self.read_wait_remote_node_num - 1
            else:
                self.read_status = UWBHelperNode.WAIT_FOR_NODE_CHECKSUM

        if self.read_status == UWBHelperNode.WAIT_FOR_NODE_CHECKSUM:
            if len(recv_buffer) > 0:
                check_sum = recv_buffer[0]
                self.delete_first_n_buf(1)
                self.read_status = UWBHelperNode.WAIT_FOR_HEADER
                return True
        return False

    def on_data_recv(self, _id, _msg, _recv_time):
        # msg = str(_msg, encoding="utf-8")
        msg = str(_msg)

        # print(f"\nRECV MSG @{_id} on {_lps_time} sys_time {self.sys_time}\n {msg}")
        if self.debug_output:
            print("\nRECV DATA @{} on {} sys_time {}\n{}".format(
                _id, _recv_time, self.sys_time, msg
            ))

        if self.data_cb is not None:
            self.data_cb(_id, _msg, self.sys_time, _recv_time)

    def on_node_data_recv(self, _id, _dis, _dis_time, _rssi, msg=""):
        self.active_node_set.add(_id)
        self.nodes_info[_id] = {
            "distance": _dis/1000,
            "rssi": _rssi,
            "distance_time": _dis_time,
            "data":msg,
            "active":True
        }
        if msg != "":
            self.on_data_recv(_id, msg, _dis_time)

    def on_data_updated(self):
        self.count = self.count + 1
        for k in self.nodes_info:
            if k in self.active_node_set:
                self.nodes_info[k]["active"] = True
            else:
                self.nodes_info[k]["active"] = False

        self.available_node_num = len(self.active_node_set)
        self.active_node_set = set()

        if self.debug_output:
            print("\rSYSTIME {} NODES {}".format(self.sys_time, len(self.nodes_info)),end="")
            if self.count % 10 == 0:
                self.broadcast_data(b"Test!!")
            
        if self.data_updated_cb is not None:
            self.data_updated_cb(self)

    def broadcast_data(self, msg):
        self.serial.write(msg)


class UWBHelperTag(UWBHelperBase):
    def __init__(self, serial_name, baud_rate=921600):
        super(UWBHelperTag, self).__init__(serial_name,baud_rate)
        self.gyro = np.zeros(3)
        self.acc = np.zeros(3)
        self.angle = np.zeros(3)
        self.quat = np.array([1, 0, 0, 0])
        self.status = -1
        self.distance_measure = - np.eye(1, 10)


    def parse_data(self, recv_buffer):
        if len(recv_buffer) < 128:
            return False
        buf = recv_buffer[-128:]

        if not(buf[0] == 0x55 and buf[1] == 0x01):
            return False

        # Little-Endian Mode: Low bit in the front,high bit in the back.

        struct_fmt = "<2xBx42s6f12x3h4f8xLB10sB"

        tag_id, tag_vel_dis_buf, gyrox, gyroy, gyroz, accx, accy, accz,\
                angx, angy, angz, q0, q1, q2, q3, sys_time, status, user_data, check = \
                struct.unpack(struct_fmt, buf)



        tag_vel_dis = []
        for i in range(14):
            num_buf = tag_vel_dis_buf[i*3:i*3+3]
            # print(num_buf)
            tag_vel_dis.append(int.from_bytes(num_buf, byteorder='little'))

        tag_pos = np.array(tag_vel_dis[0:3])/1000
        tag_vel = np.array(tag_vel_dis[3:6])/10000

        diss = np.clip(tag_vel_dis[6:14], None, 10000*1000)/1000

        self.tag_id = tag_id
        self.gyro = np.array([gyrox, gyroy, gyroz])
        self.acc = np.array([accx, accy, accz])
        self.angle = np.array([angx, angy, angz])
        self.quat = np.array([q0, q1, q2, q3])
        self.sys_time = sys_time
        self.status = status
        self.distance_measure = diss

        # if int.from_bytes(user_data, 'little') is not 0:
        #     self.on_broadcast_data_recv(user_data)

        return True

    pass



if __name__ == "__main__":
    import sys
    port = "/dev/ttyS7"
    if len(sys.argv) > 1:
        port = sys.argv[1]
    print('Trying to open port at', port)
    uwb = UWBHelperNode(port, debug_output=True)
    try:
        while True:
            uwb.update_from_buf()

    except KeyboardInterrupt:
        uwb.close()
        print("Exit by keyboard")
        exit(0)
    except:
        uwb.close()
        raise