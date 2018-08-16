# infinity_uwb_ros

This node is for infinity UWB module, which can be used to measure distance and broadcast data in a multiple drone swarm system.

## Usage
By default, we direct plug-in the infinity UWB to the manifold by USB port. Which will use serial named /dev/ttyUSB0 and 921600 as its baudrate

```bash
roslaunch infinity_uwb_ros uwb_node_manifold2.launch
```

Now we can only send no more than 250 byte data one time and use the send rate of 50hz

## Publishes

```bash
/uwb_node/remote_nodes
/uwb_node/incoming_broadcast_data
```

The example of **/uwb_node/remote_nodes** is

```bash                                              
header:                                            
  seq: 99                                          
  stamp:                                           
    secs: 1531916471                               
    nsecs: 531233072                               
  frame_id: ''                                     
sys_time: 14608531                                 
remote_node_num: 2                                 
self_id: 0                                         
node_ids: [2, 3]                                   
node_dis: [4.763000011444092, 1.1990000009536743]  
recv_distance_time: [14608510, 14608510]           
active: [True, True]                               
data_available: [False, False]                      
datas: ['', '']                                                                        
```


- **sys_time** is LPS Time provide directly by UWB module.(unit is ms)
- **remote_node_num** shows all the remote node appeared, include the node that we have loss the signal.
- **self_id** is the id of UWB node itself which will be recognize by other UWB nodes.
- **node_ids** is the list of UWB node id appeared.
Every id is unique and in the range of [0,10] now.
- **node_dis** is the last measurement of distance of the indicated node. When node disappear, this value will keep the last value.
- **recv_distance_time** is the time when that node recieve last distance measurement, if also we have message recieved, this time will also be the time of recieve message.
- **active** is if the node is still active now, when the node disappear, this value will be *False*.
- **data_available** is if last tick receive data from corrspending node.
- **datas** shows what message they receievd when there a data received this tick, if there is no message received, this value comes *""*.

Example of **/uwb_node/incoming_broadcast_data** is show below
```bash
header:
  seq: 1
  stamp:
    secs: 1531917314
    nsecs: 641238927
  frame_id: ''
remote_id: 3
remote_recv_time: 15453781
lps_time: 15453801
data: "TestSendingdata123"
```

it publish everytime a broadcast message from some node is received.

- **remote_id** is the id of who publish this message
- **remote_recv_time** is when we receive this messsage, we use distance measurement time(ms).
- **lps_time** is UWB system LPS time(ms).
- **data** is the data we received.

## Subscribe
The node subscribe only
```bash
/uwb_node/send_broadcast_data
```

sending string data to this node just send a message to every node in the system.


```
rostopic pub /uwb_node/send_broadcast_msg std_msgs/String TestSendinata123
```



## Auto Instal script

```sh
git clone https://github.com/xuhao1/SwarmAutoInstall.git
cd SwarmAutoInstall
./auto_install.sh
```