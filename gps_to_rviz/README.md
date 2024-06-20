这是基于ros和nemv gps驱动的实时位置（xy坐标）显示文件
第一步，连接gps传感器
1.首先将gps连接至电脑 检查 ls /dev 是否有ttyACM0之类的新串口产生
2.sudo cat /dev/ttyACM0来读取原始gps数据
第二步，安装gps驱动程序（ros noteic自带）
1.https://wiki.ros.org/nmea_navsat_driver安装方法及说明
2.安装成功后运行roscore
3.roscd nmea_navsat_driver/launch/
  sudo vim nmea_serial_driver.launch （也可以使用nano等编辑器修改）
启动gps驱动设置文件
将port修改为新出现的串口如ttyACM0
并修改baud参数为9600
4.给串口权限	sudo chmod 777 /dev/ttyACM0
5.运行nmea_serial_driver 节点：	roslaunch nmea_navsat_driver nmea_serial_driver.launch
6.查看话题输出 rostopic echo /fix 
出现header及经纬度即为运行成功。
其他说明
nmea_navsat_driver提供四个节点：
nmea_topic_driver，nmea_serial_driver，nmea_topic_serial_reader和nmea_socket_driver
第三步，运行git中的坐标实时转换程序为xyz坐标（可使用终端输入rviz（路径模拟器））来查看，也可以使用ros record功能录制rosbag文件来回放。
1.创建工作空间
2.修改launch文件
3.每次修改后要catkin_make编译出新的文件
4.source ~/catkin_ws/devel/setup.bash或添加进bashsrc中
5.roslaunch +launch绝对地址
可以在终端检查实时转换坐标同时可以利用rviz来查看轨迹
