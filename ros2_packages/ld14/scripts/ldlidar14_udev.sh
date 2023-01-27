echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="LD14"' >/etc/udev/rules.d/ld14_lidar_ch9102.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="LD14"' >/etc/udev/rules.d/ld14_lidar_cp2102.rules
service udev reload
sleep 2
service udev restart


