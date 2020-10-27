arm-linux-gnueabihf-g++ spu_test.c -o spu_test
sshpass -p '1' scp spu_test root@192.168.0.76:/media/fat/
#sshpass -p '1' ssh root@192.168.0.76
sshpass -p '1' ssh root@192.168.0.76 '/media/fat/spu_test'

