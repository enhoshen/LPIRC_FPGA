IP=$2
TARGET=$1 
make 
scp $1 root@$2:/home/root
