IP=$2
TARGET=$1 
make 
scp -o StrictHostKeyChecking=no $1 root@$2:/home/root
