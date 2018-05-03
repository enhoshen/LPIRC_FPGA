IP=$2
TARGET=$1 
if ["$3" = "-m"]
then
    make
fi    
scp -o StrictHostKeyChecking=no -r $1 root@$2:/home/root
