IP=$2
TARGET=$1 
if [ "$3" == "-m" ] #space is required between bracket and variable
then
    make
fi    
scp -o StrictHostKeyChecking=no -r $TARGET root@$IP:/home/root
