SERVICE="hwserver"

export LD_LIBRARY_PATH=/home/valtr/run/lua-5.3.1/lib:$LD_LIBRARY_PATH

cd /home/valtr/cvs/gwyhwserver_gwyscope

#COMMAND="env LD_LIBRARY_PATH=/home/valtr/run/lua-5.3.1/lib:$LD_LIBRARY_PATH /home/valtr/cvs/gwyhwserver_gwyscope/hwserver 51000 > /dev/null"
COMMAND="./hwserver 51000"

if pgrep "$SERVICE" > /dev/null ; then
    echo "$SERVICE is running"
else
    #echo "$SERVICE is NOT running"
    nohup $COMMAND & 
fi

