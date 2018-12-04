addresses="172.20.10.4 192.168.43.12"
for ip in $addresses; do
    ping -q -w1 $ip > /dev/null
    if [ "$?" -eq "0" ] 
    then 
        PI_REMOTE=$ip 
        break 
    fi 
done 
echo pi@$PI_REMOTE

