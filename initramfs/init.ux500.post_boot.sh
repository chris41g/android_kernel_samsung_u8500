#!/system/bin/sh
#
#target=`getprop ro.product.device`
#
#case "$target" in
#    "GT-I9070" )
#	echo "ondemand" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
#        ;;
#    "GT-I8160" )
#	echo "ondemand" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
#        ;;
#    "GT-I8530" )
#	echo "ondemand" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
#        ;;
#esac
echo "ondemand" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
