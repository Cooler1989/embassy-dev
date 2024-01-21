#  running openocd:
#     src/openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -s tcl
#  in: /home/pawelpyszko/programs/openocd_rp

target remote localhost:3333
mon reset halt
load
mon reset halt
continue

