vlib work
vcom -2002 *.vhd
vlog *.v
vsim Memoria_tb
add wave *
run -all
quit
