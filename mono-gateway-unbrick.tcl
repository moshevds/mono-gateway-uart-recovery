# Minimal Mono Gateway DK LS1046A unbrick helper.
#
# Run like this with a JLink:
#   openocd \
#       -f interface/jlink.cfg \
#       -c "transport select jtag" \
#       -f target/ls1046a.cfg \
#       -f mono-gateway-unbrick.tcl
#       -c "mono_gateway_unbrick /path/to/semihosting/folder/ bl2_semihost.bin"
#
# Or, run this with a CJMCU-2112:
#   openocd \
#       -f cp2112-jtag-gpiod.cfg \
#       -c "transport select jtag" \
#       -f target/ls1046a.cfg \
#       -f mono-gateway-unbrick.tcl \
#       -c "mono_gateway_unbrick /path/to/empty/folder/ /path/to/mono-uart-recovery-stub.bin"
#

proc select_rcw_source_9e {} {
	irscan ls1046a.sap 0x92 -endstate RUN/IDLE
	set words [split [string trim [drscan ls1046a.sap 32 0x00000000 32 0x00000000 -endstate RUN/IDLE]]]
	if {[llength $words] != 2} { error "unexpected RCW source selector read: $words" }
	set lo [expr 0x[lindex $words 0]]
	set hi [expr 0x[lindex $words 1]]
	set lo [expr {($lo & 0xfffffc00) | 0x13d}]
	set hi [expr {$hi & 0xfffffbff}]
	irscan ls1046a.sap 0x93 -endstate RUN/IDLE
	drscan ls1046a.sap 32 [format "0x%08x" $lo] 32 [format "0x%08x" $hi] -endstate RUN/IDLE
}

proc write_scan {hi16 cmd_low cmd_high data} {
	irscan ls1046a.sap 0x21 -endstate RUN/IDLE
	drscan ls1046a.sap 16 $hi16 -endstate RUN/IDLE
	irscan ls1046a.sap 0xf3 -endstate RUN/IDLE
	sleep 2
	irscan ls1046a.sap 0x24 -endstate RUN/IDLE
	drscan ls1046a.sap 32 $cmd_low 32 $cmd_high -endstate RUN/IDLE
	irscan ls1046a.sap 0xf3 -endstate RUN/IDLE
	sleep 2
	irscan ls1046a.sap 0x25 -endstate RUN/IDLE
	sleep 2
	irscan ls1046a.sap 0x25 -endstate RUN/IDLE
	drscan ls1046a.sap 32 $data -endstate RUN/IDLE
	irscan ls1046a.sap 0x25 -endstate RUN/IDLE
	sleep 2
}
proc pbi_flush {} {
	write_scan 0x0000 0x01580800 0x00000157 0x00000300
	irscan ls1046a.sap 0x21 -endstate RUN/IDLE
	drscan ls1046a.sap 16 0x0000 -endstate RUN/IDLE
	irscan ls1046a.sap 0x24 -endstate RUN/IDLE
	drscan ls1046a.sap 32 0x01581800 32 0x00000157 -endstate RUN/IDLE
	irscan ls1046a.sap 0x25 -endstate RUN/IDLE
	drscan ls1046a.sap 32 0x00000000 -endstate RUN/IDLE
}

proc stage_rcw {} {
	irscan ls1046a.sap 0x22 -endstate RUN/IDLE
	sleep 1
	write_scan 0x0002 0x01000800 0x00002014 0x0c150010
	write_scan 0x0002 0x01040800 0x00002014 0x0e000000
	write_scan 0x0002 0x01080800 0x00002014 0x00000000
	write_scan 0x0002 0x010c0800 0x00002014 0x00000000
	write_scan 0x0002 0x01100800 0x00002014 0x11335a06
	write_scan 0x0002 0x01140800 0x00002014 0x40000012
	write_scan 0x0002 0x01180800 0x00002014 0xf0000000
	write_scan 0x0002 0x011c0800 0x00002014 0xc1000000
	write_scan 0x0002 0x01200800 0x00002014 0x00000000
	write_scan 0x0002 0x01240800 0x00002014 0x00000000
	write_scan 0x0002 0x01280800 0x00002014 0x00000000
	write_scan 0x0002 0x012c0800 0x00002014 0x00430804
	write_scan 0x0002 0x01300800 0x00002014 0x20104400
	write_scan 0x0002 0x01340800 0x00002014 0x24400000
	write_scan 0x0002 0x01380800 0x00002014 0x00000096
	write_scan 0x0002 0x013c0800 0x00002014 0x00000001
	write_scan 0x0002 0x00d00800 0x00002014 0x00000c00
}

proc gateway_pbi {} {
	write_scan 0x0000 0x06000800 0x00000157 0x00000000
	write_scan 0x0000 0x06040800 0x00000157 0x10000000
	write_scan 0x0000 0x040c0800 0x00000157 0x00000000
	write_scan 0x0000 0x01780800 0x00000157 0x0000e010
	write_scan 0x0000 0x00000800 0x00000118 0x00000008
	write_scan 0x0000 0x04180800 0x00000157 0x0000009e
	write_scan 0x0000 0x041c0800 0x00000157 0x0000009e
	write_scan 0x0000 0x04200800 0x00000157 0x0000009e
	pbi_flush

	write_scan 0x0000 0x08bc0800 0x00000340 0x01000000
	write_scan 0x0000 0x01540800 0x00000340 0x47474747
	write_scan 0x0000 0x01580800 0x00000340 0x47474747
	write_scan 0x0000 0x08bc0800 0x00000340 0x00000000
	write_scan 0x0000 0x08bc0800 0x00000360 0x01000000
	write_scan 0x0000 0x01540800 0x00000360 0x47474747
	write_scan 0x0000 0x08bc0800 0x00000360 0x00000000
	pbi_flush

	write_scan 0x0000 0x08900800 0x00000340 0x01048000
	write_scan 0x0000 0x08900800 0x00000360 0x01048000
	pbi_flush

	write_scan 0x0000 0x00980800 0x00000340 0x00000000
	write_scan 0x0000 0x00980800 0x00000360 0x00000000
	pbi_flush
}

proc debug_release {} {
	write_scan 0x0000 0x07000800 0x00002016 0x80000000
	write_scan 0x0000 0x07040800 0x00002016 0x80000000
	write_scan 0x0000 0x07080800 0x00002016 0x80000000
	write_scan 0x0000 0x070c0800 0x00002016 0x80000000
	write_scan 0x0002 0x001c0800 0x00002016 0x0000000e
	write_scan 0x0002 0x005c0800 0x00002016 0x0000000e
	write_scan 0x0000 0x06800800 0x00000157 0x00000001
}

proc mono_gateway_unbrick {directory file} {
	if {$directory eq "" || $file eq ""} { error "usage: mono_gateway_unbrick <directory> <file>" }
	if {![string match "/*" $file]} { set file [file join $directory $file] }
	if {![file exists $file]} { error "BL2 image not found: $file" }

	echo "Mono Gateway DK unbrick: RCW/PBI bring-up"
	adapter speed 1000
	reset_config srst_only srst_nogate srst_push_pull connect_assert_srst
	init
	scan_chain
	adapter assert srst
	sleep 1
	select_rcw_source_9e
	irscan ls1046a.sap 0x91 -endstate RUN/IDLE
	drscan ls1046a.sap 32 0x00000004 32 0x00000000 -endstate RUN/IDLE
	sleep 100
	adapter deassert srst
	sleep 10
	irscan ls1046a.sap 0x69 -endstate RUN/IDLE
	set reset_state [drscan ls1046a.sap 32 0x00000000 32 0x00000000 -endstate RUN/IDLE]
	set reset_state [lrange $reset_state 0 end]
	if {$reset_state ne {000001a0 00000000}} { error "unexpected reset_state={$reset_state}; expected={000001a0 00000000}" }
	stage_rcw
	sleep 10
	irscan ls1046a.sap 0x69 -endstate RUN/IDLE
	set reset_state [drscan ls1046a.sap 32 0x00000000 32 0x00000000 -endstate RUN/IDLE]
	set reset_state [lrange $reset_state 0 end]
	if {$reset_state ne {00000180 00000000}} { error "unexpected reset_state={$reset_state}; expected={00000180 00000000}" }
	irscan ls1046a.sap 0x91 -endstate RUN/IDLE
	drscan ls1046a.sap 32 0x00000000 32 0x00000000 -endstate RUN/IDLE
	gateway_pbi
	debug_release

	echo "Mono Gateway DK unbrick: loading BL2"
	catch {adapter speed 10000}
	core_up 0
	targets ls1046a.cpu0
	load_image $file 0x10000000 bin
	arm semihosting enable
	arm semihosting_basedir $directory
	resume
}
