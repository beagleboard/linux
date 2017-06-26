#! /bin/sh

me=`basename $0`
usage='usage: $me [--split] [--help] [reference]'
split=no

while test $# -gt 0; do
    case "$1" in
    --split)
	split=yes
	;;
    --help)
	echo "$usage"
	exit 0
	;;
    *)
	if [ -n "$reference" ]; then
	    echo "$me: unknown flag: $1" >&2
	    echo "$usage" >&2
	    exit 1
	fi
	reference="$1"
	;;
    esac
    shift
done

VERSION=`sed 's/^VERSION = \(.*\)/\1/;t;d' Makefile`
PATCHLEVEL=`sed 's/^PATCHLEVEL = \(.*\)/\1/;t;d' Makefile`
SUBLEVEL=`sed 's/^SUBLEVEL = \(.*\)/\1/;t;d' Makefile`
EXTRAVERSION=`sed 's/^EXTRAVERSION = \(.*\)/\1/;t;d' Makefile`

if [ -z "$SUBLEVEL" -o "$SUBLEVEL" = "0" ]; then
    kvers="$VERSION.$PATCHLEVEL"
elif [ -z "$EXTRAVERSION" ]; then
    kvers="$VERSION.$PATCHLEVEL.$SUBLEVEL"
else
    kvers="$VERSION.$PATCHLEVEL.$SUBLEVEL.$EXTRAVERSION"
fi

if [ -z "$reference" ]; then
    reference="v$kvers"
fi

echo reference: $reference, kernel version: $kvers

git diff "$reference" | awk -v kvers="$kvers" -v splitmode="$split" \
'function set_current_arch(a)
{
    if (!outfiles[a]) {
	mt = "mktemp /tmp/XXXXXX"
	mt | getline outfiles[a]
	close(mt)
    }
    current_arch=a
    current_file=outfiles[a]
}

BEGIN {
    driver_arch["cpuidle/Kconfig"]="noarch"
    driver_arch["tty/serial/8250/8250_core.c"]="noarch"
    driver_arch["iommu/irq_remapping.c"]="noarch"

    driver_arch["clk/mxs/clk-imx28.c"]="arm"
    driver_arch["clocksource/mxs_timer.c"]="arm"
    driver_arch["clocksource/arm_global_timer.c"]="arm"
    driver_arch["clocksource/pxa_timer.c"]="arm"
    driver_arch["clocksource/timer-atmel-pit.c"]="arm"
    driver_arch["gpio/gpio-davinci.c"]="arm"
    driver_arch["gpio/gpio-mxc.c"]="arm"
    driver_arch["gpio/gpio-mxs.c"]="arm"
    driver_arch["gpio/gpio-omap.c"]="arm"
    driver_arch["gpio/gpio-pxa.c"]="arm"
    driver_arch["gpio/gpio-sa1100.c"]="arm"
    driver_arch["gpio/gpio-davinci.c"]="arm"
    driver_arch["gpio/gpio-mvebu.c"]="arm"
    driver_arch["gpio/gpio-zynq.c"]="arm"
    driver_arch["irqchip/irq-versatile-fpga.c"]="arm"
    driver_arch["irqchip/spear-shirq.c"]="arm"
    driver_arch["irqchip/irq-mxs.c"]="arm"
    driver_arch["irqchip/irq-s3c24xx.c"]="arm"
    driver_arch["irqchip/irq-vic.c"]="arm"
    driver_arch["irqchip/irq-atmel-aic.c"]="arm"
    driver_arch["irqchip/irq-atmel-aic5.c"]="arm"
    driver_arch["irqchip/irq-omap-intc.c"]="arm"
    driver_arch["irqchip/irq-bcm7120-l2.c"]="arm"
    driver_arch["irqchip/irq-brcmstb-l2.c"]="arm"
    driver_arch["irqchip/irq-dw-apb-ictl.c"]="arm"
    driver_arch["irqchip/irq-sunxi-nmi.c"]="arm"
    driver_arch["irqchip/irq-crossbar.c"]="arm"
    driver_arch["mfd/twl4030-irq.c"]="arm"
    driver_arch["mfd/twl6030-irq.c"]="arm"
    driver_arch["dma/edma.c"]="arm"
    driver_arch["misc/Kconfig"]="arm"
    driver_arch["net/ethernet/cadence/at91_ether.c"]="arm"
    driver_arch["gpu/ipu-v3/ipu-common.c"]="arm"
    driver_arch["gpu/ipu-v3/ipu-prv.h"]="arm"
    driver_arch["memory/omap-gpmc.c"]="arm"
    driver_arch["pinctrl/pinctrl-at91.c"]="arm"
    driver_arch["pinctrl/pinctrl-rockchip.c"]="arm"
    driver_arch["pinctrl/pinctrl-single.c"]="arm"
    driver_arch["tty/serial/xilinx_uartps.c"]="arm"
    driver_arch["clk/imx/clk-imx51-imx53.c"]="arm"
    driver_arch["clocksource/timer-imx-gpt.c"]="arm"
    driver_arch["irqchip/irq-sa11x0.c"]="arm"
    driver_arch["media/platform/vsp1/vsp1_video.c"]="arm"
    driver_arch["soc/dove/pmu.c"]="arm"

    driver_arch["irqchip/irq-gic.c"]="arm arm64"
    driver_arch["irqchip/irq-gic-v3.c"]="arm arm64"
    driver_arch["clocksource/arm_arch_timer.c"]="arm arm64"
    driver_arch["clocksource/timer-sp804.c"]="arm arm64"
    driver_arch["tty/serial/amba-pl011.c"]="arm arm64"
    driver_arch["gpio/gpio-pl061.c"]="arm arm64"

    driver_arch["tty/serial/bfin_uart.c"]="blackfin"

    driver_arch["tty/serial/mpc52xx_uart.c"]="powerpc"
    driver_arch["gpio/gpio-mpc8xxx.c"]="powerpc"
    driver_arch["soc/fsl/qe/qe_ic.c"]="powerpc"

    driver_arch["clocksource/i8253.c"]="x86"
    driver_arch["clocksource/Makefile"]="x86"
    driver_arch["clocksource/ipipe_i486_tsc_emu.S"]="x86"
    driver_arch["pci/htirq.c"]="x86"
}

match($0, /^diff --git a\/arch\/([^ \t\/]*)/) {
    split(substr($0, RSTART, RLENGTH), arch, /\//)
    a=arch[3]

    is_multiarch=0
    set_current_arch(a)
    print $0 >> current_file
    next
}

match($0, /^diff --git a\/drivers\/([^ \t]*)/) {
    file=substr($0, RSTART, RLENGTH)
    sub(/^diff --git a\/drivers\//, "", file)
    f=file

    if (!driver_arch[f]) {
	 print "Error unknown architecture for driver "f
	 unknown_file_error=1
    } else {
        a = driver_arch[f]
        if(index(a, " ")) {
            is_multiarch = 1
            split(a, multiarch, " ")
            for(a in multiarch) {
                set_current_arch(multiarch[a])
                print $0 >> current_file
            }
        } else {
            is_multiarch = 0
            set_current_arch(a)
            print $0 >> current_file
        }
        next
    }
}

/^diff --git a\/scripts\/ipipe\/genpatches.sh/ {
    is_multiarch=0
    if (splitmode == "no") {
	current_file="/dev/null"
	current_arch="nullarch"
	next
    }
}

/^diff --git/ {
    set_current_arch("noarch")
    is_multiarch=0
    print $0 >> current_file
    next
}

match ($0, /#define [I]PIPE_CORE_RELEASE[ \t]*([^ \t]*)/) {
    split(substr($0, RSTART, RLENGTH), vers, /[ \t]/)
    version[current_arch]=vers[3]
}

{
    if(is_multiarch) {
        for(a in multiarch) {
            set_current_arch(multiarch[a])
            print $0 >> current_file
        }
    } else {
        print $0 >> current_file
    }
}

END {
    close(outfiles["noarch"])
    for (a in outfiles) {
	if (unknown_file_error) {
	    if (a != "noarch")
		system("rm "outfiles[a])
	} else if (a != "noarch") {
	    dest="ipipe-core-"kvers"-"a"-"version[a]".patch"
	    close(outfiles[a])
	    system("mv "outfiles[a]" "dest)
	    if (splitmode == "no")
		system("cat "outfiles["noarch"]" >> "dest)
	    print dest
	} else if (splitmode == "yes") {
	    dest="ipipe-core-"kvers"-"a".patch"
	    system("cat "outfiles["noarch"]" > "dest)
	    print dest
	}
    }

    system("rm "outfiles["noarch"])
}
'
