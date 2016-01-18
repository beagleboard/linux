#! /bin/bash
#
#  defconfig_merge.sh
#
#  This script will do a full kernel build with sparse information, clean
#  the build, rebuild the kernel again.  Upon each build a log will be created
#  if there are warnings.  Any errors and the whole script exits.
#
#  Copyright (C) 2013-14 Texas Instruments Incorporated - http://www.ti.com/
#  ALL RIGHTS RESERVED

export unix_user=`whoami`
export user_home_directory=`getent passwd $unix_user |cut -d: -f6`

BUILD_THREADS=`grep -c "^processor" /proc/cpuinfo`
DEFCONFIG=""
CROSS_COMPILE=""
WORKING_PATH=""
LOAD_ADDR=0x80008000
LOAD_ADDRESS_VALUE=""
LOGGING_DIRECTORY="working_config"

prepare_for_exit()
{
	set_original_directory
}

check_for_compiler()
{
	COMPILER_COMMAND=$(which $CROSS_COMPILE"gcc")
	if [ -x "$COMPILER_COMMAND" ]; then
		echo "Found " "$CROSS_COMPILE"
	else
		echo "Invalid or non-existent compiler $COMPILER_COMMAND" > build_failure.txt
		return 1
	fi

        CCACHE_INSTALLED=$(which ccache)
        if [ -z "$CCACHE_INSTALLED" ]; then
		echo "To decrease build time install ccache"
        else
		CCACHE="ccache"
        fi

	CROSS_COMPILE=""$CCACHE" "$CROSS_COMPILE""
	echo "Cross compile command is ""$CROSS_COMPILE"
}

build_the_defconfig()
{
	BUILD_OUT=`mktemp`
	#Check for defconfig issues
	echo "Making the $DEFCONFIG"
	cross_make $DEFCONFIG > $BUILD_OUT 2>&1
	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wc "error:"`
	if [ "$NUM_OF_ERRORS" -gt '0' ];then
		rm $BUILD_OUT
		return 1
	fi

	NUM_OF_WARNINGS=`cat $BUILD_OUT | grep -wc "warning:"`
	if [ -f $OUT_LOG ]; then
		IS_THIS_LOGGED=`cat $OUT_LOG | grep -wc "Defconfig build"`
		if [ "$IS_THIS_LOGGED" = 0 ]; then
			echo "#########################################################" >>  $OUT_LOG
			echo "# $DEFCONFIG Defconfig build                             " >> $OUT_LOG
			echo "#########################################################" >>  $OUT_LOG
			if [ "$NUM_OF_WARNINGS" -gt 0 ]; then
				echo >> $OUT_LOG
				cat $DEFCONFIG_OUT >> $OUT_LOG
			fi
			echo >> $OUT_LOG
			echo  "There are a total of "$NUM_OF_WARNINGS" warnings in this build" >> $OUT_LOG
			echo >> $OUT_LOG
		fi
	fi

	rm $BUILD_OUT
}

build_the_new_defconfig()
{
	BUILD_OUT=`mktemp`
	#Check for defconfig issues
	echo "Making the $NEW_DEFCONFIG"
	cross_make $NEW_DEFCONFIG > $BUILD_OUT 2>&1

	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wc "error:"`
	if [ "$NUM_OF_ERRORS" -gt '0' ];then
		rm $BUILD_OUT
		return 1
	fi

	NUM_OF_WARNINGS=`cat $BUILD_OUT | grep -wc "warning:"`
	if [ -f $OUT_LOG ]; then
		IS_THIS_LOGGED=`cat $OUT_LOG | grep -wc "Defconfig build"`
		if [ "$IS_THIS_LOGGED" = 0 ]; then
			echo "#########################################################" >>  $OUT_LOG
			echo "# $NEW_DEFCONFIG Defconfig build                         " >> $OUT_LOG
			echo "#########################################################" >>  $OUT_LOG
			if [ "$NUM_OF_WARNINGS" -gt 0 ]; then
				echo >> $OUT_LOG
				cat $NEW_DEFCONFIG >> $OUT_LOG
			fi
			echo >> $OUT_LOG
			echo  "There are a total of "$NUM_OF_WARNINGS" warnings in this build" >> $OUT_LOG
			echo >> $OUT_LOG
		fi
	fi

	rm $BUILD_OUT
}

build_the_kernel()
{
	BUILD_OUT=`mktemp`
	echo "Building the kernel"
	if [ -n "$LOAD_ADDR" ]; then
		LOAD_ADDRESS_VALUE="LOADADDR=$LOAD_ADDR"
	fi

	cross_make $LOAD_ADDRESS_VALUE uImage > $BUILD_OUT 2>&1
	#Check for build errors
	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wci "error:"`
	if [ "$NUM_OF_ERRORS" -gt '0' ];then
		cat $BUILD_OUT > $OUT_LOG
		rm $BUILD_OUT
		return 1
	fi
	NUM_OF_WARNINGS=`cat $BUILD_OUT | grep -wc "warning:"`
	echo "#########################################################" >> $OUT_LOG
	echo "# Kernel Compiler Warnings" >> $OUT_LOG
	echo "#########################################################" >> $OUT_LOG
	if [ "$NUM_OF_WARNINGS" -gt 0 ]; then
		echo  >> $OUT_LOG
		cat $BUILD_OUT | grep 'warning:' >> $OUT_LOG
	fi
	echo  >> $OUT_LOG
	echo  "There are a total of "$NUM_OF_WARNINGS" warnings in this build" >> $OUT_LOG
	echo  >> $OUT_LOG

	rm $BUILD_OUT
}

build_the_dtbs()
{
	BUILD_OUT=`mktemp`
	echo "Building the device tree"
	cross_make dtbs > $BUILD_OUT 2>&1
	#Check for build errors
	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wci "error:"`
	if [ "$NUM_OF_ERRORS" -gt '0' ];then
		cat $BUILD_OUT >> $OUT_LOG
		rm $BUILD_OUT
		return 1
	fi

	NUM_OF_WARNINGS=`cat $BUILD_OUT | grep -wc "warning:"`
	echo "#########################################################" >> $OUT_LOG
	echo "# Device Tree Warnings" >> $OUT_LOG
	echo "#########################################################" >> $OUT_LOG
	if [ "$NUM_OF_WARNINGS" -gt 0 ]; then
		echo  >> $OUT_LOG
		cat $BUILD_OUT | grep 'warning:' >> $OUT_LOG
	fi
	echo  >> $OUT_LOG
	echo  "There are a total of "$NUM_OF_WARNINGS" warnings in the device tree" >> $OUT_LOG
	echo  >> $OUT_LOG

	rm $BUILD_OUT
}

build_the_modules()
{
	BUILD_OUT=`mktemp`
	echo "Building the modules"
	cross_make modules > $BUILD_OUT 2>&1
	#Check for build errors
	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wci "error:"`
	if [ "$NUM_OF_ERRORS" -gt '0' ];then
		cat $BUILD_OUT >> $OUT_LOG
		rm $BUILD_OUT
		return 1
	fi

	NUM_OF_WARNINGS=`cat $BUILD_OUT | grep -wc "warning:"`
	echo "#########################################################" >> $OUT_LOG
	echo "# Module Build Warnings" >> $OUT_LOG
	echo "#########################################################" >> $OUT_LOG
	if [ "$NUM_OF_WARNINGS" -gt 0 ]; then
		echo  >> $OUT_LOG
		cat $BUILD_OUT | grep 'warning:' >> $OUT_LOG
	fi
	echo  >> $OUT_LOG
	echo  "There are a total of "$NUM_OF_WARNINGS" warnings in the modules" >> $OUT_LOG
	echo  >> $OUT_LOG

	cross_make tar-pkg > $BUILD_OUT 2>&1
	#Check for build errors
	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wci "error:"`
	if [ "$NUM_OF_ERRORS" -gt '0' ];then
		cat $BUILD_OUT >> $OUT_LOG
		rm $BUILD_OUT
		return 1
	fi

	rm $BUILD_OUT
}

clean_the_build()
{
	cross_make mrproper
}

setup_directories()
{
	INVOKE_DIR=`pwd`
	SCRIPT_DIR=`dirname $0`
	if [ -n "$WORKING_PATH" ]; then
		# invoking script for a different kernel...
		KERNEL_DIR="$WORKING_PATH"
	else
		# The script is located in <kernel_dir>/ti_config_fragments directory..
		# Lets use that trick to find the kernel directory.
		KERNEL_DIR=`dirname $SCRIPT_DIR`
		WORKING_PATH=$KERNEL_DIR
	fi

	# Sanity checkup kernel build location.
	if [ ! -d "$KERNEL_DIR" ]; then
		echo "Kernel working dir $KERNEL_DIR is not a directory/ does not exist! exiting.."
		exit 1
	fi

	if [ -n "$DEFCONFIG_EXTRAS_FILE" ]; then
		DEFCONFIG_EXTRAS_DIR=`dirname $DEFCONFIG_EXTRAS_FILE`
		DEFCONFIG_EXTRAS_KERNEL_DIR=`dirname $DEFCONFIG_EXTRAS_DIR`
		DEFCONFIG_EXTRAS_FILE_NAME=`basename $DEFCONFIG_EXTRAS_FILE`
		DEFCONFIG_EXTRAS_FILE="$DEFCONFIG_EXTRAS_DIR/$DEFCONFIG_EXTRAS_FILE_NAME"
		if [ ! -e "$DEFCONFIG_EXTRAS_FILE" ]; then
			echo "Kernel fragments file $DEFCONFIG_EXTRAS_FILE does not exist/is not readable!"
			usage
			exit 1
		fi
		if [ ! -d "$DEFCONFIG_EXTRAS_DIR" ]; then
			echo "Kernel fragments dir $DEFCONFIG_EXTRAS_DIR does not exist/is not a directory?"
			usage
			exit 1
		fi
	fi

	# if just the appended config is provided
	if [ -n "$APPENDED_CONFIG" ]; then
		APPENDED_CONFIG_DIR=`dirname $APPENDED_CONFIG`
		APPENDED_CONFIG_FILE_NAME=`basename $APPENDED_CONFIG`
		APPENDED_CONFIG="$APPENDED_CONFIG_DIR/$APPENDED_CONFIG_FILE_NAME"
	fi

}

set_working_directory()
{
	ORIGINAL_DIR=`pwd`
	if [ "$ORIGINAL_DIR" != "$WORKING_PATH" ]; then
		cd $WORKING_PATH
	fi
}

set_original_directory()
{
	CURRENT_DIR=`pwd`
	if [ "$CURRENT_DIR" != "$ORIGINAL_DIR" ]; then
		cd $ORIGINAL_DIR
	fi
}

cross_make()
{
	make -C$KERNEL_DIR -j$BUILD_THREADS ARCH=arm $*
}

usage()
{
cat << EOF
This script will either take in a single defconfig fragment file or a
file with multiple fragments defined.  With either file the base defconfig
will initially be built, copied to a working directory, and the kernels merge_config
script will be called for each config file fragment.  Each config file fragment will be appended
to the base config and then the final output will be copied back to the kernels working
directory.

A copy of both the base config and the final config is stored in the scripts logging directory.

Single fragment command line example:

	defconfig_merge.sh -c <path to the compiler> -o <path to output log file>
	-e <path to single fragment file> -w <path to the kernel directory>

Multiple fragment file format:
"use-kernel-config=" - Is required to be defined.  This is the base defconfig
that the config fragments will be appended to.  This defconfig should be in the
Linux kernel path in arch/arm/configs.

	use-kernel-config= < defconfig base to start appending fragments >

"config-fragment=" - Is required to be defined.  There should be at least one
entry per config fragment.

	config-fragment= < path to the defconfig fragment file 1>

Multiple fragment command line example:

	defconfig_merge.sh -c <path to the compiler> -o <path to output log file>
	-f <path to multiple fragment file> -w <path to the kernel directory>

Example:
use-kernel-config=omap2plus_defconfig
config-fragment=ti_config_fragments/baseport.cfg
config-fragment=ti_config_fragments/power.cfg
config-fragment=ti_config_fragments/connectivity.cfg
config-fragment=ti_config_fragments/ipc.cfg
config-fragment=ti_config_fragments/audio_display.cfg
config-fragment=ti_config_fragments/system_test.cfg


OPTIONS:
	Setup Options:
	-c  The path to the desired compiler
	-d  The defconfig to use as a base
	-e  Single defconfig fragment to append
	-j  How many build threads to use
	-o  Output log file
	-l  uImage load address if different from 0x80008000 for no load address use ""

	Build Options:
	-n  Do not pre-build the defconfig just use the existing .config
	-m  MAKE the kernel, modules and dtb files based on the new config
	-w  Linux kernel directory

	-f  Path to file with multiple defconfig options

EOF
}

while getopts “c:j:d:e:o:l:mnbf:w:” OPTION
do
	case $OPTION in
		d)
			DEFCONFIG=$OPTARG;;
		e)
			APPENDED_CONFIG=$OPTARG;;
		c)
			CROSS_COMPILE=$OPTARG;;
		j)
			BUILD_THREADS=$OPTARG;;
		o)
			OUT_LOG=$OPTARG;;
		l)
			LOAD_ADDR=$OPTARG;;
		n)
			NO_CLEAN_DEFCONFIG=1;;
		m)
			BUILD_ALL=1;;
		f)
			DEFCONFIG_EXTRAS_FILE=$OPTARG;;
		w)
			WORKING_PATH=$OPTARG;;
		?)
			usage
			exit;;
     esac
done

setup_directories
trap prepare_for_exit EXIT SIGINT SIGTERM


if [ "$BUILD_ALL" == 1 -o "$NO_CLEAN_DEFCONFIG" != 1 ]; then
	if [ -z "$CROSS_COMPILE" ]; then
		echo "Missing cross compile"
		usage
		exit 1
	fi
fi

if [ -z "$DEFCONFIG_EXTRAS_FILE" -a -z "$APPENDED_CONFIG" ]; then
	echo "Missing config fragment information"
	usage
	exit 1
fi


if [ -z "$OUT_LOG" ]; then
	echo "Missing output log path, dumping to /dev/null"
	OUT_LOG=/dev/null
fi

set_working_directory

LOGGING_DIRECTORY="$WORKING_PATH""/ti_config_fragments/""$LOGGING_DIRECTORY"
if [ ! -d "$LOGGING_DIRETORY" ];then
	echo -e "\n\tRemoving $LOGGING_DIRECTORY"
	rm -rf $LOGGING_DIRECTORY
fi

echo -e "\n\tCreating $LOGGING_DIRECTORY for final configuration files\n"
mkdir -p $LOGGING_DIRECTORY

check_for_compiler
if [ $? -ne 0 ]; then
	 echo -e "\n\tCannot find $CROSS_COMPILE compiler\n"
	 exit 1
fi

if [ -z "$DEFCONFIG" ];then
	if [ -n "$DEFCONFIG_EXTRAS_FILE"  ]; then
		DEFCONFIG=`cat $DEFCONFIG_EXTRAS_FILE | grep "use-kernel-config=" | cut -d= -f2`
		if [ -z "$DEFCONFIG"  ]; then
			echo -e "\n\tMissing base defconfig in the file\n"
			usage
			exit 1
		fi
		echo "Using base config $DEFCONFIG from the file $DEFCONFIG_EXTRAS_FILE"
	else
		echo "Missing a defconfig cannot proceed"
		usage
		exit 1
	fi
fi

DEFCONFIG_EXTRAS="$LOGGING_DIRECTORY/merged_$DEFCONFIG"

if [ "$NO_CLEAN_DEFCONFIG" != 1 ]; then
	clean_the_build
	build_the_defconfig
	if [ $? -ne 0 ]; then
		 echo "Building $DEFCONFIG failed"
		 exit 1
	fi
else
	if [ -e ".config" ]; then
		echo "The .config shows it exists"
	else
		echo "The .config did not exist already"
		exit 1
	fi
fi

if [ -a .config ]; then
	cp -v .config $LOGGING_DIRECTORY/base_config
fi

# There is only one file passed in via command line
if [ -z "$DEFCONFIG_EXTRAS_FILE" ]; then
	echo "Only appending $APPENDED_CONFIG"
	$KERNEL_DIR/scripts/kconfig/merge_config.sh -m -r -O $LOGGING_DIRECTORY $LOGGING_DIRECTORY/base_config $APPENDED_CONFIG
	if [ $? -ne 0 ]; then
		echo "Failed to merge config $APPENDED_CONFIG"
		exit 1
	fi
else
	TEMP_FRAGMENT=`mktemp`
	cat $DEFCONFIG_EXTRAS_FILE | grep "config-fragment=" | cut -d= -f2 > $TEMP_FRAGMENT
	NUM_OF_FRAGMENTS=`wc -l $TEMP_FRAGMENT | awk '{print$1}'`
	if [ -z "$NUM_OF_FRAGMENTS" ]; then
		echo "Malformed defconfig fragment file"
		rm $TEMP_FRAGMENT
		exit 1
	fi
	cat $LOGGING_DIRECTORY/base_config > $LOGGING_DIRECTORY/temp_config
	while read APPENDED_CONFIG
	do
		if [ -z "$APPENDED_CONFIG" ]; then
			break
		fi
		if [ ! -r "$APPENDED_CONFIG" ]; then
			# If not using absolute path, then assume we are relative to kernel dir
			# of the defconfig_extras
			APPENDED_CONFIG=$DEFCONFIG_EXTRAS_KERNEL_DIR/$APPENDED_CONFIG
		fi
		if [ ! -r "$APPENDED_CONFIG" ]; then
			echo "Failed to find $APPENDED_CONFIG"
			usage
			exit 1
		fi

		$KERNEL_DIR/scripts/kconfig/merge_config.sh -m -r -O $LOGGING_DIRECTORY $LOGGING_DIRECTORY/temp_config $APPENDED_CONFIG
		if [ $? -ne 0 ]; then
			echo "Failed to merge config $APPENDED_CONFIG"
			rm $TEMP_FRAGMENT
			exit 1
		fi
		cat $APPENDED_CONFIG >> $DEFCONFIG_EXTRAS
		cat $LOGGING_DIRECTORY/.config > $LOGGING_DIRECTORY/temp_config
	done <$TEMP_FRAGMENT
	rm $TEMP_FRAGMENT
fi

# The final defconfig to create
NEW_DEFCONFIG="appended_$DEFCONFIG"

# Last step before the build
cat $LOGGING_DIRECTORY/.config > $LOGGING_DIRECTORY/final_config
echo -e "\n\t Copying $LOGGING_DIRECTORY/.config to $KERNEL_DIR/.config"
cp -v $LOGGING_DIRECTORY/.config $KERNEL_DIR/.config
echo -e "\n\t Copying $LOGGING_DIRECTORY/.config to arch/arm/configs/$NEW_DEFCONFIG"
cp -v $LOGGING_DIRECTORY/.config $KERNEL_DIR/arch/arm/configs/$NEW_DEFCONFIG

if [ -a $LOGGING_DIRECTORY/temp_config ]; then
	rm $LOGGING_DIRECTORY/temp_config
fi

if [ "$BUILD_ALL" == 1 ]; then
	build_the_new_defconfig
	if [ $? -ne 0 ]; then
		 echo "Building the defconfig failed"
		 exit 1
	fi

	build_the_kernel
	if [ $? -ne 0 ]; then
		 echo "Building the kernel failed"
		 exit 1
	fi

	build_the_dtbs
	if [ $? -ne 0 ]; then
		 echo "Building the device tree binaries failed"
		 exit 1
	fi

	build_the_modules
	if [ $? -ne 0 ]; then
		 echo "Building the modules failed"
		 exit 1
	fi
fi
