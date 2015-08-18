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

BUILD_THREADS=`grep "^processor" /proc/cpuinfo | wc -l`
DEFCONFIG="omap2plus_defconfig"
NEW_DEFCONFIG="appended_omap2plus_defconfig"
CROSS_COMPILE=
WORKING_PATH="linux-kernel"
LOAD_ADDR=0x80008000
LOAD_ADDRESS_VALUE=
LOGGING_DIRECTORY="working_config"

prepare_for_exit()
{
	set_original_directory
}

check_for_compiler()
{
	COMPILER_COMMAND=`which $CROSS_COMPILE"gcc"`
	if [ -f "$COMPILER_COMMAND" ]; then
		return 0
	else
		echo "Compiler $COMPILER_COMMAND does not exist on this host" > build_failure.txt
		return 1
	fi
}

build_the_defconfig()
{
	BUILD_OUT=`mktemp`
	#Check for defconfig issues
	echo "Making the $DEFCONFIG"
	make -j$BUILD_THREADS ARCH=arm CROSS_COMPILE=$CROSS_COMPILE $DEFCONFIG > $BUILD_OUT 2>&1
	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wc error`
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
	make -j$BUILD_THREADS ARCH=arm CROSS_COMPILE=$CROSS_COMPILE $NEW_DEFCONFIG > $BUILD_OUT 2>&1

	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wc error`
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
	if [ "$LOAD_ADDR" != "" ]; then
		LOAD_ADDRESS_VALUE="LOADADDR=$LOAD_ADDR"
	fi

	make -j$BUILD_THREADS ARCH=arm CROSS_COMPILE=$CROSS_COMPILE $LOAD_ADDRESS_VALUE uImage > $BUILD_OUT 2>&1
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
	make -j$BUILD_THREADS ARCH=arm CROSS_COMPILE=$CROSS_COMPILE dtbs > $BUILD_OUT 2>&1
	#Check for build errors
	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wci error`
	if [ "$NUM_OF_ERRORS" -gt '0' ];then
		cat $BUILD_OUT >> $OUT_LOG
		rm $BUILD_OUT
		return 1
	fi

	NUM_OF_WARNINGS=`cat $BUILD_OUT | grep -wc warnings`
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
	make -j$BUILD_THREADS ARCH=arm CROSS_COMPILE=$CROSS_COMPILE modules > $BUILD_OUT 2>&1
	#Check for build errors
	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wci error`
	if [ "$NUM_OF_ERRORS" -gt '0' ];then
		cat $BUILD_OUT >> $OUT_LOG
		rm $BUILD_OUT
		return 1
	fi

	NUM_OF_WARNINGS=`cat $BUILD_OUT | grep -wc warnings`
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

	make -j$BUILD_THREADS ARCH=arm CROSS_COMPILE=$CROSS_COMPILE tar-pkg > $BUILD_OUT 2>&1
	#Check for build errors
	NUM_OF_ERRORS=`cat $BUILD_OUT | grep -wci error`
	if [ "$NUM_OF_ERRORS" -gt '0' ];then
		cat $BUILD_OUT >> $OUT_LOG
		rm $BUILD_OUT
		return 1
	fi

	rm $BUILD_OUT
}

clean_the_build()
{
	make -j$BUILD_THREADS mrproper
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

	defconfig_merge.sh -c <path to the compiler> -o <output directory and file name>
	-e <path to single fragment file> -w <path to the working directory of the kernel>

Multiple fragment file format:
"use-kernel-config=" - Is required to be defined.  This is the base defconfig
that the config fragments will be appended to.  This defconfig should be in the
Linux kernel path in arch/arm/configs.

	use-kernel-config= < defconfig base to start appending fragments >

"config-fragment=" - Is required to be defined.  There should be at least one
entry per config fragment.

	config-fragment= < path to the defconfig fragment file 1>

Multiple fragment command line example:

	defconfig_merge.sh -c <path to the compiler> -o <output directory and file name>
	-f <path to multiple fragment file> -w <path to the working directory of the kernel>

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
	-o  Output log
	-l  uImage load address if different from 0x80008000 for no load address use ""

	Build Options:
	-n  Do not pre-build the defconfig just use the existing .config
	-m  MAKE the kernel, modules and dtb files based on the new config
	-w  Linux kernel path directory

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

if [ "$BUILD_ALL" != 1 -a "$NO_CLEAN_DEFCONFIG" != 1 ]; then
	if [ "$CROSS_COMPILE" == "" ]; then
		echo "Missing cross compile"
		usage
		exit 1
	fi
fi

if [ "$DEFCONFIG_EXTRAS_FILE" == "" -a "$APPENDED_CONFIG" == "" ]; then
	echo "Missing config fragment information"
	usage
	exit 1
fi


if [ "$OUT_LOG" == "" ]; then
	echo "Missing output log path"
	usage
	exit 1
fi

set_working_directory

LOGGING_DIRECTORY="$WORKING_PATH""ti_config_fragments/""$LOGGING_DIRECTORY"
if [ ! -d "$LOGGING_DIRETORY" ];then
	echo -e "\n\tRemoving $LOGGING_DIRECTORY"
	rm -rf $LOGGING_DIRECTORY
fi

DEFCONFIG_EXTRAS="$LOGGING_DIRECTORY/merged_omap2plus_defconfig"

echo -e "\n\tCreating $LOGGING_DIRECTORY for final configuration files\n"
mkdir -p $LOGGING_DIRECTORY

check_for_compiler
if [ $? -ne 0 ]; then
	 echo -e "\n\tCannot find $CROSS_COMPILE compiler\n"
	 prepare_for_exit
	 exit 1
fi

if [ "$DEFCONFIG_EXTRAS_FILE" != '' ]; then
	FILE_DEFCONFIG=`cat $DEFCONFIG_EXTRAS_FILE | grep "use-kernel-config=" | cut -d= -f2`
	if [ "$FILE_DEFCONFIG" == '' ]; then
		echo -e "\n\tMissing base defconfig in the file\n"
		usage
		exit 1
	fi
	echo "Using base config $DEFCONFIG from the file $DEFCONFIG_EXTRAS_FILE"
fi

if [ "$NO_CLEAN_DEFCONFIG" != 1 ]; then
	clean_the_build
	build_the_defconfig
	if [ $? -ne 0 ]; then
		 echo "Building $DEFCONFIG failed"
		 prepare_for_exit
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

cp -v .config $LOGGING_DIRECTORY/base_config

# There is only one file passed in via command line
if [ "$DEFCONFIG_EXTRAS_FILE" == '' ]; then
	echo "Only appending $DEFCONFIG_EXTRAS_FILE"
	./scripts/kconfig/merge_config.sh -m -r -O $LOGGING_DIRECTORY $LOGGING_DIRECTORY/base_config $APPENDED_CONFIG
	if [ $? -ne 0 ]; then
		echo "Failed to merge config $APPENDED_CONFIG"
		exit 1
	fi
else
	TEMP_FRAGMENT=`mktemp`
	cat $DEFCONFIG_EXTRAS_FILE | grep "config-fragment=" | cut -d= -f2 > $TEMP_FRAGMENT
	NUM_OF_FRAGMENTS=`wc -l $TEMP_FRAGMENT | awk '{print$1}'`
	if [ "$NUM_OF_FRAGMENTS" == '' ]; then
		echo "Malformed defconfig fragment file"
		rm $TEMP_FRAGMENT
		exit 1
	fi
	cat $LOGGING_DIRECTORY/base_config > $LOGGING_DIRECTORY/temp_config
	while true;
	do
		APPENDED_CONFIG=`head -1 $TEMP_FRAGMENT`
		if [ "$APPENDED_CONFIG" == '' ]; then
			break
		fi
		./scripts/kconfig/merge_config.sh -m -r -O $LOGGING_DIRECTORY $LOGGING_DIRECTORY/temp_config $APPENDED_CONFIG
		if [ $? -ne 0 ]; then
			echo "Failed to merge config $APPENDED_CONFIG"
			rm $TEMP_FRAGMENT
			exit 1
		fi
		sed -i "1d" $TEMP_FRAGMENT
		cat $APPENDED_CONFIG >> $DEFCONFIG_EXTRAS
		cat $LOGGING_DIRECTORY/.config > $LOGGING_DIRECTORY/temp_config
	done
	rm $TEMP_FRAGMENT
fi

# Last step before the build
cat $LOGGING_DIRECTORY/.config > $LOGGING_DIRECTORY/final_config
echo -e "\n\t Copying $LOGGING_DIRECTORY/.config to $WORKING_DIRECTORY/.config"
cp -v $LOGGING_DIRECTORY/.config .config
echo -e "\n\t Copying $LOGGING_DIRECTORY/.config to arch/arm/configs/$NEW_DEFCONFIG"
cp -v $LOGGING_DIRECTORY/.config arch/arm/configs/$NEW_DEFCONFIG
rm $LOGGING_DIRECTORY/temp_config

if [ "$BUILD_ALL" == 1 ]; then
	build_the_new_defconfig
	if [ $? -ne 0 ]; then
		 echo "Building the defconfig failed"
		 prepare_for_exit
		 exit 1
	fi

	build_the_kernel
	if [ $? -ne 0 ]; then
		 echo "Building the kernel failed"
		 prepare_for_exit
		 exit 1
	fi

	build_the_dtbs
	if [ $? -ne 0 ]; then
		 echo "Building the device tree binaries failed"
		 prepare_for_exit
		 exit 1
	fi

	build_the_modules
	if [ $? -ne 0 ]; then
		 echo "Building the modules failed"
		 prepare_for_exit
		 exit 1
	fi
fi
