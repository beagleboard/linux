#!/bin/bash
#
#  defconfig_builder.sh
#
#  This will perform a merged based on a file that contains information in the
#  repos to be merged.
#
#  For more information type defconfig_builder.sh -h
#
#  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
#  ALL RIGHTS RESERVED

PROCESSOR_TAG="processor:"
BUILD_TYPE_TAG="type:"
CONFIG_FRAGMENT_TAG="config-fragment="

set_working_directory()
{
	# Sanity checkup kernel build location.
	if [ ! -d "$WORKING_PATH" ]; then
		echo "Kernel working dir $WORKING_PATH is not a directory/ does not exist! exiting.."
		exit 1
	fi

	ORIGINAL_DIR=$(pwd)
	if [ "$ORIGINAL_DIR" != "$WORKING_PATH" ]; then
		cd "$WORKING_PATH"
		WORKING_PATH=$(pwd)
	fi

	TI_WORKING_PATH="$WORKING_PATH/ti_config_fragments"
	DEFCONFIG_KERNEL_PATH="$WORKING_PATH/arch/arm/configs"
	DEFCONFIG_MAP_FILE="$TI_WORKING_PATH/defconfig_map.txt"
}

prepare_for_exit() {
	rm -f "$PROCESSOR_FILE"
	rm -f "$BUILD_TYPE_FILE"
	exit
}

get_processors() {
	TEMP_PROC_FILE=$(mktemp)
	cat "$DEFCONFIG_MAP_FILE" > "$TEMP_PROC_FILE"

	y=0
	while true;
	do
		PROCESSOR_TEMP=$(grep "$PROCESSOR_TAG" "$TEMP_PROC_FILE" | awk '{print$2}' | head -n 1)
		if [ -z "$PROCESSOR_TEMP" ]; then
			break
		fi
		if ! grep -qc "$PROCESSOR_TEMP" "$PROCESSOR_FILE"; then
			y=$((y+1))
			echo -e '\t'"$y". "$PROCESSOR_TEMP" >> "$PROCESSOR_FILE"
		fi
		sed -i "1d" "$TEMP_PROC_FILE"
	done

	rm "$TEMP_PROC_FILE"
}

choose_processor() {
	get_processors

	NUM_OF_PROC=$(wc -l "$PROCESSOR_FILE" | awk '{print$1}')
	# Force the user to answer.  Maybe the user does not want to continue
	while true;
	do
		cat "$PROCESSOR_FILE"
		read -p "Please choose which processor to build for: " REPLY
		if [ "$REPLY" -gt '0' -a "$REPLY" -le "$NUM_OF_PROC" ]; then
			CHOOSEN_PROCESSOR=$(grep -w "$REPLY" "$PROCESSOR_FILE" | awk '{print$2}')
			break
		else
			echo -e "\nThis is not a choice try again!\n"
		fi
	done
}

choose_build_type() {
	TEMP_BT_FILE=$(mktemp)
	TEMP_BUILD_FILE=$(mktemp)

	grep "$CHOOSEN_PROCESSOR" "$DEFCONFIG_MAP_FILE" | grep "$BUILD_TYPE_TAG" | awk '{print$4}' > "$TEMP_BUILD_FILE"

	y=0
	while true;
	do
		CONFIG_FILE=
		CONFIG_FRAGMENTS=

		BT_TEMP=$(head -n 1 "$TEMP_BUILD_FILE")
		if [ -z "$BT_TEMP" ]; then
			break
		fi
		BUILD_DETAILS=$(grep -w "$BT_TEMP" "$DEFCONFIG_MAP_FILE")

		# Check to make sure that the config fragments exist
		TEMP_EXTRA_CONFIG_FILE=$(echo "$BUILD_DETAILS" | cut -d: -f6)
		if [ -z "$TEMP_EXTRA_CONFIG_FILE" ]; then
			CONFIG_FRAGMENTS=
		else
			for CONFIG_FRAG in $TEMP_EXTRA_CONFIG_FILE;
			do
				if [ ! -e "$CONFIG_FRAG" ]; then
					CONFIG_FRAGMENTS="N/A"
				fi
			done
		fi

		if ! grep -qc "$BT_TEMP" "$BUILD_TYPE_FILE"; then
			# If the config file and config fragments are available
			# add it to the list.
			CONFIG_FILE=$(echo "$BUILD_DETAILS" | awk '{print$8}')
			if [ "$CONFIG_FILE" = "None" ]; then
				CONFIG_FILE=
			else
				if [ -e "$TI_WORKING_PATH""/""$CONFIG_FILE" ]; then
					CONFIG_FILE=
				fi
			fi
			# If the check for the config file and the config fragments
			# pass then these two variables should be empty.  If
			# they fail then they should be N/A.
			if [ -z "$CONFIG_FILE" -a -z "$CONFIG_FRAGMENTS" ]; then
				y=$((y+1))
				echo -e '\t'"$y". "$BT_TEMP" >> "$BUILD_TYPE_FILE"
			fi
		fi
		sed -i "1d" "$TEMP_BUILD_FILE"
	done

	NUM_OF_BUILDS=$(wc -l "$BUILD_TYPE_FILE" | awk '{print$1}')
	# Force the user to answer.  Maybe the user does not want to continue
	while true;
	do
		cat "$BUILD_TYPE_FILE"
		read -p "Please choose which build: " REPLY
		if [ "$REPLY" -gt '0' -a "$REPLY" -le "$NUM_OF_BUILDS" ]; then
			CHOOSEN_BUILD_TYPE=$(grep -w "$REPLY" "$BUILD_TYPE_FILE" | awk '{print$2}')
			break
		else
			echo -e "\nThis is not a choice try again!\n"
		fi
	done
	rm "$TEMP_BT_FILE"
	rm "$TEMP_BUILD_FILE"
}

get_build_details() {

	BUILD_DETAILS=$(grep -w "$CHOOSEN_BUILD_TYPE" "$DEFCONFIG_MAP_FILE")
	DEFCONFIG=$(echo "$BUILD_DETAILS" | awk '{print$6}')
	DEFCONFIG="$DEFCONFIG_KERNEL_PATH""/""$DEFCONFIG"
	CONFIG_FILE=$(echo "$BUILD_DETAILS" | awk '{print$8}')
	# There may be a need to just build with the config fragments themselves
	if [ "$CONFIG_FILE" = "None" ]; then
		CONFIG_FILE=
	fi
	TEMP_EXTRA_CONFIG_FILE=$(echo "$BUILD_DETAILS" | cut -d: -f6)
	for CONFIG_FRAG in $TEMP_EXTRA_CONFIG_FILE;
	do
		if [ -e "$CONFIG_FRAG" ]; then
			EXTRA_CONFIG_FILE="$EXTRA_CONFIG_FILE $CONFIG_FRAG"
		else
			echo "$CONFIG_FRAG" does not exist
		fi
	done
}

usage()
{
cat << EOF

This script utilizes a map file to create defconfigs for multiple TI
platforms.

There is only one option for this script.  This option defines the working
path to where the Linux kernel resides.

	-w - Location of the TI Linux kernel

Command line Example if building from the working Linux kernel
top level directory:
	ti_config_fragments/defconfig_builder.sh -w .

Command line Example if building from the ti_config_fragments directory:
	defconfig_builder.sh -w ../.

EOF
}

#########################################
# Script Start
#########################################
while getopts "w:" OPTION
do
	case $OPTION in
		w)
			WORKING_PATH=$OPTARG;;
		?)
			usage
			exit;;
     esac
done

trap prepare_for_exit EXIT SIGINT SIGTERM

set_working_directory

if [ ! -e "$DEFCONFIG_MAP_FILE" ]; then
	echo "No defconfig map file found"
	exit 1
fi

PROCESSOR_FILE=$(mktemp)
BUILD_TYPE_FILE=$(mktemp)

choose_processor
choose_build_type
get_build_details

if [ ! -z "$CONFIG_FILE" -a -e "$TI_WORKING_PATH/$CONFIG_FILE" ]; then
	CONFIGS=$(grep "$CONFIG_FRAGMENT_TAG" "$TI_WORKING_PATH/$CONFIG_FILE" | cut -d= -f2)
fi

STATUS=$("$WORKING_PATH"/scripts/kconfig/merge_config.sh -m -r "$DEFCONFIG" "$CONFIGS" "$EXTRA_CONFIG_FILE")
if [ "$?" = "0" ];then
	echo "Creating defconfig file ""$WORKING_PATH""/arch/arm/configs/""$CHOOSEN_BUILD_TYPE"_defconfig
	cp .config "$DEFCONFIG_KERNEL_PATH"/"$CHOOSEN_BUILD_TYPE"_defconfig
else
	echo "Defconfig creation failed"
fi
