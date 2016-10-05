#!/bin/bash
#
#  defconfig_builder.sh
#
#  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
#  ALL RIGHTS RESERVED
#
#  This script will perform a merge of config fragment files into a defconfig
#  based on a map file.  The map file defines the defconfig options that have
#  been tested by TI to boot and compile.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  For more information type defconfig_builder.sh -?
#
# IMPORTANT NOTE: All modifications to this script must be verified by running
# 'shellcheck --shell=bash defconfig_builder.sh' See 'Installing' section in:
# https://github.com/koalaman/shellcheck/blob/master/README.md
# Also note that shellcheck must be at least at revision 0.3.3

DEBUG_CONFIG_TAG="debug_options"
CONFIG_FRAGMENT_TAG="config-fragment="
DISCLAIMER="\n*Please be advised that the Debug Option defconfigs may\nimpact \
performance and should only be used for debugging.\n"

# Template for temporary build files.. use PID to differentiate
TMP_PREFIX=ti_defconfig_builder_$$
TMP_TEMPLATE="$TMP_PREFIX"_XXXXX.tmp

set_working_directory() {
	# Sanity checkup kernel build location.
	if [ ! -d "$WORKING_PATH" ]; then
		WORKING_PATH=$(pwd)
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
	D=$(dirname "$PROCESSOR_FILE")
	rm -f "$PROCESSOR_FILE"
	rm -f "$BUILD_TYPE_FILE"
	rm -f "$TEMP_TYPE_FILE"
	if [ -s "$OLD_CONFIG" ]; then
		mv "$OLD_CONFIG" "$WORKING_PATH"/.config
	fi
	# Clean everyone else up if we missed any
	rm -f "$D"/"$TMP_PREFIX"*.tmp
	exit
}

check_for_config_existance() {
	# Check to make sure that the config fragments exist
	TEMP_EXTRA_CONFIG_FILE=$(echo "$BUILD_DETAILS" | cut -d: -f6)
	if [ -z "$TEMP_EXTRA_CONFIG_FILE" ]; then
		CONFIG_FRAGMENTS=
	else
		for CONFIG_FRAGMENT_FILE in $TEMP_EXTRA_CONFIG_FILE;
		do
			# If we do already point to existing file, we are good.
			if [ -e "$CONFIG_FRAGMENT_FILE" ]; then
				CONFIG_FRAG="$CONFIG_FRAGMENT_FILE"
			else
				# Assume it is present in TI working path
				CONFIG_FRAG="$TI_WORKING_PATH/$CONFIG_FRAGMENT_FILE"
			fi
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
			max_configs=$((max_configs+1))
			echo -e '\t'"$max_configs". "$BT_TEMP" >> "$BUILD_TYPE_FILE"
		fi
	fi
}

choose_build_type() {
	TEMP_BT_FILE=$(mktemp -t $TMP_TEMPLATE)
	TEMP_BUILD_FILE=$(mktemp -t $TMP_TEMPLATE)

	grep "$DEFCONFIG_FILTER" "$DEFCONFIG_MAP_FILE" | grep "^classification:" | awk '{print$4}' > "$TEMP_BUILD_FILE"

	max_configs=0
	while true;
	do
		CONFIG_FILE=
		CONFIG_FRAGMENTS=

		BT_TEMP=$(head -n 1 "$TEMP_BUILD_FILE")
		if [ -z "$BT_TEMP" ]; then
			break
		fi
		BUILD_DETAILS=$(grep -w "$BT_TEMP" "$DEFCONFIG_MAP_FILE")
		check_for_config_existance
		sed -i "1d" "$TEMP_BUILD_FILE"
	done

	NUM_OF_BUILDS=$(wc -l "$BUILD_TYPE_FILE" | awk '{print$1}')
	if [ "$NUM_OF_BUILDS" -eq 0 ]; then
		echo "Sorry no build targets for this configuration.  Are you on the right branch?"
		prepare_for_exit
	fi

	# Force the user to answer.  Maybe the user does not want to continue
	while true;
	do
		echo -e "Available ""$DEFCONFIG_FILTER"" defconfig build options:\n"
		cat "$BUILD_TYPE_FILE"
		echo ""
		read -p "Please enter the number of the defconfig to build or 'q' to exit: " REPLY

		if [ "$REPLY" = "q" -o "$REPLY" = "Q" ]; then
			prepare_for_exit
		elif ! [[ "$REPLY" =~ ^[0-9]+$ ]]; then
			echo -e "\n$REPLY is not a number of the defconfig.  Please try again!\n"
			continue
		elif [ "$REPLY" -gt '0' -a "$REPLY" -le "$NUM_OF_BUILDS" ]; then
			CHOSEN_BUILD_TYPE=$(grep -w "$REPLY" "$BUILD_TYPE_FILE" | awk '{print$2}')
			break
		else
			echo -e "\n'$REPLY' is not a valid choice. Please \
choose a value between '1' and '$max_configs':\n"
		fi
	done
	rm "$TEMP_BT_FILE"
	rm "$TEMP_BUILD_FILE"
}

list_all_targets() {

	TMP_MAP=$(mktemp -t $TMP_TEMPLATE)
	cat "$DEFCONFIG_MAP_FILE" > "$TMP_MAP"
	while true;
	do
		CONFIG_FILE=
		CONFIG_FRAGMENTS=

		BT_TEMP=$(head -n 1 "$TMP_MAP" | awk '{print$4}')
		BUILD_DETAILS=$(head -n 1 "$TMP_MAP")
		if [ -z "$BUILD_DETAILS" ]; then
			break
		fi
		check_for_config_existance
		sed -i "1d" "$TMP_MAP"
	done
	rm "$TMP_MAP"
	cat "$BUILD_TYPE_FILE"
}

get_build_details() {

	BUILD_DETAILS=$(grep -w "$CHOSEN_BUILD_TYPE" "$DEFCONFIG_MAP_FILE")
	if [ -z "$BUILD_DETAILS" ]; then
		echo "Cannot find the build type or a match for $CHOSEN_BUILD_TYPE"
		TEMP_BUILD_FILE=$(mktemp -t $TMP_TEMPLATE)
		grep "$CHOSEN_BUILD_TYPE" "$DEFCONFIG_MAP_FILE" > "$TEMP_BUILD_FILE"
		while true;
		do
			CONFIG_FILE=
			CONFIG_FRAGMENTS=

			BT_TEMP=$(head -n 1 "$TEMP_BUILD_FILE" | awk '{print$4}')
			if [ -z "$BT_TEMP" ]; then
				break
			fi
			BUILD_DETAILS=$(grep -w "$BT_TEMP" "$DEFCONFIG_MAP_FILE")
			check_for_config_existance
			sed -i "1d" "$TEMP_BUILD_FILE"
		done
		rm -rf "$TEMP_BUILD_FILE"

		NUM_OF_BUILDS=$(wc -l "$BUILD_TYPE_FILE" | awk '{print$1}')
		if [ "$NUM_OF_BUILDS" -eq 0 ]; then
			echo "Maybe try one of the following:"
			list_all_targets
		else
			echo "Did you mean any of the following?"
			cat "$BUILD_TYPE_FILE"
		fi

		return 1
	fi

	DEFCONFIG=$(echo "$BUILD_DETAILS" | awk '{print$6}')
	DEFCONFIG="$DEFCONFIG_KERNEL_PATH""/""$DEFCONFIG"
	CONFIG_FILE=$(echo "$BUILD_DETAILS" | awk '{print$8}')
	# There may be a need to just build with the config fragments themselves
	if [ "$CONFIG_FILE" = "None" ]; then
		CONFIG_FILE=
	fi

	if [ ! -e "$TI_WORKING_PATH/$CONFIG_FILE" ]; then
		echo "$TI_WORKING_PATH/$CONFIG_FILE does not exist"
		return 1
	fi

	TEMP_EXTRA_CONFIG_FILE=$(echo "$BUILD_DETAILS" | cut -d: -f6)
	for CONFIG_FRAGMENT_FILE in $TEMP_EXTRA_CONFIG_FILE;
	do
		# If we do already point to existing file, we are good.
		if [ -e "$CONFIG_FRAGMENT_FILE" ]; then
			CONFIG_FRAG="$CONFIG_FRAGMENT_FILE"
		else
			# Assume it is present in TI working path
			CONFIG_FRAG="$TI_WORKING_PATH/$CONFIG_FRAGMENT_FILE"
		fi
		if [ -e "$CONFIG_FRAG" ]; then
			EXTRA_CONFIG_FILE="$EXTRA_CONFIG_FILE $CONFIG_FRAG"
		else
			echo "$CONFIG_FRAG" does not exist
		fi
	done
}

build_defconfig() {

	if [ ! -z "$CONFIG_FILE" -a -e "$TI_WORKING_PATH/$CONFIG_FILE" ]; then
		CONFIGS=$(grep "$CONFIG_FRAGMENT_TAG" "$TI_WORKING_PATH/$CONFIG_FILE" | cut -d= -f2)
	fi

	"$WORKING_PATH"/scripts/kconfig/merge_config.sh -m -r "$DEFCONFIG" \
		"$CONFIGS" "$EXTRA_CONFIG_FILE" > /dev/null

	if [ "$?" = "0" ];then
		echo "Creating defconfig file ""$WORKING_PATH""/arch/arm/configs/""$CHOSEN_BUILD_TYPE"_defconfig
		mv .config "$DEFCONFIG_KERNEL_PATH"/"$CHOSEN_BUILD_TYPE"_defconfig
	else
		echo "Defconfig creation failed"
		return 1
	fi
}

choose_defconfig_type() {

	TEMP_TYPE_FILE=$(mktemp -t $TMP_TEMPLATE)

	TYPE_FILE=$(awk '{print$2}' "$DEFCONFIG_MAP_FILE" | sort -u | grep -i "SDK_")

	max_types=0
	for TYPE_TMP in $TYPE_FILE;
	do
		max_types=$((max_types+1))
		echo -e '\t' "$max_types." "$TYPE_TMP" >> "$TEMP_TYPE_FILE"
	done
	echo >> "$TEMP_TYPE_FILE"

	while true;
	do
		cat "$TEMP_TYPE_FILE"
		read -p "Please choose a defconfig type to build for or 'q' to exit: " REPLY
		if [ "$REPLY" = "q" -o "$REPLY" = "Q" ]; then
			prepare_for_exit
		elif ! [[ "$REPLY" =~ ^[0-9]+$ ]]; then
			echo -e "\n'$REPLY' is not a number for the build type.  Please try again!\n"
			continue
		elif [ "$REPLY" -gt '0' -a "$REPLY" -le "$max_types" ]; then
			REPLY="$REPLY""."
			DEFCONFIG_FILTER=$(awk '{if ($1 == "'"$REPLY"'") print $2;}' "$TEMP_TYPE_FILE")
			break
		else
			echo -e "\n'$REPLY' is not a valid choice. Please \
choose a value between '1' and '$max_types':\n"
		fi
	done

	DEBUG_BUILD=$(grep "$DEFCONFIG_FILTER" "$DEFCONFIG_MAP_FILE" | grep -wc "$DEBUG_CONFIG_TAG" )
	if [ "$DEBUG_BUILD" -gt '0' ]; then
		echo -e "$DISCLAIMER"
	fi
}

usage() {
cat << EOF

This script will perform a merge of config fragment files into a defconfig
based on a map file.  The map file defines the defconfig options that have
been tested by TI to boot and compile.

Optional:
	-w - Location of the TI Linux kernel
	-t - Indicates the type of defconfig to build.  This will force the
	     defconfig to build without user interaction.
	-l - List all buildable defconfig options

Command line example to generate the TI SDK AM335x processor defconfig automatically
without user interaction:

	ti_config_fragments/defconfig_builder.sh -t ti_sdk_am3x_release

Command line Example if building from the ti_config_fragments directory:
	defconfig_builder.sh -w ../.

User interactive command line example:
	ti_config_fragments/defconfig_builder.sh
EOF
}

#########################################
# Script Start
#########################################
while getopts "?w:t:l" OPTION
do
	case $OPTION in
		w)
			WORKING_PATH=$OPTARG;;
		t)
			CHOSEN_BUILD_TYPE=$OPTARG;;
		l)
			LIST_TARGETS="y";;
		?)
			usage
			exit;;
     esac
done

trap prepare_for_exit SIGHUP EXIT SIGINT SIGTERM

set_working_directory

if [ ! -e "$DEFCONFIG_MAP_FILE" ]; then
	echo "No defconfig map file found"
	exit 1
fi

BUILD_TYPE_FILE=$(mktemp -t $TMP_TEMPLATE)

if [ ! -z "$LIST_TARGETS" ]; then
	echo "The following are a list of buildable defconfigs:"
	list_all_targets
	exit 0
fi

PROCESSOR_FILE=$(mktemp -t $TMP_TEMPLATE)
OLD_CONFIG=$(mktemp -t $TMP_TEMPLATE)
if [ -f "$WORKING_PATH"/.config ]; then
	mv "$WORKING_PATH"/.config "$OLD_CONFIG"
fi

if [ ! -z "$CHOSEN_BUILD_TYPE" ]; then
	get_build_details
	if [ "$?" -gt 0 ]; then
		exit 1
	fi

	build_defconfig
	if [ "$?" -gt 0 ]; then
		exit 1
	fi
	exit 0
fi

choose_defconfig_type

choose_build_type
get_build_details

build_defconfig
