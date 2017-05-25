#! /bin/sh

set -e

shift

awk '
match($0, /COBALT_SYSCALL\([^,]*,[ \t]*[^,]*/)  {
	str=substr($0, RSTART + 15, RLENGTH - 15)
	match(str, /[^, \t]*/)
	syscall=substr(str, RSTART, RLENGTH)

	if (syscall == "") {
		print "Failed to find syscall name in line " $0 > "/dev/stderr"
		exit 1
	}

	calls = calls "	__COBALT_CALL_ENTRY(" syscall ") \\\n"
	modes = modes "	__COBALT_MODE(" str ") \\\n"
	next
}

/COBALT_SYSCALL\(/  {
	print "Failed to parse line " $0 > "/dev/stderr"
	exit 1
}

END {
	print "#define __COBALT_CALL_ENTRIES \\\n" calls "	/* end */"
	print "#define __COBALT_CALL_MODES \\\n" modes "	/* end */"
}
' $*
