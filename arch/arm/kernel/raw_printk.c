#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/init.h>

void printch(int);

static void raw_console_write(struct console *co,
			      const char *s, unsigned count)
{
	while (count-- > 0) {
		if (*s == '\n')
			printch('\r');
		printch(*s++);
	}
}

static struct console raw_console = {
	.name		= "rawcon",
	.write		= raw_console_write,
	.write_raw	= raw_console_write,
	.flags		= CON_PRINTBUFFER | CON_RAW,
	.index		= -1,
};

static int __init raw_console_init(void)
{
	register_console(&raw_console);

	return 0;
}

console_initcall(raw_console_init);
