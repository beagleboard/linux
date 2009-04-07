/*
 * linux/arch/arm/plat-omap/bootreason.c
 *
 * OMAP Bootreason passing
 *
 * Copyright (c) 2004 Nokia
 *
 * Written by David Weinehall <david.weinehall@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <mach/board.h>

static char boot_reason[16];

static int omap_bootreason_read_proc(char *page, char **start, off_t off,
					 int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf(page + len, "%s\n", boot_reason);

	*start = page + off;

	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len  : count;
}

static int __init bootreason_init(void)
{
	const struct omap_boot_reason_config *cfg;
	int reason_valid = 0;

	cfg = omap_get_config(OMAP_TAG_BOOT_REASON, struct omap_boot_reason_config);
	if (cfg != NULL) {
		strncpy(boot_reason, cfg->reason_str, sizeof(cfg->reason_str));
		boot_reason[sizeof(cfg->reason_str)] = 0;
		reason_valid = 1;
	} else {
		/* Read the boot reason from the OMAP registers */
	}

	if (!reason_valid)
		return -ENOENT;

	printk(KERN_INFO "Bootup reason: %s\n", boot_reason);

	if (!create_proc_read_entry("bootreason", S_IRUGO, NULL,
					omap_bootreason_read_proc, NULL))
		return -ENOMEM;

	return 0;
}

late_initcall(bootreason_init);
