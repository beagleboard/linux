/**
 * @file
 * Analogy for Linux, UAPI bits
 * @note Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>
 * @note Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _RTDM_UAPI_ANALOGY_H
#define _RTDM_UAPI_ANALOGY_H

/* --- Misc precompilation constant --- */
#define A4L_NAMELEN 20

#define A4L_INFINITE 0
#define A4L_NONBLOCK (-1)

/* --- Common Analogy types --- */

typedef unsigned short sampl_t;
typedef unsigned long lsampl_t;

/* MMAP ioctl argument structure */
struct a4l_mmap_arg {
	unsigned int idx_subd;
	unsigned long size;
	void *ptr;
};
typedef struct a4l_mmap_arg a4l_mmap_t;

/* Constants related with buffer size
   (might be used with BUFCFG ioctl) */
#define A4L_BUF_MAXSIZE 0x1000000
#define A4L_BUF_DEFSIZE 0x10000
#define A4L_BUF_DEFMAGIC 0xffaaff55

/* BUFCFG ioctl argument structure */
struct a4l_buffer_config {
	/* NOTE: with the last buffer implementation, the field
	   idx_subd became useless; the buffer are now
	   per-context. So, the buffer size configuration is specific
	   to an opened device. There is a little exception: we can
	   define a default buffer size for a device.
	   So far, a hack is used to implement the configuration of
	   the default buffer size */
	unsigned int idx_subd;
	unsigned long buf_size;
};
typedef struct a4l_buffer_config a4l_bufcfg_t;

/* BUFINFO ioctl argument structure */
struct a4l_buffer_info {
	unsigned int idx_subd;
	unsigned long buf_size;
	unsigned long rw_count;
};
typedef struct a4l_buffer_info a4l_bufinfo_t;

/* BUFCFG2 / BUFINFO2 ioctl argument structure */
struct a4l_buffer_config2 {
	unsigned long wake_count;
	unsigned long reserved[3];
};
typedef struct a4l_buffer_config2 a4l_bufcfg2_t;

/* POLL ioctl argument structure */
struct a4l_poll {
	unsigned int idx_subd;
	unsigned long arg;
};
typedef struct a4l_poll a4l_poll_t;

/* DEVCFG ioctl argument structure */
struct a4l_link_desc {
	unsigned char bname_size;
	char *bname;
	unsigned int opts_size;
	void *opts;
};
typedef struct a4l_link_desc a4l_lnkdesc_t;

/* DEVINFO ioctl argument structure */
struct a4l_dev_info {
	char board_name[A4L_NAMELEN];
	char driver_name[A4L_NAMELEN];
	int nb_subd;
	int idx_read_subd;
	int idx_write_subd;
};
typedef struct a4l_dev_info a4l_dvinfo_t;

#define CIO 'd'
#define A4L_DEVCFG _IOW(CIO,0,a4l_lnkdesc_t)
#define A4L_DEVINFO _IOR(CIO,1,a4l_dvinfo_t)
#define A4L_SUBDINFO _IOR(CIO,2,a4l_sbinfo_t)
#define A4L_CHANINFO _IOR(CIO,3,a4l_chinfo_arg_t)
#define A4L_RNGINFO _IOR(CIO,4,a4l_rnginfo_arg_t)
#define A4L_CMD _IOWR(CIO,5,a4l_cmd_t)
#define A4L_CANCEL _IOR(CIO,6,unsigned int)
#define A4L_INSNLIST _IOR(CIO,7,unsigned int)
#define A4L_INSN _IOR(CIO,8,unsigned int)
#define A4L_BUFCFG _IOR(CIO,9,a4l_bufcfg_t)
#define A4L_BUFINFO _IOWR(CIO,10,a4l_bufinfo_t)
#define A4L_POLL _IOR(CIO,11,unsigned int)
#define A4L_MMAP _IOWR(CIO,12,unsigned int)
#define A4L_NBCHANINFO _IOR(CIO,13,a4l_chinfo_arg_t)
#define A4L_NBRNGINFO _IOR(CIO,14,a4l_rnginfo_arg_t)

/* These IOCTLs are bound to be merged with A4L_BUFCFG and A4L_BUFINFO
   at the next major release */
#define A4L_BUFCFG2 _IOR(CIO,15,a4l_bufcfg_t)
#define A4L_BUFINFO2 _IOWR(CIO,16,a4l_bufcfg_t)

/*!
 * @addtogroup analogy_lib_async1
 * @{
 */

/*!
 * @anchor ANALOGY_CMD_xxx @name ANALOGY_CMD_xxx
 * @brief Common command flags definitions
 * @{
 */

/**
 * Do not execute the command, just check it
 */
#define A4L_CMD_SIMUL 0x1
/**
 * Perform data recovery / transmission in bulk mode
 */
#define A4L_CMD_BULK 0x2
/**
 * Perform a command which will write data to the device
 */
#define A4L_CMD_WRITE 0x4

	  /*! @} ANALOGY_CMD_xxx */

/*!
 * @anchor TRIG_xxx @name TRIG_xxx
 * @brief Command triggers flags definitions
 * @{
 */

/**
 * Never trigger
 */
#define TRIG_NONE	0x00000001
/**
 * Trigger now + N ns
 */
#define TRIG_NOW	0x00000002
/**
 * Trigger on next lower level trig
 */
#define TRIG_FOLLOW	0x00000004
/**
 * Trigger at time N ns
 */
#define TRIG_TIME	0x00000008
/**
 * Trigger at rate N ns
 */
#define TRIG_TIMER	0x00000010
/**
 * Trigger when count reaches N
 */
#define TRIG_COUNT	0x00000020
/**
 * Trigger on external signal N
 */
#define TRIG_EXT	0x00000040
/**
 * Trigger on analogy-internal signal N
 */
#define TRIG_INT	0x00000080
/**
 * Driver defined trigger
 */
#define TRIG_OTHER	0x00000100
/**
 * Wake up on end-of-scan
 */
#define TRIG_WAKE_EOS	0x0020
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_MASK 0x00030000
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_NEAREST 0x00000000
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_DOWN 0x00010000
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_UP 0x00020000
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_UP_NEXT 0x00030000

	  /*! @} TRIG_xxx */

/*!
 * @anchor CHAN_RNG_AREF @name Channel macros
 * @brief Specific precompilation macros and constants useful for the
 * channels descriptors tab located in the command structure
 * @{
 */

/**
 * Channel indication macro
 */
#define CHAN(a) ((a) & 0xffff)
/**
 * Range definition macro
 */
#define RNG(a) (((a) & 0xff) << 16)
/**
 * Reference definition macro
 */
#define AREF(a) (((a) & 0x03) << 24)
/**
 * Flags definition macro
 */
#define FLAGS(a) ((a) & CR_FLAGS_MASK)
/**
 * Channel + range + reference definition macro
 */
#define PACK(a, b, c) (a | RNG(b) | AREF(c))
/**
 * Channel + range + reference + flags definition macro
 */
#define PACK_FLAGS(a, b, c, d) (PACK(a, b, c) | FLAGS(d))

/**
 * Analog reference is analog ground
 */
#define AREF_GROUND 0x00
/**
 * Analog reference is analog common
 */
#define AREF_COMMON 0x01
/**
 * Analog reference is differential
 */
#define AREF_DIFF 0x02
/**
 * Analog reference is undefined
 */
#define AREF_OTHER 0x03

	  /*! @} CHAN_RNG_AREF */

#if !defined(DOXYGEN_CPP)

#define CR_FLAGS_MASK 0xfc000000
#define CR_ALT_FILTER (1<<26)
#define CR_DITHER CR_ALT_FILTER
#define CR_DEGLITCH CR_ALT_FILTER
#define CR_ALT_SOURCE (1<<27)
#define CR_EDGE	(1<<30)
#define CR_INVERT (1<<31)

#endif /* !DOXYGEN_CPP */

/*!
 * @brief Structure describing the asynchronous instruction
 * @see a4l_snd_command()
 */

struct a4l_cmd_desc {
	unsigned char idx_subd;
			       /**< Subdevice to which the command will be applied. */

	unsigned long flags;
			       /**< Command flags */

	/* Command trigger characteristics */
	unsigned int start_src;
			       /**< Start trigger type */
	unsigned int start_arg;
			       /**< Start trigger argument */
	unsigned int scan_begin_src;
			       /**< Scan begin trigger type */
	unsigned int scan_begin_arg;
			       /**< Scan begin trigger argument */
	unsigned int convert_src;
			       /**< Convert trigger type */
	unsigned int convert_arg;
			       /**< Convert trigger argument */
	unsigned int scan_end_src;
			       /**< Scan end trigger type */
	unsigned int scan_end_arg;
			       /**< Scan end trigger argument */
	unsigned int stop_src;
			       /**< Stop trigger type */
	unsigned int stop_arg;
			   /**< Stop trigger argument */

	unsigned char nb_chan;
			   /**< Count of channels related with the command */
	unsigned int *chan_descs;
			    /**< Tab containing channels descriptors */

	/* Driver specific fields */
	unsigned int valid_simul_stages;
			   /** < cmd simulation valid stages (driver dependent) */

	unsigned int data_len;
			   /**< Driver specific buffer size */
	sampl_t *data;
	                   /**< Driver specific buffer pointer */
};
typedef struct a4l_cmd_desc a4l_cmd_t;

/*! @} analogy_lib_async1 */

/* --- Range section --- */

/** Constant for internal use only (must not be used by driver
    developer).  */
#define A4L_RNG_FACTOR 1000000

/**
 * Volt unit range flag
 */
#define A4L_RNG_VOLT_UNIT 0x0
/**
 * MilliAmpere unit range flag
 */
#define A4L_RNG_MAMP_UNIT 0x1
/**
 * No unit range flag
 */
#define A4L_RNG_NO_UNIT 0x2
/**
 * External unit range flag
 */
#define A4L_RNG_EXT_UNIT 0x4

/**
 * Macro to retrieve the range unit from the range flags
 */
#define A4L_RNG_UNIT(x) (x & (A4L_RNG_VOLT_UNIT |	\
			      A4L_RNG_MAMP_UNIT |	\
			      A4L_RNG_NO_UNIT |		\
			      A4L_RNG_EXT_UNIT))

/* --- Subdevice flags desc stuff --- */

/* TODO: replace ANALOGY_SUBD_AI with ANALOGY_SUBD_ANALOG
   and ANALOGY_SUBD_INPUT */

/* Subdevice types masks */
#define A4L_SUBD_MASK_READ 0x80000000
#define A4L_SUBD_MASK_WRITE 0x40000000
#define A4L_SUBD_MASK_SPECIAL 0x20000000

/*!
 * @addtogroup analogy_subdevice
 * @{
 */

/*!
 * @anchor ANALOGY_SUBD_xxx @name Subdevices types
 * @brief Flags to define the subdevice type
 * @{
 */

/**
 * Unused subdevice
 */
#define A4L_SUBD_UNUSED (A4L_SUBD_MASK_SPECIAL|0x1)
/**
 * Analog input subdevice
 */
#define A4L_SUBD_AI (A4L_SUBD_MASK_READ|0x2)
/**
 * Analog output subdevice
 */
#define A4L_SUBD_AO (A4L_SUBD_MASK_WRITE|0x4)
/**
 * Digital input subdevice
 */
#define A4L_SUBD_DI (A4L_SUBD_MASK_READ|0x8)
/**
 * Digital output subdevice
 */
#define A4L_SUBD_DO (A4L_SUBD_MASK_WRITE|0x10)
/**
 * Digital input/output subdevice
 */
#define A4L_SUBD_DIO (A4L_SUBD_MASK_SPECIAL|0x20)
/**
 * Counter subdevice
 */
#define A4L_SUBD_COUNTER (A4L_SUBD_MASK_SPECIAL|0x40)
/**
 * Timer subdevice
 */
#define A4L_SUBD_TIMER (A4L_SUBD_MASK_SPECIAL|0x80)
/**
 * Memory, EEPROM, DPRAM
 */
#define A4L_SUBD_MEMORY (A4L_SUBD_MASK_SPECIAL|0x100)
/**
 * Calibration subdevice  DACs
 */
#define A4L_SUBD_CALIB (A4L_SUBD_MASK_SPECIAL|0x200)
/**
 * Processor, DSP
 */
#define A4L_SUBD_PROC (A4L_SUBD_MASK_SPECIAL|0x400)
/**
 * Serial IO subdevice
 */
#define A4L_SUBD_SERIAL (A4L_SUBD_MASK_SPECIAL|0x800)
/**
 * Mask which gathers all the types
 */
#define A4L_SUBD_TYPES (A4L_SUBD_UNUSED |	 \
			   A4L_SUBD_AI |	 \
			   A4L_SUBD_AO |	 \
			   A4L_SUBD_DI |	 \
			   A4L_SUBD_DO |	 \
			   A4L_SUBD_DIO |	 \
			   A4L_SUBD_COUNTER | \
			   A4L_SUBD_TIMER |	 \
			   A4L_SUBD_MEMORY |	 \
			   A4L_SUBD_CALIB |	 \
			   A4L_SUBD_PROC |	 \
			   A4L_SUBD_SERIAL)

/*! @} ANALOGY_SUBD_xxx */

/*!
 * @anchor ANALOGY_SUBD_FT_xxx @name Subdevice features
 * @brief Flags to define the subdevice's capabilities
 * @{
 */

/* Subdevice capabilities */
/**
 * The subdevice can handle command (i.e it can perform asynchronous
 * acquisition)
 */
#define A4L_SUBD_CMD 0x1000
/**
 * The subdevice support mmap operations (technically, any driver can
 * do it; however, the developer might want that his driver must be
 * accessed through read / write
 */
#define A4L_SUBD_MMAP 0x8000

/*! @} ANALOGY_SUBD_FT_xxx */

/*!
 * @anchor ANALOGY_SUBD_ST_xxx @name Subdevice status
 * @brief Flags to define the subdevice's status
 * @{
 */

/* Subdevice status flag(s) */
/**
 * The subdevice is busy, a synchronous or an asynchronous acquisition
 * is occuring
 */
#define A4L_SUBD_BUSY_NR 0
#define A4L_SUBD_BUSY (1 << A4L_SUBD_BUSY_NR)

/**
 * The subdevice is about to be cleaned in the middle of the detach
 * procedure
 */
#define A4L_SUBD_CLEAN_NR 1
#define A4L_SUBD_CLEAN (1 << A4L_SUBD_CLEAN_NR)


/*! @} ANALOGY_SUBD_ST_xxx */

/* --- Subdevice related IOCTL arguments structures --- */

/* SUDBINFO IOCTL argument */
struct a4l_subd_info {
	unsigned long flags;
	unsigned long status;
	unsigned char nb_chan;
};
typedef struct a4l_subd_info a4l_sbinfo_t;

/* CHANINFO / NBCHANINFO IOCTL arguments */
struct a4l_chan_info {
	unsigned long chan_flags;
	unsigned char nb_rng;
	unsigned char nb_bits;
};
typedef struct a4l_chan_info a4l_chinfo_t;

struct a4l_chinfo_arg {
	unsigned int idx_subd;
	void *info;
};
typedef struct a4l_chinfo_arg a4l_chinfo_arg_t;

/* RNGINFO / NBRNGINFO IOCTL arguments */
struct a4l_rng_info {
	long min;
	long max;
	unsigned long flags;
};
typedef struct a4l_rng_info a4l_rnginfo_t;

struct a4l_rng_info_arg {
	unsigned int idx_subd;
	unsigned int idx_chan;
	void *info;
};
typedef struct a4l_rng_info_arg a4l_rnginfo_arg_t;

/*! @} */

#define A4L_INSN_MASK_READ 0x8000000
#define A4L_INSN_MASK_WRITE 0x4000000
#define A4L_INSN_MASK_SPECIAL 0x2000000

/*!
 * @addtogroup analogy_lib_sync1
 * @{
 */

/*!
 * @anchor ANALOGY_INSN_xxx @name Instruction type
 * @brief Flags to define the type of instruction
 * @{
 */

/**
 * Read instruction
 */
#define A4L_INSN_READ (0 | A4L_INSN_MASK_READ)
/**
 * Write instruction
 */
#define A4L_INSN_WRITE (1 | A4L_INSN_MASK_WRITE)
/**
 * "Bits" instruction
 */
#define A4L_INSN_BITS (2 | A4L_INSN_MASK_READ | \
		       A4L_INSN_MASK_WRITE)
/**
 * Configuration instruction
 */
#define A4L_INSN_CONFIG (3 | A4L_INSN_MASK_READ | \
			 A4L_INSN_MASK_WRITE)
/**
 * Get time instruction
 */
#define A4L_INSN_GTOD (4 | A4L_INSN_MASK_READ | \
		       A4L_INSN_MASK_SPECIAL)
/**
 * Wait instruction
 */
#define A4L_INSN_WAIT (5 | A4L_INSN_MASK_WRITE | \
		       A4L_INSN_MASK_SPECIAL)
/**
 * Trigger instruction (to start asynchronous acquisition)
 */
#define A4L_INSN_INTTRIG (6 | A4L_INSN_MASK_WRITE | \
			  A4L_INSN_MASK_SPECIAL)

	  /*! @} ANALOGY_INSN_xxx */

/**
 * Maximal wait duration
 */
#define A4L_INSN_WAIT_MAX 100000

/*!
 * @anchor INSN_CONFIG_xxx @name Configuration instruction type
 * @brief Values to define the type of configuration instruction
 * @{
 */

#define A4L_INSN_CONFIG_DIO_INPUT		0
#define A4L_INSN_CONFIG_DIO_OUTPUT		1
#define A4L_INSN_CONFIG_DIO_OPENDRAIN		2
#define A4L_INSN_CONFIG_ANALOG_TRIG		16
#define A4L_INSN_CONFIG_ALT_SOURCE		20
#define A4L_INSN_CONFIG_DIGITAL_TRIG		21
#define A4L_INSN_CONFIG_BLOCK_SIZE		22
#define A4L_INSN_CONFIG_TIMER_1			23
#define A4L_INSN_CONFIG_FILTER			24
#define A4L_INSN_CONFIG_CHANGE_NOTIFY		25
#define A4L_INSN_CONFIG_SERIAL_CLOCK		26
#define A4L_INSN_CONFIG_BIDIRECTIONAL_DATA	27
#define A4L_INSN_CONFIG_DIO_QUERY		28
#define A4L_INSN_CONFIG_PWM_OUTPUT		29
#define A4L_INSN_CONFIG_GET_PWM_OUTPUT		30
#define A4L_INSN_CONFIG_ARM			31
#define A4L_INSN_CONFIG_DISARM			32
#define A4L_INSN_CONFIG_GET_COUNTER_STATUS	33
#define A4L_INSN_CONFIG_RESET			34
#define A4L_INSN_CONFIG_GPCT_SINGLE_PULSE_GENERATOR	1001	/* Use CTR as single pulsegenerator */
#define A4L_INSN_CONFIG_GPCT_PULSE_TRAIN_GENERATOR	1002	/* Use CTR as pulsetraingenerator */
#define A4L_INSN_CONFIG_GPCT_QUADRATURE_ENCODER	1003	/* Use the counter as encoder */
#define A4L_INSN_CONFIG_SET_GATE_SRC		2001	/* Set gate source */
#define A4L_INSN_CONFIG_GET_GATE_SRC		2002	/* Get gate source */
#define A4L_INSN_CONFIG_SET_CLOCK_SRC		2003	/* Set master clock source */
#define A4L_INSN_CONFIG_GET_CLOCK_SRC		2004	/* Get master clock source */
#define A4L_INSN_CONFIG_SET_OTHER_SRC		2005	/* Set other source */
#define A4L_INSN_CONFIG_SET_COUNTER_MODE	4097
#define A4L_INSN_CONFIG_SET_ROUTING		4099
#define A4L_INSN_CONFIG_GET_ROUTING		4109

/*! @} INSN_CONFIG_xxx */

/*!
 * @anchor ANALOGY_COUNTER_xxx @name Counter status bits
 * @brief Status bits for INSN_CONFIG_GET_COUNTER_STATUS
 * @{
 */

#define A4L_COUNTER_ARMED		0x1
#define A4L_COUNTER_COUNTING		0x2
#define A4L_COUNTER_TERMINAL_COUNT	0x4

	  /*! @} ANALOGY_COUNTER_xxx */

/*!
 * @anchor ANALOGY_IO_DIRECTION @name IO direction
 * @brief Values to define the IO polarity
 * @{
 */

#define A4L_INPUT	0
#define A4L_OUTPUT	1
#define A4L_OPENDRAIN	2

	  /*! @} ANALOGY_IO_DIRECTION */


/*!
 * @anchor ANALOGY_EV_xxx @name Events types
 * @brief Values to define the Analogy events. They might used to send
 * some specific events through the instruction interface.
 * @{
 */

#define A4L_EV_START		0x00040000
#define A4L_EV_SCAN_BEGIN	0x00080000
#define A4L_EV_CONVERT		0x00100000
#define A4L_EV_SCAN_END		0x00200000
#define A4L_EV_STOP		0x00400000

/*! @} ANALOGY_EV_xxx */

/*!
 * @brief Structure describing the synchronous instruction
 * @see a4l_snd_insn()
 */

struct a4l_instruction {
	unsigned int type;
		       /**< Instruction type */
	unsigned int idx_subd;
			   /**< Subdevice to which the instruction will be applied. */
	unsigned int chan_desc;
			    /**< Channel descriptor */
	unsigned int data_size;
			    /**< Size of the intruction data */
	void *data;
		    /**< Instruction data */
};
typedef struct a4l_instruction a4l_insn_t;

/*!
 * @brief Structure describing the list of synchronous instructions
 * @see a4l_snd_insnlist()
 */

struct a4l_instruction_list {
	unsigned int count;
			/**< Instructions count */
	a4l_insn_t *insns;
			  /**< Tab containing the instructions pointers */
};
typedef struct a4l_instruction_list a4l_insnlst_t;

/*! @} analogy_lib_sync1 */

struct a4l_calibration_subdev {
	a4l_sbinfo_t *info;
	char *name;
	int slen;
	int idx;
};

struct a4l_calibration_subdev_data {
	int index;
	int channel;
	int range;
	int expansion;
	int nb_coeff;
	double *coeff;

};

struct a4l_calibration_data {
	char *driver_name;
	char *board_name;
	int nb_ai;
	struct a4l_calibration_subdev_data *ai;
	int nb_ao;
	struct a4l_calibration_subdev_data *ao;
};

struct a4l_polynomial {
	int expansion;
	int order;
	int nb_coeff;
	double *coeff;
};


#endif /* _RTDM_UAPI_ANALOGY_H */
