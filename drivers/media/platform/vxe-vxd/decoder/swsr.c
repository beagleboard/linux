// SPDX-License-Identifier: GPL-2.0
/*
 * Software Shift Register Access fucntions
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 * Re-written for upstreming
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "swsr.h"
#include "vdec_defs.h"

#define NBIT_8BYTE_MASK(n)    ((1ULL << (n)) - 1)

/* Input FIFO length (in bytes). */
#define SWSR_INPUT_FIFO_LENGTH      8

/* Output FIFO length (in bits). */
#define SWSR_OUTPUT_FIFO_LENGTH     64

#define SWSR_NALTYPE_LENGTH         8

#define SWSR_MAX_SYNTAX_LENGTH      32

#define SWSR_ASSERT(expected) ({WARN_ON(!(expected)); 0; })

struct swsr_buffer {
	void **lst_link;
	/* Pointer to bitstream data. */
	unsigned char *data;
	/* Number of bytes of bitstream */
	unsigned long long num_bytes;
	/* Index (in bytes) to next data within the buffer */
	unsigned long long byte_offset;
	/* Number of bytes read from input FIFO */
	unsigned long long num_bytes_read;
};

struct swsr_input {
	/* Bitstream data (byte-based and pre emu prev) - left aligned. */
	unsigned long long fifo;
	/* Number of *bytes* in Input FIFO */
	unsigned int num_bytes;
	struct swsr_config config;
	/* Emulation prevention mode used to process data in Input FIFO */
	enum swsr_emprevent emprevent;
	/* Number of bytes in emulation prevention sequence */
	unsigned int emprev_seq_len;
	/* Size of bitstream declared at initialisation */
	unsigned long long bitstream_size;
	/*
	 * Number of bytes required from input buffer before checking
	 * next emulation prevention sequence.
	 */
	unsigned int bytes_for_next_sequ;
	/* Byte count read from size delimiter */
	unsigned long long byte_count;
	unsigned long long bytes_read_since_delim;
	/* Cumulative offset (in bytes) into input buffer data */
	unsigned long long bitstream_offset;
	/* Bitstream delimiter found (see #SWSR_delim_type) */
	unsigned char delim_found;
	/*
	 * No More Valid Data before next delimiter.
	 * Set only for SWSR_EMPREVENT_00000300.
	 */
	unsigned char no_moredata;
	/* Pointer to current input buffer in the context of Input FIFO */
	struct swsr_buffer *buf;
	/* Start offset within buffer of current delimited unit */
	long delimited_unit_start_offset;
	/* Size of current delimited unit (if already calculated) */
	unsigned int delimited_unit_size;
	/* Current bit offset within the current delimited unit */
	unsigned int delimunit_bitofst;
};

struct swsr_output {
	/*
	 * Bitstream data (post emulation prevention removal
	 * delimiter checking) - left aligned.
	 */
	unsigned long long fifo;
	/* Number of *bits* in Output FIFO */
	unsigned int num_bits;
	unsigned long long totalbits_consumed;
};

struct swsr_buffer_ctx {
	/*
	 * Callback function to notify event and provide/request data.
	 * See #SWSR_eCbEvent for event types and description
	 * of CB argument usage.
	 */
	swsr_callback_fxn cb_fxn;
	/* Caller supplied pointer for callback */
	void *cb_param;
	/* List of buffers */
	struct lst_t free_buffer_list;
	/*
	 * List of buffers (#SWSR_sBufferCtx) whose data reside
	 * in the Input/Output FIFOs.
	 */
	struct lst_t used_buffer_list;
};

struct swsr_context {
	/* IMG_TRUE if the context is initialised */
	unsigned char initialised;
	/* A pointer to an exception handler */
	swsr_except_handler_fxn exception_handler_fxn;
	/* Caller supplied pointer */
	void *pexception_param;
	/* Last recorded exception */
	enum swsr_exception exception;
	/* Buffer context data */
	struct swsr_buffer_ctx buffer_ctx;
	/* Context of shift register input. */
	struct swsr_input input;
	/* Context of shift register output */
	struct swsr_output output;
};

static unsigned long long left_aligned_nbit_8byte_mask(unsigned int mask, unsigned int nbits)
{
	return (((unsigned long long)mask << (64 - nbits)) |
			(unsigned long long)NBIT_8BYTE_MASK(64 - nbits));
}

/*
 * buffer has been exhausted and there is still more bytes declared in bitstream
 */
static int swsr_extractbyte(struct swsr_context *ctx, unsigned char *byte_ext)
{
	struct swsr_input *input;
	struct swsr_buffer_ctx *buf_ctx;
	unsigned char byte = 0;
	unsigned long long cur_byte_offset;
	unsigned int result = 0;

	if (!ctx || !byte_ext)
		return IMG_ERROR_FATAL;

	input = &ctx->input;
	buf_ctx = &ctx->buffer_ctx;

	cur_byte_offset = input->bitstream_offset;

	if (input->buf && input->buf->byte_offset < input->buf->num_bytes) {
		input->bitstream_offset++;
		byte = input->buf->data[input->buf->byte_offset++];
	} else if (input->bitstream_offset < input->bitstream_size) {
		struct swsr_buffer *buffer;

		buffer = lst_removehead(&buf_ctx->free_buffer_list);
		if (!buffer)
			return IMG_ERROR_FATAL;

		buffer->num_bytes_read = 0;
		buffer->byte_offset = 0;

		buf_ctx->cb_fxn(SWSR_EVENT_INPUT_BUFFER_START,
			buf_ctx->cb_param, 0,
			&buffer->data, &buffer->num_bytes);
		SWSR_ASSERT(buffer->data && buffer->num_bytes > 0);

		if (buffer->data && buffer->num_bytes > 0) {
			input->buf = buffer;

			/* Add input buffer to output buffer list. */
			lst_add(&buf_ctx->used_buffer_list, input->buf);

			input->bitstream_offset++;
			byte = input->buf->data[input->buf->byte_offset++];
		}
	}

	{
		struct swsr_buffer *buffer = input->buf;

		if (!buffer)
			buffer = lst_first(&buf_ctx->used_buffer_list);

		if (!buffer || buffer->num_bytes_read > buffer->num_bytes) {
			input->delimited_unit_start_offset = -1;
			input->delimited_unit_size = 0;
		}
	}
	/* If the bitstream offset hasn't increased we failed to read a byte. */
	if (cur_byte_offset == input->bitstream_offset) {
		input->buf = NULL;
		result = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
	}

	*byte_ext = byte;

	return result;
}

static unsigned char swsr_checkfor_delimiter(struct swsr_context *ctx)
{
	struct swsr_input *input;
	unsigned char delim_found = 0;

	input = &ctx->input;

	/* Check for delimiter. */
	if (input->config.delim_type == SWSR_DELIM_SCP) {
		unsigned int shift = (SWSR_INPUT_FIFO_LENGTH * 8)
			- input->config.delim_length;
		unsigned long long sequ = input->fifo >> shift;

		/*
		 * Check if the SCP value is matched outside of
		 * emulation prevention data.
		 */
		if (sequ == input->config.scp_value && input->bytes_for_next_sequ == 0)
			delim_found = 1;

	} else if (input->config.delim_type == SWSR_DELIM_SIZE) {
		delim_found = (input->bytes_read_since_delim >= input->byte_count) ? 1 : 0;
	}

	return delim_found;
}

static int swsr_increment_cur_bufoffset(struct swsr_context *ctx)
{
	struct swsr_buffer_ctx *buf_ctx;
	struct swsr_buffer *cur_buf;

	buf_ctx = &ctx->buffer_ctx;

	/* Update the number of bytes read from input FIFO for current buffer */
	cur_buf = lst_first(&buf_ctx->used_buffer_list);
	if (cur_buf->num_bytes_read >= cur_buf->num_bytes) {
		/* Mark current bitstream buffer as fully consumed */
		cur_buf->num_bytes_read = cur_buf->num_bytes;

		/* Notify the application that the old buffer is exhausted. */
		buf_ctx->cb_fxn(SWSR_EVENT_OUTPUT_BUFFER_END,
			buf_ctx->cb_param, 0,
			NULL, NULL);

		/*
		 * Discard the buffer whose data was at the head of
		 * the input FIFO.
		 */
		cur_buf = lst_removehead(&buf_ctx->used_buffer_list);
		/* Add the buffer container to free list. */
		lst_add(&buf_ctx->free_buffer_list, cur_buf);

		/*
		 * Since the byte that we read was actually from the next
		 * buffer increment it's counter.
		 */
		cur_buf = lst_first(&buf_ctx->used_buffer_list);
		cur_buf->num_bytes_read++;
	} else {
		cur_buf->num_bytes_read++;
	}

	return 0;
}

static enum swsr_found swsr_readbyte_from_inputfifo(struct swsr_context *ctx,
						    unsigned char *byte)
{
	struct swsr_input *input;
	enum swsr_found found = SWSR_FOUND_NONE;
	unsigned int result = 0;

	input = &ctx->input;

	input->delim_found |= swsr_checkfor_delimiter(ctx);

	/*
	 * Refill the input FIFO before checking for emulation prevention etc.
	 * The only exception is when there are no more bytes left to extract
	 * from input buffer.
	 */
	while (input->num_bytes < SWSR_INPUT_FIFO_LENGTH && result == 0) {
		unsigned char byte;

		result = swsr_extractbyte(ctx, &byte);
		if (result == 0) {
			input->fifo |= ((unsigned long long)byte <<
				((SWSR_INPUT_FIFO_LENGTH - 1 - input->num_bytes) * 8));
			input->num_bytes += 1;
		}
	}

	if (input->num_bytes == 0) {
		found = SWSR_FOUND_EOD;
	} else if (!input->delim_found) {
		/*
		 * Check for emulation prevention when enabled and enough
		 * bytes are remaining in input FIFO.
		 */
		if (input->emprevent != SWSR_EMPREVENT_NONE &&
		    /*
		     * Ensure you have enough bytes to check for emulation
		     * prevention.
		     */
		    input->num_bytes >= input->emprev_seq_len &&
		    (input->config.delim_type != SWSR_DELIM_SIZE ||
		    /*
		     * Ensure that you don't remove emu bytes beyond current
		     * delimited unit.
		     */
		     ((input->bytes_read_since_delim + input->emprev_seq_len) <
		     input->byte_count)) && input->bytes_for_next_sequ == 0) {
			unsigned char emprev_removed = 0;
			unsigned int shift = (SWSR_INPUT_FIFO_LENGTH - input->emprev_seq_len) * 8;
			unsigned long long sequ = input->fifo >> shift;

			if (input->emprevent == SWSR_EMPREVENT_00000300) {
				if ((sequ & 0xffffff00) == 0x00000300) {
					if ((sequ & 0x000000ff) > 0x03)
						pr_err("Invalid start code emulation preventionbytes found\n");

					/*
					 * Instead of trying to remove the emulation prevention
					 * byte from the middle of the FIFO simply make it zero
					 * and drop the next byte from the FIFO which will
					 * also be zero.
					 */
					input->fifo &= left_aligned_nbit_8byte_mask
							(0xffff00ff,
							 input->emprev_seq_len * 8);
					input->fifo <<= 8;

					emprev_removed = 1;
				} else if ((sequ & 0xffffffff) == 0x00000000 ||
					(sequ & 0xffffffff) == 0x00000001) {
					input->no_moredata = 1;
				}
			} else if (input->emprevent == SWSR_EMPREVENT_ff00) {
				if (sequ == 0xff00) {
					/* Remove the zero byte. */
					input->fifo <<= 8;
					input->fifo |= (0xff00ULL << shift);
					emprev_removed = 1;
				}
			} else if (input->emprevent == SWSR_EMPREVENT_000002) {
				/*
				 * Remove the emulation prevention bytes
				 * if we find 22 consecutive 0 bits
				 * (from a byte-aligned position?!)
				 */
				if (sequ == 0x000002) {
					/*
					 * Appear to "remove" the 0x02 byte by clearing
					 * it and then dropping the top (zero) byte.
					 */
					input->fifo &= left_aligned_nbit_8byte_mask
							(0xffff00,
							 input->emprev_seq_len * 8);
					input->fifo <<= 8;
					emprev_removed = 1;
				}
			}

			if (emprev_removed) {
				input->num_bytes--;
				input->bytes_read_since_delim++;

				/* Increment the buffer offset for the
				 *  byte that has been removed.
				 */
				swsr_increment_cur_bufoffset(ctx);

				/*
				 * Signal that two more new bytes in the emulation
				 * prevention sequence are required before another match
				 * can be made.
				 */
				input->bytes_for_next_sequ = input->emprev_seq_len - 2;
			}
		}

		if (input->bytes_for_next_sequ > 0)
			input->bytes_for_next_sequ--;

		/* return the first bytes from read data */
		*byte = (unsigned char)(input->fifo >> ((SWSR_INPUT_FIFO_LENGTH - 1) * 8));
		input->fifo <<= 8;

		input->num_bytes--;
		input->bytes_read_since_delim++;

		/* Increment the buffer offset for byte that has been read. */
		swsr_increment_cur_bufoffset(ctx);

		found = SWSR_FOUND_DATA;
	} else {
		found = SWSR_FOUND_DELIM;
	}

	return found;
}

static enum swsr_found swsr_consumebyte_from_inputfifo
	(struct swsr_context *ctx, unsigned char *byte)
{
	enum swsr_found found;

	found = swsr_readbyte_from_inputfifo(ctx, byte);

	if (found == SWSR_FOUND_DATA) {
		/* Only whole bytes can be read from Input FIFO. */
		ctx->output.totalbits_consumed += 8;
		ctx->input.delimunit_bitofst += 8;
	}

	return found;
}

static int swsr_fill_outputfifo(struct swsr_context *ctx)
{
	unsigned char byte;
	enum swsr_found found = SWSR_FOUND_DATA;

	/* Fill output FIFO with whole bytes up to (but not over) max length */
	while (ctx->output.num_bits <= (SWSR_OUTPUT_FIFO_LENGTH - 8) && found == SWSR_FOUND_DATA) {
		found = swsr_readbyte_from_inputfifo(ctx, &byte);
		if (found == SWSR_FOUND_DATA) {
			ctx->output.fifo |= ((unsigned long long)byte <<
				(SWSR_OUTPUT_FIFO_LENGTH - 8 - ctx->output.num_bits));
			ctx->output.num_bits += 8;
		}
	}

	return 0;
}

static unsigned int swsr_getbits_from_outputfifo(struct swsr_context *ctx,
						 unsigned int numbits,
						 unsigned char bconsume)
{
	unsigned int bitsread;

	/*
	 * Fetch more bits from the input FIFO if the output FIFO
	 * doesn't have enough bits to satisfy the request on its own.
	 */
	if (numbits > ctx->output.num_bits)
		swsr_fill_outputfifo(ctx);

	/* Ensure that are now enough bits in the output FIFO. */
	if (numbits > ctx->output.num_bits) {
		/* Tried to access into an SCP or other delimiter. */
		if (ctx->input.delim_found) {
			ctx->exception = SWSR_EXCEPT_ACCESS_INTO_SCP;
		} else {
			/*
			 * Data has been exhausted if after extracting bits
			 * there are still not enough bits in the internal
			 * storage to fulfil the number requested.
			 */
			ctx->exception = SWSR_EXCEPT_ACCESS_BEYOND_EOD;
		}

		ctx->exception_handler_fxn(ctx->exception, ctx->pexception_param);

		/* Return zero if the bits couldn't be obtained */
		bitsread = 0;
	} else {
		unsigned int shift;

		/* Extract all the bits from the output FIFO */
		shift = (SWSR_OUTPUT_FIFO_LENGTH - numbits);
		bitsread = (unsigned int)(ctx->output.fifo >> shift);

		if (bconsume) {
			/* Update output FIFO. */
			ctx->output.fifo <<= numbits;
			ctx->output.num_bits -= numbits;
		}
	}

	if (bconsume && ctx->exception == SWSR_EXCEPT_NO_EXCEPTION) {
		ctx->output.totalbits_consumed += numbits;
		ctx->input.delimunit_bitofst += numbits;
	}

	/* Return the bits */
	return bitsread;
}

int swsr_read_signed_expgoulomb(void *ctx_hndl)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	unsigned int exp_goulomb;
	unsigned char unsign;

	/* Validate input arguments. */
	if (!ctx) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	/* Read unsigned value then convert to signed value */
	exp_goulomb = swsr_read_unsigned_expgoulomb(ctx);

	unsign = exp_goulomb & 1;
	exp_goulomb >>= 1;
	exp_goulomb = (unsign) ? exp_goulomb + 1 : -(int)exp_goulomb;

	if (ctx->exception != SWSR_EXCEPT_NO_EXCEPTION)
		ctx->exception_handler_fxn(ctx->exception, ctx->pexception_param);

	/* Return the signed value */
	return exp_goulomb;
}

static unsigned int swsr_readunsigned_expgoulomb(struct swsr_context *ctx)
{
	unsigned int numbits = 0;
	unsigned int bitpeeked;
	unsigned int bitread;
	unsigned int setbits;
	unsigned int expgoulomb;

	/* Loop until we have found a non-zero nibble or reached 31 0-bits */
	/* first read is 3 bits only to prevent an illegal 32-bit peek */
	numbits = 1;
	do {
		bitpeeked = swsr_peekbits(ctx, numbits);
		/* Check for non-zero nibble */
		if (bitpeeked != 0)
			break;

		numbits++;

	} while (numbits < 32);

	/* Correct the number of leading zero bits */
	numbits--;

	if (bitpeeked) {
		/* read leading zeros and 1-bit */
		bitread = swsr_read_bits(ctx, numbits + 1);
		if (bitread != 1)
			ctx->exception = SWSR_EXCEPT_EXPGOULOMB_ERROR;
	} else {
		/*
		 * read 31 zero bits - special case to deal with 31 or 32
		 * leading zeros
		 */
		bitread = swsr_read_bits(ctx, 31);
		if (bitread != 0)
			ctx->exception = SWSR_EXCEPT_EXPGOULOMB_ERROR;

		/*
		 * next 3 bits make either 31 0-bit code:'1xx',
		 * or 32 0-bit code:'010'
		 */
		/*
		 * only valid 32 0-bit code is:'0..010..0'
		 * and results in 0xffffffff
		 */
		bitpeeked = swsr_peekbits(ctx, 3);

		if (ctx->exception == SWSR_EXCEPT_NO_EXCEPTION) {
			if (0x4 & bitpeeked) {
				bitread = swsr_read_bits(ctx, 1);
				numbits = 31;
			} else {
				if (bitpeeked != 2)
					ctx->exception = SWSR_EXCEPT_EXPGOULOMB_ERROR;

				bitread = swsr_read_bits(ctx, 3);
				bitread = swsr_read_bits(ctx, 31);
				if (bitread != 0)
					ctx->exception = SWSR_EXCEPT_EXPGOULOMB_ERROR;

				return 0xffffffff;
			}
		} else {
			/* encountered an exception while reading code */
			/* just return a valid value */
			return 0;
		}
	}

	/* read data bits */
	bitread = 0;
	if (numbits)
		bitread = swsr_read_bits(ctx, numbits);

	/* convert exp-goulomb to value */
	setbits = (1 << numbits) - 1;
	expgoulomb = setbits + bitread;
	/* Return the value */
	return expgoulomb;
}

unsigned int swsr_read_unsigned_expgoulomb(void *ctx_hndl)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	unsigned int value;

	/* Validate input arguments. */
	if (!ctx) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	value = swsr_readunsigned_expgoulomb(ctx);

	if (ctx->exception != SWSR_EXCEPT_NO_EXCEPTION)
		ctx->exception_handler_fxn(ctx->exception, ctx->pexception_param);

	return value;
}

enum swsr_exception swsr_check_exception(void *ctx_hndl)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	enum swsr_exception exception;

	/* Validate input arguments. */
	if (!ctx) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return (enum swsr_exception)IMG_ERROR_INVALID_PARAMETERS;
	}

	exception = ctx->exception;

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return (enum swsr_exception)IMG_ERROR_NOT_INITIALISED;
	}

	ctx->exception = SWSR_EXCEPT_NO_EXCEPTION;
	return exception;
}

int swsr_check_more_rbsp_data(void *ctx_hndl, unsigned char *more_rbsp_data)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;

	int rembitsinbyte;
	unsigned char currentbyte;
	int numof_aligned_rembits;
	unsigned long long rest_alignedbytes;
	unsigned char moredata = 0;

	/* Validate input arguments. */
	if (!ctx) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	if (ctx->input.emprevent != SWSR_EMPREVENT_00000300) {
		pr_err("SWSR cannot determine More RBSP data for a stream without SWSR_EMPREVENT_00000300: %s\n",
		       __func__);
		return IMG_ERROR_OPERATION_PROHIBITED;
	}

	/*
	 * Always fill the output FIFO to ensure the no_moredata flag is set
	 * when there are enough remaining bytes
	 */

	swsr_fill_outputfifo(ctx);

	if (ctx->output.num_bits != 0) {
		/* Calculate the number of bits in the MS byte */
		rembitsinbyte = (ctx->output.num_bits & 0x7);
		if (rembitsinbyte == 0)
			rembitsinbyte = 8;

		numof_aligned_rembits = (ctx->output.num_bits - rembitsinbyte);

		/* Peek the value of last byte. */
		currentbyte = swsr_peekbits(ctx, rembitsinbyte);
		rest_alignedbytes = (ctx->output.fifo >>
			(64 - ctx->output.num_bits)) &
			((1ULL << numof_aligned_rembits) - 1);

		if ((currentbyte == (1 << (rembitsinbyte - 1))) &&
		    (numof_aligned_rembits == 0 || (rest_alignedbytes == 0 &&
		    ((((((unsigned int)numof_aligned_rembits >> 3)) <
		    ctx->input.emprev_seq_len) &&
		    ctx->input.num_bytes == 0) || ctx->input.no_moredata))))
			moredata = 0;
		else
			moredata = 1;
	}

	*more_rbsp_data = moredata;

	return 0;
}

unsigned int swsr_read_onebit(void *ctx_hndl)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	unsigned int bitread;

	/* Validate input arguments. */
	if (!ctx_hndl) {
		VDEC_ASSERT(0);
		return -EIO;
	}

	ctx = (struct swsr_context *)ctx_hndl;

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	/* Optimize with inline code (specific version of call below). */
	bitread = swsr_read_bits(ctx, 1);

	return bitread;
}

unsigned int swsr_read_bits(void *ctx_hndl, unsigned int no_bits)
{
	struct swsr_context *ctx;

	/* Validate input arguments. */
	if (!ctx_hndl) {
		VDEC_ASSERT(0);
		return -EIO;
	}

	ctx = (struct swsr_context *)ctx_hndl;

	/* Validate input arguments. */
	if (!ctx->initialised) {
		pr_err("%s: Invalid SWSR context\n", __func__);
		ctx->exception = SWSR_EXCEPT_INVALID_CONTEXT;
		ctx->exception_handler_fxn(ctx->exception, ctx->pexception_param);

		return 0;
	}

	if (no_bits > SWSR_MAX_SYNTAX_LENGTH) {
		pr_err("Maximum symbol length exceeded\n");
		ctx->exception = SWSR_EXCEPT_WRONG_CODEWORD_ERROR;
		ctx->exception_handler_fxn(ctx->exception, ctx->pexception_param);

		return 0;
	}

	return swsr_getbits_from_outputfifo(ctx, no_bits, 1);
}

int swsr_read_signedbits(void *ctx_hndl, unsigned int no_bits)
{
	struct swsr_context *ctx;
	int outbits = 0;

	/* Validate input arguments. */
	if (!ctx_hndl) {
		VDEC_ASSERT(0);
		return -EIO;
	}

	ctx = (struct swsr_context *)ctx_hndl;

	/* Check if the context has been initialized. */
	if (!ctx->initialised) {
		pr_err("%s: Invalid SWSR context\n", __func__);
		ctx->exception = SWSR_EXCEPT_INVALID_CONTEXT;
		ctx->exception_handler_fxn(ctx->exception, ctx->pexception_param);

		return 0;
	}

	if ((no_bits + 1) > SWSR_MAX_SYNTAX_LENGTH) {
		pr_err("Maximum symbol length exceeded\n");
		ctx->exception = SWSR_EXCEPT_WRONG_CODEWORD_ERROR;
		ctx->exception_handler_fxn(ctx->exception, ctx->pexception_param);

		return 0;
	}
	outbits = swsr_getbits_from_outputfifo(ctx, no_bits, 1);

	return (swsr_getbits_from_outputfifo(ctx, 1, 1)) ? -outbits : outbits;
}

unsigned int swsr_peekbits(void *ctx_hndl, unsigned int no_bits)
{
	struct swsr_context *ctx;

	/* validate input parameters */
	if (!ctx_hndl) {
		VDEC_ASSERT(0);
		return -EIO;
	}

	ctx = (struct swsr_context *)ctx_hndl;

	/* Validate input arguments. */
	if (!ctx->initialised) {
		pr_err("%s: Invalid SWSR context\n", __func__);
		ctx->exception = SWSR_EXCEPT_INVALID_CONTEXT;
		ctx->exception_handler_fxn(ctx->exception, ctx->pexception_param);

		return 0;
	}

	if (no_bits > SWSR_MAX_SYNTAX_LENGTH) {
		pr_err("Maximum symbol length exceeded\n");
		ctx->exception = SWSR_EXCEPT_WRONG_CODEWORD_ERROR;
		ctx->exception_handler_fxn(ctx->exception, ctx->pexception_param);

		return 0;
	}

	return swsr_getbits_from_outputfifo(ctx, no_bits, 0);
}

int swsr_byte_align(void *ctx_hndl)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	unsigned int numbits;

	/* Validate input arguments. */
	if (!ctx) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	numbits = (ctx->output.num_bits & 0x7);
	/* Read the required number of bits if not already byte-aligned. */
	if (numbits != 0)
		swsr_read_bits(ctx, numbits);

	SWSR_ASSERT((ctx->output.num_bits & 0x7) == 0);

	return 0;
}

int swsr_get_total_bitsconsumed(void *ctx_hndl, unsigned long long *total_bitsconsumed)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;

	/* Validate input arguments. */
	if (!ctx || !total_bitsconsumed) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	*total_bitsconsumed = ctx->output.totalbits_consumed;

	return 0;
}

int swsr_get_byte_offset_curbuf(void *ctx_hndl, unsigned long long *byte_offset)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	struct swsr_buffer *outbuf;

	/* Validate input arguments. */
	if (!ctx || !byte_offset) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	if (ctx->output.num_bits != 0) {
		pr_err("SWSR output FIFO not empty. First seek to next delimiter: %s\n",
		       __func__);
		return IMG_ERROR_OPERATION_PROHIBITED;
	}

	outbuf = lst_first(&ctx->buffer_ctx.used_buffer_list);
	if (outbuf)
		*byte_offset = outbuf->num_bytes_read;
	else
		return IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;

	return 0;
}

static int swsr_update_emprevent(enum swsr_emprevent emprevent,
				 struct swsr_context *ctx)
{
	struct swsr_input *input;

	input = &ctx->input;

	input->emprevent = emprevent;
	switch (input->emprevent) {
	case SWSR_EMPREVENT_00000300:
		input->emprev_seq_len = 4;
		break;

	case SWSR_EMPREVENT_ff00:
		input->emprev_seq_len = 2;
		break;

	case SWSR_EMPREVENT_000002:
		input->emprev_seq_len = 3;
		break;

	default:
		input->emprev_seq_len = 0;
		break;
	}

	return 0;
}

int swsr_consume_delim(void *ctx_hndl, enum swsr_emprevent emprevent,
		       unsigned int size_delim_length, unsigned long long *byte_count)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	struct swsr_input *input;
	unsigned long long delimiter = 0;

	/* Validate input arguments. */
	if (!ctx || emprevent >= SWSR_EMPREVENT_MAX ||
	    (ctx->input.config.delim_type == SWSR_DELIM_SIZE &&
	    size_delim_length > SWSR_MAX_DELIM_LENGTH)) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	if (ctx->input.config.delim_type == SWSR_DELIM_SIZE &&
	    size_delim_length == 0 && !byte_count) {
		pr_err("Byte count value must be provided when size delimiter is zero length: %s\n",
		       __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	input = &ctx->input;

	/*
	 * Ensure that the input is at a delimiter since emulation prevention
	 * removal will not have spanned into this next unit.
	 * This allows emulation prevention detection modes to be changed.
	 * Now check for delimiter.
	 */
	input->delim_found = swsr_checkfor_delimiter(ctx);

	if (!input->delim_found)
		return IMG_ERROR_UNEXPECTED_STATE;

	/* Output bitstream FIFOs should be empty. */
	/* NOTE: flush output queue using seek function. */
	SWSR_ASSERT(ctx->output.num_bits == 0);

	/* Only update the delimiter length for size delimiters. */
	if (input->config.delim_type == SWSR_DELIM_SIZE)
		input->config.delim_length = size_delim_length;

	/* Update the emulation prevention detection/removal scheme */
	swsr_update_emprevent(emprevent, ctx);

	/*
	 * Peek at the NAL type and return in callback only
	 * when delimiter is in bitstream.
	 */
	if (input->config.delim_length) {
		unsigned int shift;
		unsigned char naltype;

		/*
		 * Peek at the next 8-bits after the delimiter that
		 * resides in internal FIFO.
		 */
		shift = SWSR_OUTPUT_FIFO_LENGTH -
			(input->config.delim_length + SWSR_NALTYPE_LENGTH);
		naltype = (input->fifo >> shift) & NBIT_8BYTE_MASK(SWSR_NALTYPE_LENGTH);

		/*
		 * Notify caller of NAL type so that bitstream segmentation
		 * can take place before the delimiter is consumed
		 */
		ctx->buffer_ctx.cb_fxn(SWSR_EVENT_DELIMITER_NAL_TYPE, ctx->buffer_ctx.cb_param,
			naltype, NULL, NULL);
	}

	/*
	 * Clear the delimiter found flag and reset bytes read to allow
	 * reading of data from input FIFO.
	 */
	input->delim_found = 0;

	if (input->config.delim_length != 0) {
		unsigned long long scpvalue = input->config.scp_value;
		unsigned int i;
		unsigned char byte = 0;

		/*
		 * Ensure that delimiter is not detected while delimiter
		 * is read.
		 */
		if (input->config.delim_type == SWSR_DELIM_SIZE) {
			input->bytes_read_since_delim = 0;
			input->byte_count = (input->config.delim_length + 7) / 8;
		} else if (input->config.delim_type == SWSR_DELIM_SCP) {
			input->config.scp_value = 0xdeadbeefdeadbeefUL;
		}

		/*
		 * Fill output FIFO only with bytes at least partially
		 * used for delimiter.
		 */
		for (i = 0; i < ((input->config.delim_length + 7) / 8); i++) {
			swsr_readbyte_from_inputfifo(ctx, &byte);

			ctx->output.fifo |= ((unsigned long long)byte <<
				(SWSR_OUTPUT_FIFO_LENGTH - 8 - ctx->output.num_bits));
			ctx->output.num_bits += 8;
		}

		/*
		 * Read delimiter from output FIFO leaving any remaining
		 * non-byte-aligned bits behind.
		 */
		delimiter = swsr_getbits_from_outputfifo(ctx, input->config.delim_length, 1);

		/* Restore SCP value. */
		if (input->config.delim_type == SWSR_DELIM_SCP)
			input->config.scp_value = scpvalue;
	} else {
		/*
		 * For size delimited bitstreams without a delimiter use
		 * the byte count provided.
		 */
		SWSR_ASSERT(*byte_count > 0);
		delimiter = *byte_count;
		SWSR_ASSERT(input->config.delim_type == SWSR_DELIM_SIZE);
	}

	if (input->config.delim_type == SWSR_DELIM_SCP)
		SWSR_ASSERT((delimiter & NBIT_8BYTE_MASK(input->config.delim_length)) ==
			input->config.scp_value);
	else if (input->config.delim_type == SWSR_DELIM_SIZE) {
		input->byte_count = delimiter;

		/* Return byte count if argument provided. */
		if (byte_count)
			*byte_count = input->byte_count;
	}

	input->bytes_read_since_delim = 0;
	{
		struct swsr_buffer *buffer = input->buf;

		if (!buffer)
			buffer = lst_first(&ctx->buffer_ctx.used_buffer_list);
		if (buffer)
			input->delimited_unit_start_offset = (long)buffer->num_bytes_read;
		else
			input->delimited_unit_start_offset = 0;
	}
	input->delimited_unit_size = 0;
	input->delimunit_bitofst = 0;

	input->no_moredata = 0;

	return 0;
}

enum swsr_found swsr_seek_delim_or_eod(void *ctx_hndl)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	enum swsr_found found = SWSR_FOUND_DATA;
	unsigned char byte;

	/* Validate input arguments. */
	if (!ctx) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return (enum swsr_found)IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return (enum swsr_found)IMG_ERROR_NOT_INITIALISED;
	}

	/* Read the residual contents of the output FIFO */
	swsr_byte_align(ctx);
	while (ctx->output.num_bits > 0) {
		SWSR_ASSERT((ctx->output.num_bits & 0x7) == 0);
		swsr_read_bits(ctx, 8);
	}
	SWSR_ASSERT(ctx->output.num_bits == 0);
	if (ctx->input.config.delim_type == SWSR_DELIM_SCP) {
		struct swsr_input *input = &ctx->input;
		struct swsr_output *output = &ctx->output;

		while (found == SWSR_FOUND_DATA) {
			unsigned char *offset;
			unsigned int delimlength_inbytes;
			unsigned char *startoffset;
			unsigned long long mask;
			unsigned long long scp;
			unsigned char scpfirstbyte;

			/*
			 * ensure that all the data in the input FIFO comes
			 * from the current buffer
			 */
			if (input->buf && input->buf->byte_offset <= input->num_bytes) {
				found = swsr_consumebyte_from_inputfifo(ctx, &byte);
				continue;
			}

			/* consume remaining bytes from the FIFO */
			if (!input->buf) {
				found = swsr_consumebyte_from_inputfifo(ctx, &byte);
				continue;
			}

			delimlength_inbytes = (input->config.delim_length + 7) / 8;

			/*
			 * Make the mask and the scp value byte aligned to
			 * speed up things
			 */
			mask = ((1UL << input->config.delim_length) - 1) <<
				(8 * delimlength_inbytes - input->config.delim_length);
			scp = input->config.scp_value <<
				(8 * delimlength_inbytes - input->config.delim_length);
			scpfirstbyte = (scp >> 8 * (delimlength_inbytes - 1)) & 0xFF;

			/* rollback the input FIFO */
			input->buf->byte_offset -= input->num_bytes;
			input->buf->num_bytes_read -= input->num_bytes;
			input->bitstream_offset -= input->num_bytes;
			input->num_bytes = 0;
			input->fifo = 0;

			startoffset = input->buf->data + input->buf->byte_offset;

			while (found == SWSR_FOUND_DATA) {
				offset = memchr(input->buf->data + input->buf->byte_offset,
						scpfirstbyte,
						input->buf->num_bytes -
						(input->buf->byte_offset + delimlength_inbytes -
						1));

				if (offset) {
					unsigned int i;

					/*
					 * load bytes that might be SCP into
					 * the FIFO
					 */
					for (i = 0; i < delimlength_inbytes; i++) {
						input->fifo <<= 8;
						input->fifo |= offset[i];
					}

					input->buf->byte_offset = offset - input->buf->data;

					if ((input->fifo & mask) == scp) {
						unsigned long long bytesread = offset
							- startoffset;

						/*
						 * Scp found, fill the rest of
						 * the FIFO
						 */
					for (i = delimlength_inbytes;
						i < SWSR_INPUT_FIFO_LENGTH &&
						input->buf->byte_offset + i <
						input->buf->num_bytes;
						i++) {
						input->fifo <<= 8;
						input->fifo |= offset[i];
					}

						input->fifo <<= (SWSR_INPUT_FIFO_LENGTH - i) * 8;

						input->bytes_for_next_sequ = 0;
						input->num_bytes = i;

						input->buf->byte_offset += i;

						input->buf->num_bytes_read = offset -
							input->buf->data;
						input->bitstream_offset += bytesread + i;

						output->totalbits_consumed += bytesread * 8;

						input->delimunit_bitofst += bytesread * 8;

						output->num_bits = 0;
						output->fifo = 0;

						SWSR_ASSERT(swsr_checkfor_delimiter(ctx));

						found = SWSR_FOUND_DELIM;
					} else {
						input->buf->byte_offset++;
					}
				} else {
					/* End of the current buffer */
					unsigned int bytesread = input->buf->num_bytes -
						(startoffset - input->buf->data);
					unsigned int i;

					/* update offsets */
					input->bitstream_offset += bytesread;
					output->totalbits_consumed += bytesread * 8;
					input->delimunit_bitofst += bytesread * 8;

					input->buf->byte_offset = input->buf->num_bytes;
					input->buf->num_bytes_read = input->buf->num_bytes -
						(delimlength_inbytes - 1);

					/* load remaining bytes to FIFO */
					offset = input->buf->data +
						input->buf->num_bytes -
						(delimlength_inbytes - 1);
					for (i = 0; i < delimlength_inbytes - 1;
						i++) {
						input->fifo <<= 8;
						input->fifo |= offset[i];
					}

					input->fifo <<= (SWSR_INPUT_FIFO_LENGTH - i) * 8;

					input->bytes_for_next_sequ = 0;
					input->num_bytes = delimlength_inbytes - 1;

					output->num_bits = 0;
					output->fifo = 0;

					/*
					 * Consume a few bytes from the next
					 * byte to check if there is scp on
					 * buffers boundary
					 */
					for (i = 0;
						i < delimlength_inbytes && found == SWSR_FOUND_DATA;
						i++) {
						found = swsr_consumebyte_from_inputfifo(ctx, &byte);
						SWSR_ASSERT(found != SWSR_FOUND_NONE);
					}

					break;
				}
			}
		}
	} else {
		/*
		 * Extract data from input FIFO until data is not found either
		 * because we have run out or a SCP has been detected.
		 */
		while (found == SWSR_FOUND_DATA) {
			found = swsr_consumebyte_from_inputfifo(ctx, &byte);
			SWSR_ASSERT(found != SWSR_FOUND_NONE);
		}
	}

	/*
	 * When the end of data has been reached there should be no
	 * more data in the input FIFO.
	 */
	if (found == SWSR_FOUND_EOD)
		SWSR_ASSERT(ctx->input.num_bytes == 0);

	SWSR_ASSERT(found != SWSR_FOUND_DATA);
	return found;
}

enum swsr_found swsr_check_delim_or_eod(void *ctx_hndl)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	enum swsr_found found = SWSR_FOUND_DATA;

	/* Validate input arguments. */
	if (!ctx) {
		pr_err("Invalid arguments to function: %s\n", __func__);

		return (enum swsr_found)IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);

		return (enum swsr_found)IMG_ERROR_NOT_INITIALISED;
	}

	/*
	 * End of data when all FIFOs are empty and there is nothing left to
	 * read from the input buffers.
	 */
	if (ctx->output.num_bits == 0 && ctx->input.num_bytes == 0 &&
	    ctx->input.bitstream_offset >= ctx->input.bitstream_size)
		found = SWSR_FOUND_EOD;
	else if (ctx->output.num_bits == 0 && swsr_checkfor_delimiter(ctx)) {
		/*
		 * Output queue is empty and delimiter is at the head of
		 * input queue.
		 */
		found = SWSR_FOUND_DELIM;
	}

	return found;
}

int swsr_start_bitstream(void *ctx_hndl, const struct swsr_config *config,
			 unsigned long long bitstream_size, enum swsr_emprevent emprevent)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	struct swsr_buffer *buffer;
	unsigned int result;

	/* Validate input arguments. */
	if (!ctx || !config || config->delim_type >= SWSR_DELIM_MAX ||
	    config->delim_length > SWSR_MAX_DELIM_LENGTH ||
	    config->scp_value > NBIT_8BYTE_MASK(config->delim_length) ||
	    emprevent >= SWSR_EMPREVENT_MAX) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	/* Move all used buffers into free list */
	buffer = lst_removehead(&ctx->buffer_ctx.used_buffer_list);
	while (buffer) {
		lst_add(&ctx->buffer_ctx.free_buffer_list, buffer);
		buffer = lst_removehead(&ctx->buffer_ctx.used_buffer_list);
	}

	/* Clear all the shift-register state (except config) */
	memset(&ctx->input, 0, sizeof(ctx->input));
	memset(&ctx->output, 0, sizeof(ctx->output));

	/* Update input FIFO configuration */
	ctx->input.bitstream_size = bitstream_size;
	ctx->input.config = *config;
	result = swsr_update_emprevent(emprevent, ctx);
	SWSR_ASSERT(result == 0);

	/*
	 * Signal delimiter found to ensure that no data is read out of
	 * input FIFO
	 * while fetching the first bitstream data into input FIFO.
	 */
	ctx->input.delim_found = 1;
	result = swsr_fill_outputfifo(ctx);
	SWSR_ASSERT(result == 0);

	/* Now check for delimiter. */
	ctx->input.delim_found = swsr_checkfor_delimiter(ctx);

	return 0;
}

int swsr_deinitialise(void *ctx_hndl)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	struct swsr_buffer *buffer;

	/* Validate input arguments. */
	if (!ctx) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	/* Free all used buffer containers */
	buffer = lst_removehead(&ctx->buffer_ctx.used_buffer_list);
	while (buffer) {
		kfree(buffer);
		buffer = lst_removehead(&ctx->buffer_ctx.used_buffer_list);
	}

	/* Free all free buffer containers. */
	buffer = lst_removehead(&ctx->buffer_ctx.free_buffer_list);
	while (buffer) {
		kfree(buffer);
		buffer = lst_removehead(&ctx->buffer_ctx.free_buffer_list);
	}

	ctx->initialised = 0;
	kfree(ctx);

	return 0;
}

int swsr_initialise(swsr_except_handler_fxn exception_handler_fxn,
		    void *exception_cbparam, swsr_callback_fxn callback_fxn,
		    void *cb_param, void **ctx_hndl)
{
	struct swsr_context *ctx;
	struct swsr_buffer *buffer;
	unsigned int i;
	unsigned int result;

	/* Validate input arguments. */
	if (!exception_handler_fxn || !exception_cbparam || !callback_fxn ||
	    !cb_param || !ctx_hndl) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Allocate and initialise shift-register context */
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		VDEC_ASSERT(0);
		return -EINVAL;
	}

	/* Setup shift-register context */
	ctx->exception_handler_fxn = exception_handler_fxn;
	ctx->pexception_param = exception_cbparam;

	ctx->buffer_ctx.cb_fxn = callback_fxn;
	ctx->buffer_ctx.cb_param = cb_param;

	/*
	 * Allocate a new buffer container for each byte in internal storage.
	 * This is the theoretical maximum number of buffers in the SWSR at
	 * any one time.
	 */
	for (i = 0; i < SWSR_INPUT_FIFO_LENGTH + (SWSR_OUTPUT_FIFO_LENGTH / 8);
		i++) {
		/* Allocate a buffer container */
		buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
		SWSR_ASSERT(buffer);
		if (!buffer) {
			result = IMG_ERROR_OUT_OF_MEMORY;
			goto error;
		}

		/* Add container to free list */
		lst_add(&ctx->buffer_ctx.free_buffer_list, buffer);
	}

	SWSR_ASSERT(SWSR_MAX_SYNTAX_LENGTH <= (sizeof(unsigned int) * 8));

	ctx->initialised = 1;
	*ctx_hndl = ctx;

	return 0;
error:
	buffer = lst_removehead(&ctx->buffer_ctx.free_buffer_list);
	while (buffer) {
		kfree(buffer);
		buffer = lst_removehead(&ctx->buffer_ctx.free_buffer_list);
	}
	kfree(ctx);

	return result;
}

static unsigned char swsr_israwdata_extraction_supported(struct swsr_context *ctx)
{
	/*
	 * For now only h.264/HEVC like 0x000001 SCP delimited
	 * bistreams are supported.
	 */
	if (ctx->input.config.delim_type == SWSR_DELIM_SCP &&
	    ctx->input.config.delim_length == (3 * 8) &&
	    ctx->input.config.scp_value == 0x000001)
		return 1;

	return 0;
}

static int swsr_getcurrent_delimited_unitsize(struct swsr_context *ctx, unsigned int *size)
{
	struct swsr_buffer *buf;

	buf = ctx->input.buf;
	if (!buf)
		buf = lst_first(&ctx->buffer_ctx.used_buffer_list);

	if (buf && ctx->input.delimited_unit_start_offset >= 0 &&
	    ctx->input.delimited_unit_start_offset < buf->num_bytes) {
		unsigned long long bufptr =
			(unsigned long long)ctx->input.delimited_unit_start_offset;
		unsigned int zeros = 0;

		/* Scan the current buffer for the next SCP. */
		while (1) {
			/* Look for two consecutive 0 bytes. */
			while ((bufptr < buf->num_bytes) && (zeros < 2)) {
				if (buf->data[bufptr++] == 0)
					zeros++;
				else
					zeros = 0;
			}
			/*
			 * If we're not at the end of the buffer already and
			 * the next byte is 1, we've got it.
			 */
			/*
			 * If we're at the end of the buffer, just assume
			 * we've got it too
			 * as we do not support buffer spanning units.
			 */
			if (bufptr < buf->num_bytes && buf->data[bufptr] == 1) {
				break;
			} else if (bufptr == buf->num_bytes) {
				zeros = 0;
				break;
			}
			/*
			 * Finally just decrease the number of 0s found
			 * already and go on scanning.
			 */
			else
				zeros = 1;
		}
		/* Calculate the unit size. */
		ctx->input.delimited_unit_size = (unsigned int)(bufptr -
			(unsigned long long)ctx->input.delimited_unit_start_offset) - zeros;
		*size = ctx->input.delimited_unit_size;
	} else {
		return IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
	}

	return 0;
}

int swsr_get_current_delimited_unitsize(void *ctx_hndl, unsigned int *size)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;

	/* Validate input arguments. */
	if (!ctx || !size) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	if (!swsr_israwdata_extraction_supported(ctx))
		return IMG_ERROR_NOT_SUPPORTED;

	return swsr_getcurrent_delimited_unitsize(ctx, size);
}

int swsr_get_current_delimited_unit(void *ctx_hndl, unsigned char *data, unsigned int *size)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;
	struct swsr_buffer *buf;
	unsigned int copysize;

	/* Validate input arguments. */
	if (!ctx || !data || !size || *size == 0) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	if (!swsr_israwdata_extraction_supported(ctx))
		return IMG_ERROR_NOT_SUPPORTED;

	buf = ctx->input.buf;
	if (!buf)
		buf = lst_first(&ctx->buffer_ctx.used_buffer_list);

	if (buf && ctx->input.delimited_unit_start_offset >= 0) {
		if (ctx->input.delimited_unit_size == 0)
			swsr_getcurrent_delimited_unitsize(ctx, &copysize);

		if (ctx->input.delimited_unit_size < *size)
			*size = ctx->input.delimited_unit_size;

		memcpy(data, buf->data + ctx->input.delimited_unit_start_offset, *size);
	} else {
		return IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
	}

	return 0;
}

int swsr_get_current_delimited_unit_bit_offset(void *ctx_hndl, unsigned int *bit_offset)
{
	struct swsr_context *ctx = (struct swsr_context *)ctx_hndl;

	/* Validate input arguments. */
	if (!ctx || !bit_offset) {
		pr_err("Invalid arguments to function: %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!ctx->initialised) {
		pr_err("SWSR not yet initialised: %s\n", __func__);
		return IMG_ERROR_NOT_INITIALISED;
	}

	if (!swsr_israwdata_extraction_supported(ctx))
		return IMG_ERROR_NOT_SUPPORTED;

	if (ctx->input.delimited_unit_start_offset >= 0)
		*bit_offset = ctx->input.delimunit_bitofst;

	return 0;
}
