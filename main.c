/*
Copyright (c) 2023 Clownacy

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
PERFORMANCE OF THIS SOFTWARE.
*/

/* Documentation of AS's code file format can be found here:
   http://john.ccac.rwth-aachen.de:8000/as/as_EN.html#sect_5_1_ */

/* Terminology in this code reflects the above documentation. */

#include <setjmp.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "accurate-kosinski/lib/kosinski-compress.h"

static FILE *input_file, *output_file;
static jmp_buf jump_buffer;
static unsigned char padding_buffer[0x1000];
static unsigned char z80_buffer[0x2000];
static unsigned int z80_data_size = 0;
static unsigned long maximum_address = 0;
static cc_bool last_segment_was_compressable_z80_code = cc_false;

static unsigned int ReadByte(void)
{
	const int byte = fgetc(input_file);

	if (byte == EOF)
	{
		fputs("Error: File ended prematurely.\n", stderr);
		longjmp(jump_buffer, 1);
	}

	return (unsigned long)byte;
}

static void ReadBytes(unsigned char* const buffer, const unsigned int total_bytes)
{
	if (fread(buffer, total_bytes, 1, input_file) == 0)
	{
		fputs("Error: File ended prematurely.\n", stderr);
		longjmp(jump_buffer, 1);
	}
}

static unsigned long ReadInteger(const unsigned int total_bytes)
{
	unsigned long value;
	unsigned int i;

	value = 0;

	for (i = 0; i < total_bytes; ++i)
		value |= (unsigned long)ReadByte() << (i * 8);

	return value;
}

static unsigned int ReadWord(void)
{
	return ReadInteger(2);
}

static unsigned long ReadLongInt(void)
{
	return ReadInteger(4);
}

static unsigned int AccurateKosinskiCompressCallback_ReadByte(void* const user_data)
{
	static unsigned int z80_read_index = 0;
	
	(void)user_data;

	return z80_read_index == z80_data_size ? (unsigned int)-1 : z80_buffer[z80_read_index++];
}

static void AccurateKosinskiCompressCallback_WriteByte(void* const user_data, const unsigned int byte)
{
	(void)user_data;

	fputc(byte, output_file);
}

static void EmitCompressedZ80Code(void)
{
	if (last_segment_was_compressable_z80_code)
	{
		static const KosinskiCompressCallbacks accurate_kosinski_compress_callbacks = {NULL, AccurateKosinskiCompressCallback_ReadByte, NULL, AccurateKosinskiCompressCallback_WriteByte};
		unsigned long end_address;

		KosinskiCompress(&accurate_kosinski_compress_callbacks, cc_false);

		end_address = ftell(output_file);

		if (end_address > maximum_address)
			maximum_address = end_address;

		last_segment_was_compressable_z80_code = cc_false;
	}
}

static void ProcessSegment(const unsigned int processor_family)
{
	const unsigned long start_address = ReadLongInt();
	const unsigned int length = ReadWord();
	const unsigned long end_address = start_address + length;

	/* Sound driver Z80 code must be compressed.
	   The telltale sign of compressable Z80 code is that its first segment has an address of 0. */
	if (processor_family == 0x51 && (start_address == 0 || last_segment_was_compressable_z80_code))
	{
		/* What we do is read as many consecutive Z80 segments as possible into a buffer and then
		   compressed and emit it when we encounter a non-Z80 segment or the end of the code file. */
		last_segment_was_compressable_z80_code = cc_true;

		if (start_address >= sizeof(z80_buffer))
		{
			fprintf(stderr, "Error: Compressed Z80 segment will not fit in Z80 RAM (Z80 RAM ends at 0x2000 but segment starts at 0x%lX).\n", start_address);
			longjmp(jump_buffer, 1);
		}
		else if (end_address >= sizeof(z80_buffer))
		{
			fprintf(stderr, "Error: Compressed Z80 segment will not fit in Z80 RAM (Z80 RAM ends at 0x2000 but segment ends at 0x%lX).\n", end_address);
			longjmp(jump_buffer, 1);
		}
		else
		{
			ReadBytes(&z80_buffer[start_address], length);

			if (end_address > z80_data_size)
				z80_data_size = end_address;
		}
	}
	else
	{
		static unsigned char copy_buffer[0x1000];

		unsigned long i;

		EmitCompressedZ80Code();

		if (start_address > maximum_address)
		{
			/* Set padding bytes between segments to 0xFF.
			   This is needed by Sonic 1, Knuckles in Sonic 2, Sonic 3, and Sonic & Knuckles. */
			/* TODO: Sonic 2. */
			const unsigned long padding_length = start_address - maximum_address;

			fseek(output_file, maximum_address, SEEK_SET);

			for (i = 0; i < padding_length; i += sizeof(padding_buffer))
				fwrite(padding_buffer, CC_MIN(sizeof(padding_buffer), padding_length - i), 1, output_file);
		}
		else
		{
			fseek(output_file, start_address, SEEK_SET);
		}

		/* Copy segment data. We do some batching using a buffer to improve performance. */
		for (i = 0; i < length; i += sizeof(copy_buffer))
		{
			const unsigned long bytes_to_do = CC_MIN(sizeof(copy_buffer), length - i);

			ReadBytes(copy_buffer, bytes_to_do);
			fwrite(copy_buffer, bytes_to_do, 1, output_file);
		}

		if (end_address > maximum_address)
			maximum_address = end_address;
	}
}

static cc_bool ProcessRecords(void)
{
	memset(padding_buffer, 0xFF, sizeof(padding_buffer));

	if (setjmp(jump_buffer) == 0)
	{
		for (;;)
		{
			const unsigned int record_header = ReadByte();
			unsigned int processor_family, granularity;

			switch (record_header)
			{
				case 0:
					/* Creator string. This marks the end of the file. */

					/* Emit the Z80 code here too, just in case it's the last segment in the file. */
					EmitCompressedZ80Code();

					return cc_true;

				case 0x80:
					/* Entry point. We don't care about this. */
					ReadLongInt();
					break;

				case 0x81:
					/* Arbitrary segment. */
					processor_family = ReadByte();
					ReadByte(); /* Segment. We don't care about this. */
					granularity = ReadByte();

					if (granularity != 1)
					{
						fprintf(stderr, "Error: Unsupported granularity of %u (only 1 is supported).\n", granularity);
						return cc_false;
					}

					ProcessSegment(processor_family);

					break;

				default:
					if (record_header >= 0x80)
					{
						fprintf(stderr, "Error: Unrecognised record header value (0x%02X).\n", record_header);
						return cc_false;
					}
					else
					{
						/* Legacy CODE segment. */
						ProcessSegment(record_header);
					}

					break;
			}
		}
	}

	return cc_false;
}

int main(int argc, char **argv)
{
	int exit_code = EXIT_FAILURE;
	const char *input_filename = NULL, *output_filename = NULL;

	/* Skip filename. */
	--argc; ++argv;

	/* Process arguments. */
	for (; argc != 0; --argc, ++argv)
	{
		const char* const argument = *argv;

		if (argument[0] == '-')
			; /* TODO */
		else if (input_filename == NULL)
			input_filename = argument;
		else if (output_filename == NULL)
			output_filename = argument;
	}

	/* Open input and output files. */
	input_file = fopen(input_filename, "rb");

	if (input_file == NULL)
	{
		fprintf(stderr, "Error: Could not open input file '%s' for reading.\n", input_filename);
	}
	else
	{
		output_file = fopen(output_filename, "wb");

		if (output_file == NULL)
		{
			fprintf(stderr, "Error: Could not open output file '%s' for writing.\n", output_filename);
		}
		else
		{
			/* Read and check the header's magic number. */
			unsigned char magic[2];

			if (fread(magic, sizeof(magic), 1, input_file) == 0)
				fputs("Error: Could not read header magic value.\n", stderr);
			else if (magic[0] != 0x89 || magic[1] != 0x14)
				fprintf(stderr, "Error: Invalid header magic value - expected 0x8914 but got 0x%02X%02X.\nInput file is either corrupt or not a valid AS code file.\n", magic[0], magic[1]);
			else if (ProcessRecords())
				exit_code = EXIT_SUCCESS;

			fclose(output_file);
		}

		fclose(input_file);
	}

	return exit_code;
}
