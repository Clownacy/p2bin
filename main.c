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
#include "clownlzss/kosinski.h"
#include "clownlzss/saxman.h"
#include "lz_comp2/LZSS.h"

typedef enum Compression
{
	COMPRESSION_UNCOMPRESSED,
	COMPRESSION_KOSINSKI,
	COMPRESSION_KOSINSKI_OPTIMISED,
	COMPRESSION_SAXMAN,
	COMPRESSION_SAXMAN_OPTIMISED
} Compression;

static const char *header_filename;
static FILE *input_file, *output_file;
static jmp_buf jump_buffer;
static unsigned char padding_buffer[0x1000];
static unsigned char z80_buffer[0x2000];
static unsigned int z80_data_size = 0;
static unsigned int z80_read_index = 0;
static unsigned long maximum_address = 0;
static cc_bool last_segment_was_compressable_z80_code = cc_false;
static Compression compression_format = COMPRESSION_UNCOMPRESSED;
static unsigned int padding_value = 0;

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
	(void)user_data;

	return z80_read_index == z80_data_size ? (unsigned int)-1 : z80_buffer[z80_read_index++];
}

static void AccurateKosinskiCompressCallback_WriteByte(void* const user_data, const unsigned int byte)
{
	(void)user_data;

	fputc(byte, output_file);
}

static void ClownLZSSCallback_Write(void* const user_data, const unsigned char byte)
{
	(void)user_data;

	fputc(byte, output_file);
}

static void ClownLZSSCallback_Seek(void* const user_data, const size_t position)
{
	(void)user_data;

	fseek(output_file, position, SEEK_SET);
}

static size_t ClownLZSSCallback_Tell(void* const user_data)
{
	(void)user_data;

	return ftell(output_file);
}

static const ClownLZSS_Callbacks clownlzss_callbacks = {NULL, ClownLZSSCallback_Write, ClownLZSSCallback_Seek, ClownLZSSCallback_Tell};

static int LZSS_ReadByte(void* const user_data)
{
	(void)user_data;

	return z80_read_index == z80_data_size ? EOF : z80_buffer[z80_read_index++];
}

static unsigned long EmitCompressedZ80Code(void)
{
	if (last_segment_was_compressable_z80_code)
	{
		const unsigned long start_address = ftell(output_file);
		unsigned long end_address;

		switch (compression_format)
		{
			case COMPRESSION_UNCOMPRESSED:
				fwrite(z80_buffer, z80_data_size, 1, output_file);
				break;

			case COMPRESSION_KOSINSKI:
			{
				static const KosinskiCompressCallbacks callbacks = {NULL, AccurateKosinskiCompressCallback_ReadByte, NULL, AccurateKosinskiCompressCallback_WriteByte};
				KosinskiCompress(&callbacks, cc_false);
				break;
			}

			case COMPRESSION_KOSINSKI_OPTIMISED:
				if (!ClownLZSS_KosinskiCompress(z80_buffer, z80_data_size, &clownlzss_callbacks))
				{
					fputs("Error: Failed to allocate memory for compressor.\n", stderr);
					longjmp(jump_buffer, 1);
				}

				break;

			case COMPRESSION_SAXMAN:
				Encode(LZSS_ReadByte, NULL, output_file);
				fputc('N', output_file); /* Sonic 2 has this strange termination byte. It's not actually needed for anything. */
				break;

			case COMPRESSION_SAXMAN_OPTIMISED:
				if (!ClownLZSS_SaxmanCompressWithoutHeader(z80_buffer, z80_data_size, &clownlzss_callbacks))
				{
					fputs("Error: Failed to allocate memory for compressor.\n", stderr);
					longjmp(jump_buffer, 1);
				}

				break;
		}

		end_address = ftell(output_file);

		if (end_address > maximum_address)
			maximum_address = end_address;

		last_segment_was_compressable_z80_code = cc_false;

		return end_address - start_address;
	}

	return 0;
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

		const unsigned long compressed_z80_code_size = EmitCompressedZ80Code();

		if (compressed_z80_code_size != 0)
		{
			/* If the segment after the compressed data overlaps it, then not enough space was allocated for it. */
			if (start_address < (unsigned long)ftell(output_file))
				fprintf(stderr, "Warning: Space reserved for the compressed Z80 data is too small - at least 0x%lX bytes are needed. Increase the value used by the 'org' after the Z80 data.\n", compressed_z80_code_size);

			if (header_filename != NULL)
			{
				/* Output the size of the compressed data to the header file for fixpointer to amend the ROM with. */
				FILE* const header_file = fopen(header_filename, "r+");

				if (header_file == NULL)
				{
					fputs("Error: Could not open header file for amending.\n", stderr);
					longjmp(jump_buffer, 1);
				}
				else
				{
					fprintf(header_file, "comp_z80_size 0x%lX ", compressed_z80_code_size);
					fclose(header_file);
				}
			}
		}

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
	memset(padding_buffer, padding_value, sizeof(padding_buffer));

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

					/* Legacy CODE segment. */
					ProcessSegment(record_header);

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

	if (argc <= 1)
	{
		/* Display usage prompt. */
		/* The message is split because ANSI C has a stupidly-low limit on the largest string literal. */
		fputs(
			"Usage: p2bin [options] [input filename] [output filename] [header filename]\n"
			"\n"
			"Options\n"
			"  -zk  - Compress sound driver in Kosinski format.\n"
			"  -zko - Compress sound driver in Kosinski format (optimised).\n"
			"  -zs  - Compress sound driver in Saxman format.\n"
			"  -zso - Compress sound driver in Saxman format (optimised).\n"
			"  -pXX - Set padding value to the specified two-digit hexadecimal number.\n"
			"\n"
		, stderr);
		fputs(
			"This tool converts a Macro Assembler AS '.p' code file to a ROM file.\n"
			"Consecutive Z80 segments starting at address 0 can be compressed in a specified\n"
			"format, and the size of this compressed data will be written to the header file.\n"
		, stderr);

		return EXIT_SUCCESS;
	}

	/* Skip filename. */
	--argc; ++argv;

	/* Process arguments. */
	for (; argc != 0; --argc, ++argv)
	{
		const char* const argument = *argv;

		if (argument[0] == '-')
		{
			switch (argument[1])
			{
				case 'z':
					/* Z80 compression format. */
					switch (argument[2])
					{
						case 'k':
							switch (argument[3])
							{
								case '\0':
									compression_format = COMPRESSION_KOSINSKI;
									continue;

								case 'o':
									compression_format = COMPRESSION_KOSINSKI_OPTIMISED;
									continue;
							}

							break;

						case 's':
							switch (argument[3])
							{
								case '\0':
									compression_format = COMPRESSION_SAXMAN;
									continue;

								case 'o':
									compression_format = COMPRESSION_SAXMAN_OPTIMISED;
									continue;
							}

							break;
					}

					break;

				case 'p':
					/* Padding value. */
					if (sscanf(argument, "-p%X", &padding_value) == 0)
						fputs("Error: Could not parse '-p' argument's padding value.\n", stderr);

					continue;
			}

			fprintf(stderr, "Error: Unrecognised option '%s'.\n", argument);
		}
		else if (input_filename == NULL)
		{
			input_filename = argument;
		}
		else if (output_filename == NULL)
		{
			output_filename = argument;
		}
		else if (header_filename == NULL)
		{
			header_filename = argument;
		}
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
