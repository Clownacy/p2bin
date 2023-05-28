#ifndef LZSS_H
#define LZSS_H

#include <stdio.h>

void Encode(int (*read_callback)(void *user_data), const void *user_data, FILE *outfile);
void Decode(int (*read_callback)(void *user_data), const void *user_data, FILE *outfile);

#endif /* LZSS_H */
