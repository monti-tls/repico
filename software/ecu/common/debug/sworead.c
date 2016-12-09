#!/bin/c

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv)
{
	int buf_size = 2;
	unsigned char buf[buf_size];

	while (fread(buf, 1, buf_size, stdin) == buf_size)
	{
		fprintf(stdout, "%c", buf[1]);
		fflush(stdout);
	}

	return 0;
}
