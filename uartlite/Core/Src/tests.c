/*
 * tests.c
 *
 *  Created on: Feb 1, 2023
 *      Author: jdoorn
 */


#include <stdio.h>
#include <string.h>


int is_ok(char * buff, int count) {
	if (!strnstr(buff, "OK", count)) {
		return 0;
	}
	return 1;
}

char * is_incoming_data(char * buff, int count) {
	return strnstr(buff, "+IPD", count);
}

int main(){

	char * good_ok = "ashdlfkjasd hf OK jklsadflas \r\n";
	char * bad_ok = "asdhla a\r\n\r\n";
	if (is_ok(good_ok, strlen(good_ok))) {
		printf("Passed OK test with valid OK string\n");
	}
	if (!is_ok(bad_ok, strlen(bad_ok))) {
			printf("Passed OK test with bad OK string\n");
	}
//
//	printf("hello world");

}
