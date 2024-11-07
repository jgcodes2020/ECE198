/*
 * read_write.c
 *
 *  Created on: Nov 7, 2024
 *      Author: jgcodes
 */

#include "main.h"

#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#define UART_TIMEOUT 1000

extern UART_HandleTypeDef huart2;

__attribute__((weak)) int _read(int file, char *ptr, int len);
__attribute__((weak)) int _write(int file, char *ptr, int len);

static int uart_read(UART_HandleTypeDef* huart, char* ptr, int len);
static int uart_write(UART_HandleTypeDef* huart, char* ptr, int len);

// write syscall.
int _write(int file, char *ptr, int len) {
	switch (file) {
	case STDOUT_FILENO:
	case STDERR_FILENO:
		return uart_write(&huart2, ptr, len);
	default:
		errno = EBADF;
		return -1;
	}
}

// read syscall.
int _read(int file, char *ptr, int len) {
	switch (file) {
	case STDIN_FILENO:
		return uart_read(&huart2, ptr, len);
	default:
		errno = EBADF;
		return -1;
	}
}

// Simple implementation of the read() syscall over STM32 UART.
static int uart_read(UART_HandleTypeDef* huart, char* ptr, int len) {
	uint32_t len2 = (uint32_t) len;
	uint16_t true_len;
	if (len2 > UINT16_MAX) {
		true_len = UINT16_MAX;
	}
	else {
		true_len = len2;
	}

	HAL_StatusTypeDef status = HAL_UART_Receive(&huart, ptr, true_len, UART_TIMEOUT);
	switch (status) {
	case HAL_OK:
		return true_len;
	case HAL_ERROR:
		errno = EIO;
		return -1;
	case HAL_BUSY:
		errno = EAGAIN;
		return -1;
	case HAL_TIMEOUT:
		errno = ETIMEDOUT;
		return -1;
	}
	return -1;
}
// Simple implementation of the write() syscall over STM32 UART.
static int uart_write(UART_HandleTypeDef* huart, char* ptr, int len) {
	uint32_t len2 = (uint32_t) len;
	uint16_t true_len;
	if (len2 > UINT16_MAX) {
		true_len = UINT16_MAX;
	}
	else {
		true_len = len2;
	}

	HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t*) ptr, true_len, UART_TIMEOUT);
	switch (status) {
	case HAL_OK:
		return true_len;
	case HAL_ERROR:
		errno = EIO;
		return -1;
	case HAL_BUSY:
		errno = EAGAIN;
		return -1;
	case HAL_TIMEOUT:
		errno = ETIMEDOUT;
		return -1;
	}
	return -1;
}
