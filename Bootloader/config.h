/*
 * config.h
 *
 * Created: 19.02.2018 13:17:44
 *  Author: M43734
 */ 

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <stdio.h>

#ifdef DEBUG
#define dbgprintf printf
#else
#define dbgprintf(x, ...) {}
#endif

#define BRAM_ADDR 0x40074000U

#endif /* CONFIG_H_ */