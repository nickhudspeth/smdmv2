/**
 ************************************************************************
 * @file errorstates.c
 * @brief
 * @author Nicholas Morrow <nmorrow@crystaldiagnostics.com>
 * @bug
 * @copyright Copyright (C) 2019 Pathogen Systems, Inc. dba Crystal Diagnostics

 * @attention
 The MIT License (MIT)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to
 deal in the Software without restriction, including without limitation the
 rights to use, copy, modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/

#include "errorstates.h"
/**********************    GLOBAL VARIABLES    ***********************/

/*******************    FUNCTION IMPLEMENTATIONS    ********************/
void errorstate_getstring(uint8_t state, char* buffer) {
	switch (state) {
	case ERRSTATE_NONE:
		sprintf(buffer, "ERRSTATE_NONE");
		break;
	case ERRSTATE_PAUSED:
		sprintf(buffer, "ERRSTATE_PAUSED");
		break;
	case ERRSTATE_ESTOP:
		sprintf(buffer, "ERRSTATE_ESTOP");
		break;
	case ERRSTATE_OVERHEAT:
		sprintf(buffer, "ERRSTATE_OVERHEAT");
		break;
	case ERRSTATE_OVERCURRENT:
		sprintf(buffer, "ERRSTATE_OVERCURRENT");
		break;
	case ERRSTATE_UNINITIALIZED:
		sprintf(buffer, "ERRSTATE_UNINITIALIZED");
		break;
	case ERRSTATE_SELFCHECK_FAIL:
		sprintf(buffer, "ERRSTATE_SELFCHECK_FAIL");
		break;
	case ERRSTATE_INMOTION:
		sprintf(buffer, "ERRSTATE_INMOTION");
		break;
	case ERRSTATE_TARGET_REACHED:
		sprintf(buffer, "ERRSTATE_TARGET_REACHED");
		break;
	}
}
