/*************************************************************************
Title:    userdefs.h - 
Author:   Nicholas Morrow <nmorrow@crystaldiagnostics.com>
File:     userdefs.h
Software: STM32Fxxx_HAL_Driver, CMSIS-CORE
Hardware: STM32Fxxx
License:  The MIT License (MIT)

DESCRIPTION:


USAGE:

NOTES:

TO-DO:


LICENSE:
    Copyright (C) 2019 Pathogen Systems, Inc. dba Crystal Diagnostics

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

*************************************************************************/
#ifndef USERDEFS_H_
#define USERDEFS_H_
/**********************    INCLUDE DIRECTIVES    ***********************/


/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define PORT_MALLOC(size) malloc(size)
#define PORT_FREE(size) free(size)
#define PORT_REALLOC(size) realloc(size)
/***********************    FUNCTION PROTOTYPES    ***********************/

#endif /* USERDEFS_H_ */
