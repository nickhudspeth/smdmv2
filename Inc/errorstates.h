/*************************************************************************
 Title:    errorstates.h -
 Author:   Nicholas Morrow <nmorrow@crystaldiagnostics.com>
 File:     errorstates.h
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
#ifndef ERRORSTATES_H_
#define ERRORSTATES_H_
/**********************    INCLUDE DIRECTIVES    ***********************/

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
/** The device is functioning normally. */
#define ERRSTATE_NONE 0x09
/** The device is in a non-error paused motion state and will remain in this
 * state pending issuance of a RESUME command. Upon receipt of a RESUME command,
 * motion will continue from the current location to the destination. */
#define ERRSTATE_PAUSED 0x02
/** Motor drivers have been disabled and any current motion operations in
 * progress have been cancelled due to receipt of an EMERGENCY STOP command. The
 * device will remain in this state pending issuance of a RESUME command. Upon
 * receipt of a RESUME command, the device will be returned to normal
 * operational state. */
#define ERRSTATE_ESTOP 0x04
#define ERRSTATE_OVERHEAT 0x14
#define ERRSTATE_OVERCURRENT 0x11
#define ERRSTATE_UNINITIALIZED 0x0A
#define ERRSTATE_SELFCHECK_FAIL 0x1D

/***********************    FUNCTION PROTOTYPES    ***********************/

#endif /* ERRORSTATES_H_ */
