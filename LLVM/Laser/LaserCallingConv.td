//===-- LaserCallingConv.td - Calling Conventions Laser ----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the Sparc architectures.
//
//===----------------------------------------------------------------------===//

// The return convention:
// Return value will be stored in RETVAL
def RetCC_LASER : CallingConv<[
    CCIfType<[i16], CCAssignToReg<[RETVAL]>> ,
    CCIfType<[i16], CCAssignToStack<2, 2>>
]>;

// // Passing convention: 
// def CC_LASER : CallingConv<[
//     // Promote i8 arguments to i16
//     CCIfType<[i8], CCPromoteToType<i16>>,
//     // The first 2 argument passed in registers
//     CCIfType<[i16], CCAssignToReg<[R8, R9]>>,
//     // Fall-back, and use the stack    
//     CCIfType<[i16], CCAssignToStack<2, 2>>
// ]>;

// The callee saved register set
def CSR_016 : CalleeSavedRegs<(add R8, R9)>;

