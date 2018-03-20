//===-- LaserRegisterInfo.td - Laser Register defs ---------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the Laser register file
//===----------------------------------------------------------------------===//

// We have banks of 16 registers each.
class LaserReg<bits<16> Enc, string n> : Register<n> {
  let HWEncoding = Enc;
  let Namespace = "LASER";
}

// LASER CPU Registers
class LaserGPRReg<bits<16> Enc, string n> : LaserReg<Enc, n>;

def FLAGR:      LaserReg <0, "flagr">,    DwarfRegNum<[0]>;
def SP:         LaserReg <1, "sp">,       DwarfRegNum<[1]>;
def FP:         LaserReg <2, "fp">,       DwarfRegNum<[2]>;
def SS:         LaserReg <3, "ss">,       DwarfRegNum<[3]>;
def LR:         LaserReg <4, "lr">,       DwarfRegNum<[4]>;
def RETADDR:    LaserReg <5, "retaddr">,  DwarfRegNum<[5]>;
def GP:         LaserReg <6, "gp">,       DwarfRegNum<[6]>;
def RETVAL:     LaserReg <7, "retval">,   DwarfRegNum<[7]>;


foreach i = 8-15  in {
        def R#i : LaserGPRReg<i, "r"#i>, DwarfRegNum<[#i]>;
}

// @General Registers class
def GNPRegs : RegisterClass<"LASER", [i16], 16, 
    (add R8, R9, R10, R11, R12, R13, R14, R15, SP, RETVAL)>;

def CPURegs : RegisterClass<"LASER", [i16], 16, 
    (add FLAGR, SP, FP, SS, LR, RETADDR, GP)>;

def RTRegs : RegisterClass<"LASER", [i16], 16, 
    (add R8, R9, R10, R11, R12, R13, R14, R15)>;


// @Status Registers class
def SR : RegisterClass<"LASER", [i16], 16, (add FLAGR)>;

// @Control Flow Registers class
def CFR  : RegisterClass<"LASER", [i16], 16, (add RETADDR)>;

def RVR  : RegisterClass<"LASER", [i16], 16, (add RETVAL)>;




