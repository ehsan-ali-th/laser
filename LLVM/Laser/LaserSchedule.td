//===-- LaserSchedule.td - Describe the Laser Itineries ----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

def IIAlu : InstrItinClass;
def IILoad : InstrItinClass;
def IIStore : InstrItinClass;
def IIBranch : InstrItinClass;
def IIImul : InstrItinClass;
def IIIdiv : InstrItinClass;

def IIPseudo : InstrItinClass;



def IIC_iu_or_fpu_instr : InstrItinClass;
def IIC_iu_instr : InstrItinClass;
def IIC_fpu_normal_instr : InstrItinClass;
def IIC_fpu_fast_instr : InstrItinClass;
def IIC_jmp_or_call : InstrItinClass;
def IIC_ldd : InstrItinClass;
def IIC_st : InstrItinClass;
def IIC_std : InstrItinClass;
def IIC_iu_smul : InstrItinClass;
def IIC_iu_umul : InstrItinClass;
def IIC_iu_div : InstrItinClass;
def IIC_ticc : InstrItinClass;
def IIC_ldstub : InstrItinClass;
def IIC_fpu_muls : InstrItinClass;
def IIC_fpu_muld : InstrItinClass;
def IIC_fpu_divs : InstrItinClass;
def IIC_fpu_divd : InstrItinClass;
def IIC_fpu_sqrts : InstrItinClass;
def IIC_fpu_sqrtd : InstrItinClass;
def IIC_fpu_abs : InstrItinClass;
def IIC_fpu_movs : InstrItinClass;
def IIC_fpu_negs : InstrItinClass;
def IIC_smac_umac : InstrItinClass;
def IIC_fpu_stod : InstrItinClass;


