//===-- LaserRegisterInfo.cpp - LASER Register Information -== --------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//

//===----------------------------------------------------------------------===//
//
// This file contains the LASER implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "laser-reg-info"

#include "LaserRegisterInfo.h"
#include "Laser.h"
#include "LaserMachineFunctionInfo.h"
#include "LaserSubtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/CodeGen/TargetInstrInfo.h"


#define GET_REGINFO_TARGET_DESC
#include "LaserGenRegisterInfo.inc"

using namespace llvm;

LaserRegisterInfo::LaserRegisterInfo(const LaserSubtarget &ST) : 
  LaserGenRegisterInfo(LASER::LR),
  Subtarget(ST)
{}

//===----------------------------------------------------------------------===//
// Callee Saved Registers methods
//===----------------------------------------------------------------------===//
/// LASER Callee Saved Registers
// In LaserCallConv.td,
// def CSR_016 : CalleeSavedRegs<(add LR, FP,
// (sequence "S%u", 2, 0))>;
// llc create CSR_O16_SaveList and CSR_O16_RegMask from above defined.
const MCPhysReg *
LaserRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  return CSR_016_SaveList;
}

const uint32_t *
LaserRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
					CallingConv::ID) const {
  return CSR_016_RegMask;
}


// pure virtual method
//@getReservedRegs {
BitVector LaserRegisterInfo::
getReservedRegs(const MachineFunction &MF) const {
  //@getReservedRegs body {
  static const uint16_t ReservedCPURegs[] = {
    LASER::FLAGR, LASER::SP, LASER::FP, LASER::SS,  LASER::LR, LASER::RETADDR, LASER::GP
  };
  BitVector Reserved(getNumRegs());
  for (unsigned I = 0; I < array_lengthof(ReservedCPURegs); ++I)

    Reserved.set(ReservedCPURegs[I]);
  return Reserved;
}

//@eliminateFrameIndex {
//- If no eliminateFrameIndex(), it will hang on run.

// pure virtual method
// FrameIndex represent objects inside a abstract stack.
// We must replace FrameIndex with an stack/frame pointer
// direct reference.
void LaserRegisterInfo::
eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj,
		    unsigned FIOperandNum, RegScavenger *RS) const {
  MachineInstr &MI = *II;
  DebugLoc DL = MI.getDebugLoc();
  MachineFunction &MF = *MI.getParent()->getParent();
  // MachineFrameInfo &MFI = MF.getFrameInfo();
  const LaserSubtarget &Subtarget = MF.getSubtarget<LaserSubtarget>();
  // LaserMachineFunctionInfo *LaserFI = MF.getInfo<LaserMachineFunctionInfo>();
  MachineBasicBlock &MBB = *MI.getParent();

  //  MachineRegisterInfo &RegInfo = MF.getRegInfo();
  MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
  const TargetRegisterClass *RC = &LASER::GNPRegsRegClass;
  unsigned Reg = RegInfo.createVirtualRegister(RC);



  unsigned i = 0;
  while (!MI.getOperand(i).isFI()) {
    ++i;
    assert(i < MI.getNumOperands() &&
	   "Instr doesn't have FrameIndex operand!");
  }

  DEBUG(errs() << "\nFunction : " << MF.getFunction().getName() << "\n";
	errs() << "<--------->\n" << MI);

  int FrameIndex = MI.getOperand(i).getIndex();
  uint64_t stackSize = MF.getFrameInfo().getStackSize();
  int64_t spOffset = MF.getFrameInfo().getObjectOffset(FrameIndex);

  DEBUG(errs() << "FrameIndex : " << FrameIndex << "\n"
	<< "spOffset : " << spOffset << "\n"
	<< "stackSize : " << stackSize << "\n");

  // const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  // int MinCSFI = 0;
  // int MaxCSFI = -1;

  // if (CSI.size()) {
  //   MinCSFI = CSI[0].getFrameIdx();
  //   MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  // }

  // The following stack frame objects are always referenced relative to $sp:
  // 1. Outgoing arguments.
  // 2. Pointer to dynamically allocated stack space.
  // 3. Locations for callee-saved registers.
  // Everything else is referenced relative to whatever register
  // getFrameRegister() returns.
  unsigned FrameReg;
  FrameReg = LASER::SP;

  // Calculate final offset.
  // - There is no need to change the offset if the frame object is one of the
  // following: an outgoing argument, pointer to a dynamically allocated
  // stack space or a $gp restore location,
  // - If the frame object is any of the following, its offset must be adjusted
  // by adding the size of the stack:
  // incoming argument, callee-saved register location or local variable.
  int64_t Offset;
  Offset = spOffset + (int64_t)stackSize;
  Offset += MI.getOperand(i+1).getImm();
  DEBUG(errs() << "Offset : " << Offset << "\n" << "<--------->\n");
  // If MI is not a debug value, make sure Offset fits in the 16-bit immediate
  // field.
  if (!MI.isDebugValue() && !isInt<16>(Offset)) {
    assert("(!MI.isDebugValue() && !isInt<16>(Offset))");
  }

  // assert(RS && "Register scavenging must be on");
  //  const TargetRegisterClass *RC = MRI.getRegClass(Reg);

  // TODO: Learn how to use RegScavenger to get unused  registers and also mark them
  // as used.

  // RS->enterBasicBlock(MBB);
  // unsigned RegUnused0 = RS->FindUnusedReg(&LASER::GNPRegsRegClass);
  // if (!RegUnused0)
  //   RegUnused0 = RS->scavengeRegister(&LASER::GNPRegsRegClass, II, SPAdj);
  // assert(RegUnused0 && "Register scavenger failed");
  // RS->setRegUsed(RegUnused0);

  const TargetInstrInfo &TII = *Subtarget.getInstrInfo();
  unsigned IMD = LASER::IMD;
  unsigned ADD = LASER::ADD;
  // unsigned LR = RegUnused0;

  BuildMI(*MI.getParent(), II, DL, TII.get(IMD), Reg).addImm(Offset);
  BuildMI(*MI.getParent(), II, DL, TII.get(ADD), Reg)
    .addReg(FrameReg).addReg(Reg);


  MI.getOperand(i).ChangeToRegister(Reg, false, false, true);
  MI.getOperand(i+1).ChangeToImmediate(Offset);


}
//}

bool
LaserRegisterInfo::requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool LaserRegisterInfo::
requiresFrameIndexScavenging(const MachineFunction &MF) const {
  return true;
}


bool
LaserRegisterInfo::trackLivenessAfterRegAlloc(const MachineFunction &MF) const {
  return true;
}

// pure virtual method
unsigned LaserRegisterInfo::
getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
  return TFI->hasFP(MF) ? (LASER::FP) :
    (LASER::SP);
}

// const TargetRegisterClass *
// LaserRegisterInfo::intRegClass(unsigned Size) const {
//   return &LASER::GNPRegsRegClass;
// }

