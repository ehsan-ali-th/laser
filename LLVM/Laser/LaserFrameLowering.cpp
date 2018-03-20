//===-- LaserFrameLowering.cpp - Laser Frame Information ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Laser implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "LaserFrameLowering.h"
#include "LaserInstrInfo.h"
#include "LaserMachineFunctionInfo.h"
#include "LaserSubtarget.h"
#include "LaserTargetMachine.h"

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetOptions.h"


using namespace llvm;

/// Mark \p Reg and all registers aliasing it in the bitset.
static void setAliasRegs(MachineFunction &MF, BitVector &SavedRegs, unsigned Reg) {
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  for (MCRegAliasIterator AI(Reg, TRI, true); AI.isValid(); ++AI)
    SavedRegs.set(*AI);
}


const LaserFrameLowering *LaserFrameLowering::create (const LaserSubtarget &ST) {
  return llvm::createLaserFrameLowering (ST);
}

// hasFP - Return true if the specified function should have a dedicated frame
// pointer register. This is true if the function has variable sized allocas,
// if it needs dynamic stack realignment, if frame pointer elimination is
// disabled, or if the frame address is taken.
bool LaserFrameLowering::hasFP (const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo *TRI = STI.getRegisterInfo();
  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
    MFI.hasVarSizedObjects() || MFI.isFrameAddressTaken() ||
    TRI->needsStackRealignment(MF);
}

void LaserFrameLowering::emitPrologue(MachineFunction &MF,
				       MachineBasicBlock &MBB) const {

  assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");

  MachineFrameInfo &MFI = MF.getFrameInfo();
  // LaserMachineFunctionInfo *LaserFI = MF.getInfo<LaserMachineFunctionInfo>();
  const LaserInstrInfo &TII =
    *static_cast<const LaserInstrInfo*>(STI.getInstrInfo());
  // const LaserRegisterInfo &RegInfo =
  //   *static_cast<const LaserRegisterInfo *>(STI.getRegisterInfo());
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc dl = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  //  LaserABIInfo ABI = STI.getABI();
  unsigned SP = LASER::SP;
  // unsigned FP = LASER::FP;
  // unsigned MOV = LASER::MOV;
  
  MachineRegisterInfo &RegInfo2 = MBB.getParent()->getRegInfo();
  const TargetRegisterClass *RC = &LASER::RTRegsRegClass;
  unsigned DesReg = RegInfo2.createVirtualRegister(RC);
  
  // First, compute final stack size.
  uint64_t StackSize = MFI.getStackSize();

  // No need to allocate space on the stack.
  if (StackSize == 0 && !MFI.adjustsStack()) 
    return;

  // MachineModuleInfo &MMI = MF.getMMI();
  // const MCRegisterInfo *MRI = MMI.getContext().getRegisterInfo();

  // Adjust stack.
  TII.adjustStackPtr(SP, DesReg, -StackSize, MBB, MBBI);

}

void LaserFrameLowering::emitEpilogue(MachineFunction &MF,
				       MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  // LaserMachineFunctionInfo *LaserFI = MF.getInfo<LaserMachineFunctionInfo>();
  const LaserInstrInfo &TII =
    *static_cast<const LaserInstrInfo *>(STI.getInstrInfo());
  // const LaserRegisterInfo &RegInfo =
  //   *static_cast<const LaserRegisterInfo *>(STI.getRegisterInfo());
  DebugLoc dl = MBBI->getDebugLoc();
  //  LaserABIInfo ABI = STI.getABI();
  unsigned SP = LASER::SP;
  // unsigned FP = LASER::FP;
  // unsigned MOV = LASER::MOV;res

  MachineRegisterInfo &RegInfo2 = MBB.getParent()->getRegInfo();
  const TargetRegisterClass *RC = &LASER::RTRegsRegClass;
  unsigned DesReg = RegInfo2.createVirtualRegister(RC);

 // Get the number of bytes from FrameInfo
  uint64_t StackSize = MFI.getStackSize();
  if (!StackSize)
    return;
  // Adjust stack.
  TII.adjustStackPtr(SP, DesReg, StackSize, MBB, MBBI);
}

const LaserFrameLowering *
llvm::createLaserFrameLowering(const LaserSubtarget &ST) {
  return new LaserFrameLowering(ST);
}

bool
LaserFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  // Reserve call frame if the size of the maximum call frame fits into 16-bit
  // immediate field and there are no variable sized objects on the stack.
  // Make sure the second register scavenger spill slot can be accessed with one
  // instruction.
  return isInt<16>(MFI.getMaxCallFrameSize() + getStackAlignment()) &&
    !MFI.hasVarSizedObjects();
}

// This method is called immediately before PrologEpilogInserter scans the
// physical registers used to determine what callee saved registers should be
// spilled. This method is optional.
void LaserFrameLowering::determineCalleeSaves(MachineFunction &MF,
					      BitVector &SavedRegs,
					      RegScavenger *RS) const {
  //@determineCalleeSaves-body
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  // LaserMachineFunctionInfo *LaserFI = MF.getInfo<LaserMachineFunctionInfo>();
  // MachineRegisterInfo& MRI = MF.getRegInfo();
  if (MF.getFrameInfo().hasCalls())
    setAliasRegs(MF, SavedRegs, LASER::LR);
  return;
}

// Eliminate ADJCALLSTACKDOWN, ADJCALLSTACKUP pseudo instructions
MachineBasicBlock::iterator LaserFrameLowering::
eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const {

  unsigned SP = LASER::SP;
  unsigned DesReg = LASER::R8;

  if (!hasReservedCallFrame(MF)) {
    int64_t Amount = I->getOperand(0).getImm();
    if (I->getOpcode() == LASER::ADJCALLSTACKDOWN)
      Amount = -Amount;

    STI.getInstrInfo()->adjustStackPtr(SP, DesReg, Amount, MBB, I);
  }

  return MBB.erase(I);
}

bool LaserFrameLowering::
spillCalleeSavedRegisters(MachineBasicBlock &MBB,
			  MachineBasicBlock::iterator MI,
			  const std::vector<CalleeSavedInfo> &CSI,
			  const TargetRegisterInfo *TRI) const {

  MachineFunction *MF = MBB.getParent();
  MachineBasicBlock *EntryBlock = &MF->front();
  const TargetInstrInfo &TII = *MF->getSubtarget().getInstrInfo();

  for (unsigned i = 0, e = CSI.size(); i != e; ++i) {
    // Add the callee-saved register as live-in. Do not add if the register is
    // LR and return address is taken, because it has already been added in
    // method LaserTargetLowering::LowerRETURNADDR.
    // It's killed at the spill, unless the register is LR and return address
    // is taken.
    unsigned Reg = CSI[i].getReg();
    bool IsRAAndRetAddrIsTaken = (Reg == LASER::LR)
        && MF->getFrameInfo().isReturnAddressTaken();
    if (!IsRAAndRetAddrIsTaken)
      EntryBlock->addLiveIn(Reg);

    // Insert the spill to the stack frame.
    bool IsKill = !IsRAAndRetAddrIsTaken;
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.storeRegToStackSlot(*EntryBlock, MI, Reg, IsKill,
                            CSI[i].getFrameIdx(), RC, TRI);
  }

  return true;

}


