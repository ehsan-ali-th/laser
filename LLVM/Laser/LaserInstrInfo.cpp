//===-- LaserInstrInfo.cpp - Laser Instruction Information ------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Laser implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "LaserInstrInfo.h"
#include "Laser.h"
#include "LaserMachineFunctionInfo.h"
#include "LaserSubtarget.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"


using namespace llvm;

#define GET_INSTRINFO_CTOR_DTOR
#include "LaserGenInstrInfo.inc"

// Pin the vtable to this file.
void LaserInstrInfo::anchor() {}

//@LaserInstrInfo {
LaserInstrInfo::LaserInstrInfo(const LaserSubtarget &STI) :
  LaserGenInstrInfo(LASER::ADJCALLSTACKDOWN, LASER::ADJCALLSTACKUP),

  Subtarget(STI),
  RI(STI)
 {}

const LaserInstrInfo *LaserInstrInfo::create(LaserSubtarget &STI) {
  return new LaserInstrInfo(STI);
}

const LaserRegisterInfo &LaserInstrInfo::getRegisterInfo() const {
  return RI;
}

MachineInstr*
LaserInstrInfo::emitFrameIndexDebugValue(MachineFunction &MF, int FrameIx,
					uint64_t Offset, const MDNode *MDPtr,
					DebugLoc DL) const {
  MachineInstrBuilder MIB = BuildMI(MF, DL, get(LASER::DBG_VALUE))
    .addFrameIndex(FrameIx).addImm(0).addImm(Offset).addMetadata(MDPtr);
  return &*MIB;
}

/// Expand Pseudo instructions into real backend instructions
bool LaserInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock *MBB = MI.getParent();
  switch(MI.getOpcode()) {
  default:
    return false;
  case LASER::RET_FLAG:
    expandRetFlag(MBB, MI);
    break;
  case LASER::ICmp_FLAG:
    expandICmpFlag(MBB, MI);
    break;
  case LASER::JumpSetge_FLAG:
    expandJumpSetgeFlag(MBB, MI);
    break;
  case LASER::DivRemP_FLAG:
    expandDivRem_FLAG(MBB, MI);
    break;
  // case LASER::CALL_FLAG:
  //   expandCall_FLAG(MBB, MI);
  //   break;
  }

  MBB->erase(MI);
  return true;
}

void LaserInstrInfo::expandRetFlag(MachineBasicBlock *MBB,
				 MachineInstr &MI) const {
  
  BuildMI(*MBB, MI, MI.getDebugLoc(), get(LASER::RET)).addReg(LASER::RETVAL);
}

void LaserInstrInfo::expandICmpFlag(MachineBasicBlock *MBB,
				 MachineInstr &MI) const {
  
  const MachineOperand &Op0 = MI.getOperand(0);
  const MachineOperand &Op1 = MI.getOperand(1);

  BuildMI(*MBB, MI, MI.getDebugLoc(), get(LASER::CMP))
    .addReg(Op0.getReg())
    .addReg(Op1.getReg());
}

void LaserInstrInfo::expandJumpSetgeFlag(MachineBasicBlock *MBB,
				 MachineInstr &MI) const {
  
  const MachineOperand &Op0 = MI.getOperand(0);

  // If RS = RD : then zero flag then jump
  BuildMI(*MBB, MI, MI.getDebugLoc(), get(LASER::JZ)) 
    .addMBB(Op0.getMBB());
  // If RS > RD then no carry then Jump
  BuildMI(*MBB, MI, MI.getDebugLoc(), get(LASER::JNC)) 
    .addMBB(Op0.getMBB());
}

void LaserInstrInfo::expandDivRem_FLAG(MachineBasicBlock *MBB,
				 MachineInstr &MI) const {
  
  const MachineOperand &Op0 = MI.getOperand(0);
  const MachineOperand &Op1 = MI.getOperand(1);
  const MachineOperand &Op2 = MI.getOperand(2);

  BuildMI(*MBB, MI, MI.getDebugLoc(), get(LASER::DIV)) 
    .addMBB(Op0.getMBB())
    .addMBB(Op1.getMBB())
    .addMBB(Op2.getMBB());
}

void LaserInstrInfo::expandCall_FLAG(MachineBasicBlock *MBB,
				 MachineInstr &MI) const {
  
  const MachineOperand &Op0 = MI.getOperand(0);

  MachineRegisterInfo &RegInfo = MBB->getParent()->getRegInfo();
  const TargetRegisterClass *RC = &LASER::GNPRegsRegClass;
  unsigned DesReg = RegInfo.createVirtualRegister(RC);
  
  BuildMI(*MBB, MI, MI.getDebugLoc(), get(LASER::IMD),DesReg) 
    .addGlobalAddress(Op0.getGlobal());
  BuildMI(*MBB, MI, MI.getDebugLoc(), get(LASER::CALL)) 
    .addGlobalAddress(Op0.getGlobal());
}



void LaserInstrInfo::
storeRegToStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
		unsigned SrcReg, bool isKill, int FI,
		const TargetRegisterClass *RC, const TargetRegisterInfo *TRI,
		int64_t Offset) const {

  // unsigned IMD = LASER::IMD;
  unsigned Opc = LASER::ST;
  // unsigned ADD = LASER::ADD;
  // unsigned FrameReg = LASER::SP;

  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOStore);

  // BuildMI(MBB, I, DL, get(IMD), SrcReg).addImm(Offset);
  // BuildMI(MBB, I, DL, get(ADD), SrcReg)
  //   .addReg(FrameReg).addReg(SrcReg);

  BuildMI(MBB, I, DL, get(Opc))
    .addReg(SrcReg, getKillRegState(isKill))
    .addFrameIndex(FI)
    .addMemOperand(MMO)
    .addImm(Offset);
}

void LaserInstrInfo::
loadRegFromStack(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
		 unsigned DestReg, int FI, const TargetRegisterClass *RC,
		 const TargetRegisterInfo *TRI, int64_t Offset) const {

  // unsigned IMD = LASER::IMD;
  unsigned Opc = LASER::LD;

  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();
  MachineMemOperand *MMO = GetMemOperand(MBB, FI, MachineMemOperand::MOLoad);

  //  BuildMI(MBB, I, DL, get(IMD), DestReg).addImm(Offset);

  BuildMI(MBB, I, DL, get(Opc), DestReg).addFrameIndex(FI).addImm(Offset)
    .addMemOperand(MMO);
}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned
LaserInstrInfo::isLoadFromStackSlot(const MachineInstr &MI, int &FrameIndex) const{
  //  assert(0 && "Unimplemented");
  return 0;
}
  
/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned
LaserInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                   int &FrameIndex) const {
  //assert(0 && "Unimplemented");
  return 0;
}

/// Adjust SP by Amount bytes.
void LaserInstrInfo::adjustStackPtr(unsigned SP, unsigned DesReg, int64_t Amount,
				     MachineBasicBlock &MBB,
				     MachineBasicBlock::iterator I) const {

  DebugLoc DL = I != MBB.end() ? I->getDebugLoc() : DebugLoc();
  
    unsigned ADD = LASER::ADD;
    unsigned SUB = LASER::SUB;
    unsigned IMD = LASER::IMD;
    unsigned MOV = LASER::MOV;

  if (Amount > 0) {
    // Positive amount, use add instruction
    BuildMI(MBB, I, DL, get(IMD), DesReg).addImm(Amount);
    BuildMI(MBB, I, DL, get(ADD), DesReg).addReg(DesReg).addReg(SP);
    BuildMI(MBB, I, DL, get(MOV), SP).addReg(DesReg);
  }
  else {
    // Negative amount, use sub instrction
    BuildMI(MBB, I, DL, get(IMD), DesReg).addImm(-1 * Amount);
    BuildMI(MBB, I, DL, get(SUB), DesReg).addReg(DesReg).addReg(SP);
    BuildMI(MBB, I, DL, get(MOV), SP).addReg(DesReg);
  }
}

void LaserInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  const DebugLoc &DL, unsigned DestReg,
                                  unsigned SrcReg, bool KillSrc) const {
  unsigned Opc = LASER::MOV;

  // if (LASER::CPURegsRegClass.contains(DestReg)) { // Copy to CPU Reg.
  //   if (LASER::CPURegsRegClass.contains(SrcReg))
  //     Opc = LASER::MOV;
  //   // else if (SrcReg == LASER::HI)
  //   //   Opc = LASER::MFHI, SrcReg = 0;
  //   // else if (SrcReg == LASER::LO)
  //   //   Opc = LASER::MFLO, SrcReg = 0;
  // }

  // else if (LASER::CPURegsRegClass.contains(SrcReg)) { // Copy from CPU Reg.
  //   Opc = LASER::MOV;
  // }

  assert(Opc && "Cannot copy registers");

  // MachineInstrBuilder MIB = BuildMI(MBB, I, DL, get(Opc))
  //   .addReg(DestReg)
  //   .addReg(SrcReg);

  // if (DestReg)
  //   MIB.addReg(DestReg, RegState::Define);

  // if (ZeroReg)
  //   MIB.addReg(ZeroReg);

  // if (SrcReg)
  //   MIB.addReg(SrcReg, getKillRegState(KillSrc));
}

// void LaserInstrInfo::adjustStackPtr(unsigned SP, int64_t Amount,
//                                      MachineBasicBlock &MBB,
//                                      MachineBasicBlock::iterator I) const {
//   DebugLoc DL = I != MBB.end() ? I->getDebugLoc() : DebugLoc();
//   unsigned ADD = LASER::ADD;

//   if (isInt<16>(Amount)) {
//   if (Amount > 0) {
//     // Positive amount, use add instruction
//     unsigned ADD = LASER::ADD;
//     unsigned IMD = LASER::IMD;
//     BuildMI(MBB, I, DL, get(IMD), SP).addImm(Amount);
//     BuildMI(MBB, I, DL, get(ADD), SP).addReg(DesReg).addReg(SP);
//   }
//   else {
//     // Negative amount, use sub instrction
//     unsigned SUB = LASER::SUB;
//     unsigned IMD = LASER::IMD;
//     BuildMI(MBB, I, DL, get(IMD), SP).addImm(-1 * Amount);
//     BuildMI(MBB, I, DL, get(SUB), SP).addReg(DesReg).addReg(SP);
//   }




//   // if (isInt<16>(Amount)) {
//   //   // addiu sp, sp, amount
//   //  BuildMI(MBB, I, DL, get(ADD), SP).addReg(SP).addImm(Amount);
//   // }
//   // else { // Expand immediate that doesn't fit in 16-bit.
//   //   unsigned Reg = loadImmediate(Amount, MBB, I, DL, nullptr);
//   //   BuildMI(MBB, I, DL, get(ADDu), SP).addReg(SP).addReg(Reg, RegState::Kill);
//   // }
// }


MachineMemOperand *
LaserInstrInfo::GetMemOperand(MachineBasicBlock &MBB, int FI,
                             MachineMemOperand::Flags Flags) const {

  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned Align = MFI.getObjectAlignment(FI);

  return MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(MF, FI),
                                 Flags, MFI.getObjectSize(FI), Align);
}

