//===-- LaserFrameLowering.h - Define frame lowering for Laser --*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_LASER_LASERFRAMELOWERING_H
#define LLVM_LIB_TARGET_LASER_LASERFRAMELOWERING_H

#include "Laser.h"
#include "llvm/CodeGen/TargetFrameLowering.h"


namespace llvm {

class LaserSubtarget;

class LaserFrameLowering : public TargetFrameLowering {
protected:
  const LaserSubtarget &STI;

public:
  explicit LaserFrameLowering(const LaserSubtarget &ST)
    : TargetFrameLowering(StackGrowsDown,
                            /*StackAlignment=*/16,
                            /*LocalAreaOffset=*/0),
        STI(ST) {}

  static const LaserFrameLowering *create(const LaserSubtarget &ST);

  /// emitProlog/emitEpilog - These methods insert prolog and epilog code into
  /// the function.
  void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  bool hasFP(const MachineFunction & /*MF*/) const override;

  bool hasReservedCallFrame(const MachineFunction &MF) const override;

  void determineCalleeSaves(MachineFunction &MF, BitVector &SavedRegs,
                            RegScavenger *RS) const override;

  MachineBasicBlock::iterator
  eliminateCallFramePseudoInstr(MachineFunction &MF,
                                  MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I) const override;

  bool spillCalleeSavedRegisters(MachineBasicBlock &MBB,
				 MachineBasicBlock::iterator MI,
				 const std::vector<CalleeSavedInfo> &CSI,
				 const TargetRegisterInfo *TRI) const override;


};

/// Create LaserFrameLowering objects.
const LaserFrameLowering *createLaserFrameLowering(const LaserSubtarget &ST);


} // End llvm namespace

#endif
