//===-- LaserRegisterInfo.h - Laser Register Information Impl -----*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Laser implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_LASER_LASERREGISTERINFO_H
#define LLVM_LIB_TARGET_LASER_LASERREGISTERINFO_H

#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "LaserGenRegisterInfo.inc"

namespace llvm {
  class  LaserSubtarget;

struct LaserRegisterInfo : public LaserGenRegisterInfo {
protected:
  const LaserSubtarget &Subtarget;

public:
  LaserRegisterInfo(const LaserSubtarget &Subtarget);

  /// Code Generation virtual methods...
  const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;

  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
				       CallingConv::ID) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  bool requiresRegisterScavenging(const MachineFunction &MF) const override;

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override;

  bool trackLivenessAfterRegAlloc(const MachineFunction &MF) const override;

  const TargetRegisterClass *intRegClass(unsigned Size) const;

  /// Stack Frame Processing Methods
  void eliminateFrameIndex(MachineBasicBlock::iterator II,
			   int SPAdj, unsigned FIOperandNum,
			   RegScavenger *RS = nullptr) const override;

  /// Debug information queries.
  unsigned getFrameRegister(const MachineFunction &MF) const override;



  /// \brief Return GPR register class.
  //const TargetRegisterClass *intRegClass(unsigned Size) const;

};
} // end namespace llvm
#endif

