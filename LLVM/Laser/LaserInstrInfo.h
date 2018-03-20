
//===-- LaserInstrInfo.h - Cpu0 Instruction Information ----------*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Cpu0 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_LASER_LASERINSTRINFO_H
#define LLVM_LIB_TARGET_LASER_LASERINSTRINFO_H

#include "LaserRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"


#define GET_INSTRINFO_HEADER
#include "LaserGenInstrInfo.inc"


namespace llvm {
  class LaserSubtarget;

  class LaserInstrInfo : public LaserGenInstrInfo {
    void anchor();

    void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
		     const DebugLoc &DL, unsigned DestReg, unsigned SrcReg,
		     bool KillSrc) const override;

  protected:
    const LaserSubtarget &Subtarget;
    const LaserRegisterInfo RI;

  public:
    explicit LaserInstrInfo(const LaserSubtarget &STI);

    static const LaserInstrInfo *create(LaserSubtarget &STI);

    const LaserRegisterInfo &getRegisterInfo() const;


     MachineInstr* emitFrameIndexDebugValue(MachineFunction &MF,
						   int FrameIx, uint64_t Offset,
						   const MDNode *MDPtr,
						   DebugLoc DL) const;

    /// Adjust SP by Amount bytes.
    void adjustStackPtr(unsigned SP, unsigned DesReg, int64_t Amount,
                              MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const;
    // void adjustStackPtr(unsigned SP, int64_t Amount, MachineBasicBlock &MBB,
    // 			MachineBasicBlock::iterator I) const;

   
    //    static const LaserInstrInfo *create(LaserSubtarget &STI);
    /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info. As
    /// such, whenever a client has an instance of instruction info, it should
    /// always be able to get register info as well (through this method).
    ///
    /// Return the number of bytes of code the specified instruction may be.
    unsigned GetInstSizeInBytes(const MachineInstr *MI) const;

    bool expandPostRAPseudo(MachineInstr &MI) const override;


    void storeRegToStackSlot(MachineBasicBlock &MBB,
			     MachineBasicBlock::iterator MBBI,
			     unsigned SrcReg, bool isKill, int FrameIndex,
			     const TargetRegisterClass *RC,
			     const TargetRegisterInfo *TRI) const override {
      storeRegToStack(MBB, MBBI, SrcReg, isKill, FrameIndex, RC, TRI, 0);
    }

    void loadRegFromStackSlot(MachineBasicBlock &MBB,
			      MachineBasicBlock::iterator MBBI,
			      unsigned DestReg, int FrameIndex,
			      const TargetRegisterClass *RC,
			      const TargetRegisterInfo *TRI) const override {
      loadRegFromStack(MBB, MBBI, DestReg, FrameIndex, RC, TRI, 0);
    }

    virtual void storeRegToStack(MachineBasicBlock &MBB,
				 MachineBasicBlock::iterator MI,
				 unsigned SrcReg, bool isKill, int FrameIndex,
				 const TargetRegisterClass *RC,
				 const TargetRegisterInfo *TRI,
				 int64_t Offset) const;

    virtual void loadRegFromStack(MachineBasicBlock &MBB,
				  MachineBasicBlock::iterator MI,
				  unsigned DestReg, int FrameIndex,
				  const TargetRegisterClass *RC,
				  const TargetRegisterInfo *TRI,
				  int64_t Offset) const;

    /// isLoadFromStackSlot - If the specified machine instruction is a direct
    /// load from a stack slot, return the virtual or physical register number of
    /// the destination along with the FrameIndex of the loaded stack slot.  If
    /// not, return 0.  This predicate must return 0 if the instruction has
    /// any side effects other than loading from the stack slot.
    unsigned isLoadFromStackSlot(const MachineInstr &MI,
    					 int &FrameIndex) const override;

    /// isStoreToStackSlot - If the specified machine instruction is a direct
    /// store to a stack slot, return the virtual or physical register number of
    /// the source reg along with the FrameIndex of the loaded stack slot.  If
    /// not, return 0.  This predicate must return 0 if the instruction has
    /// any side effects other than storing to the stack slot.
    unsigned isStoreToStackSlot(const MachineInstr &MI,
    					int &FrameIndex) const override;

    MachineMemOperand *GetMemOperand(MachineBasicBlock &MBB, int FI,
				     MachineMemOperand::Flags Flags) const;

  private:
    void expandRetFlag(MachineBasicBlock *MBB, MachineInstr &MI) const;
    void expandICmpFlag(MachineBasicBlock *MBB, MachineInstr &MI) const;
    void expandJumpSetgeFlag(MachineBasicBlock *MBB, MachineInstr &MI) const;
    void expandDivRem_FLAG(MachineBasicBlock *MBB, MachineInstr &MI) const;
    void expandCall_FLAG(MachineBasicBlock *MBB, MachineInstr &MI) const;

  };

}
#endif

