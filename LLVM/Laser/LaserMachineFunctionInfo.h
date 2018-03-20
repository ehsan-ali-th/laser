//===- LaserMachineFunctionInfo.h - Laser Machine Function Info -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares  Laser specific per-machine-function information.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_LASER_LASERMACHINEFUNCTIONINFO_H
#define LLVM_LIB_TARGET_LASER_LASERMACHINEFUNCTIONINFO_H

#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

  class LaserMachineFunctionInfo : public MachineFunctionInfo {
  public:
    explicit LaserMachineFunctionInfo(MachineFunction &MF)
      : MF(MF), 
	SRetReturnReg(0), 
	GlobalBaseReg(0),
	VarArgsFrameIndex(0), 
	CallsEhReturn(false), 
	CallsEhDwarf(false),
	InArgFIRange(std::make_pair(-1, 0)),
	OutArgFIRange(std::make_pair(-1, 0)), 
	GPFI(0), 
	DynAllocFI(0),
	EmitNOAT(false),
	MaxCallFrameSize(0)
    {}

    ~LaserMachineFunctionInfo();

    bool isInArgFI(int FI) const {
      return FI <= InArgFIRange.first && FI >= InArgFIRange.second;
    }
    void setLastInArgFI(int FI) { InArgFIRange.second = FI; }
    bool isOutArgFI(int FI) const {
      return FI <= OutArgFIRange.first && FI >= OutArgFIRange.second;
    }

    int getGPFI() const { return GPFI; }
    void setGPFI(int FI) { GPFI = FI; }
    bool isGPFI(int FI) const { return GPFI && GPFI == FI; }

    bool isDynAllocFI(int FI) const { return DynAllocFI && DynAllocFI == FI; }

    unsigned getSRetReturnReg() const { return SRetReturnReg; }
    void setSRetReturnReg(unsigned Reg) { SRetReturnReg = Reg; }

    bool globalBaseRegFixed() const;
    bool globalBaseRegSet() const;
    unsigned getGlobalBaseReg();

    int getVarArgsFrameIndex() const { return VarArgsFrameIndex; }
    void setVarArgsFrameIndex(int Index) { VarArgsFrameIndex = Index; }

    bool hasByvalArg() const { return HasByvalArg; }

    void setFormalArgInfo(unsigned Size, bool HasByval) {
      IncomingArgSize = Size;
      HasByvalArg = HasByval;
    }

    unsigned getIncomingArgSize() const { return IncomingArgSize; }

    bool callsEhReturn() const { return CallsEhReturn; }
    void setCallsEhReturn() { CallsEhReturn = true; }

    bool callsEhDwarf() const { return CallsEhDwarf; }
    void setCallsEhDwarf() { CallsEhDwarf = true; }

    void createEhDataRegsFI();
    int getEhDataRegFI(unsigned Reg) const { return EhDataRegFI[Reg]; }

    unsigned getMaxCallFrameSize() const { return MaxCallFrameSize; }
    void setMaxCallFrameSize(unsigned S) { MaxCallFrameSize = S; }

    bool getEmitNOAT() const { return EmitNOAT; }
    void setEmitNOAT() { EmitNOAT = true; }

    /// Create a MachinePointerInfo that has an ExternalSymbolPseudoSourceValue
    /// object representing a GOT entry for an external function.
    MachinePointerInfo callPtrInfo(const char *ES);

    /// Create a MachinePointerInfo that has a GlobalValuePseudoSourceValue object
    /// representing a GOT entry for a global function.
    MachinePointerInfo callPtrInfo(const GlobalValue *GV);

  private:
    virtual void anchor();

    MachineFunction& MF;
    
    /// SRetReturnReg - Some subtargets require that sret lowering includes
    /// returning the value of the returned struct in a register. This field
    /// holds the virtual register into which the sret argument is passed.
    unsigned SRetReturnReg;

    /// GlobalBaseReg - keeps track of the virtual register initialized for
    /// use as the global base register. This is used for PIC in some PIC
    /// relocation models.
    unsigned GlobalBaseReg;

    // Frame Index for start of varargs area.
    int VarArgsFrameIndex;

    /// True if function has a byval argument.
    bool HasByvalArg;

    /// Size of incoming argument area.
    unsigned IncomingArgSize;

    /// CallsEhReturn - Whether the function calls llvm.eh.return.
    bool CallsEhReturn;

    /// CallsEhDwarf - Whether the function calls llvm.eh.dwarf.
    bool CallsEhDwarf;

    /// Frame objects for spilling eh data registers.
    int EhDataRegFI[2];

    // Range of frame object indices.
    // InArgFIRange: Range of indices of all frame objects created during call to
    //               LowerFormalArguments.
    // OutArgFIRange: Range of indices of all frame objects created during call to
    //                LowerCall except for the frame object for restoring $gp.
    std::pair<int, int> InArgFIRange, OutArgFIRange;

    int GPFI; // Index of the frame object for restoring $gp

    mutable int DynAllocFI; // Frame index of dynamically allocated stack area.

    bool EmitNOAT;

    unsigned MaxCallFrameSize;

  };
}

#endif
