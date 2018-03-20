//===-- LaserISelLowering.h - Laser DAG Lowering Interface --------*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that Laser uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_LASER_LASERISELLOWERING_H
#define LLVM_LIB_TARGET_LASER_LASERISELLOWERING_H

#include "Laser.h"
#include "MCTargetDesc/LaserBaseInfo.h"

#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/IR/Function.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLowering.h"
//#include "MCTargetDesc/LaserABIInfo.h"

#include <deque>

namespace llvm {
  namespace LASERISD {
    enum NodeType {
      // Start the numbering from where ISD NodeType finishes.
      FIRST_NUMBER = ISD::BUILTIN_OP_END,
      // call
      LaserCall,
      ABS_GV, // Get absolute global value
      // Get the Higher/Lower 16 bits from a 32-bit immediate.
      Hi,
      Lo,
      // Handle gp_rel (small data/bss sections) relocation.
      GPRel,
      // Load global symbol
      LOAD_SYM,
      // Thread Pointer
      ThreadPointer,
      // Return
      Ret,
      EH_RETURN,
      // DivRem(u)
      DivRem,
      DivRemU,
      Wrapper,
      // Integer Compare
      ICmp,
      Jump,
      JumpSetge,
      Sync
    };
  }

  class LaserMachineFunctionInfo;
  class LaserSubtarget;

  //@class LaserTargetLowering
  class LaserTargetLowering : public TargetLowering {

  public:
    explicit LaserTargetLowering (const LaserTargetMachine &TM, 
				  const LaserSubtarget &STI);

    static const LaserTargetLowering *create (const LaserTargetMachine &TM,					     
					     const LaserSubtarget &STI);

    SDValue LowerOperation (SDValue Op, SelectionDAG &DAG) const override;

    // This method returns the name of a target specific DAG node.
    const char *getTargetNodeName(unsigned Opcode) const override;

    SDValue PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI) const override;

  protected:
    // ByValArgInfo - Byval argument information.
    struct ByValArgInfo {
      unsigned FirstIdx; // Index of the first register used.
      unsigned NumRegs; // Number of registers used for this argument.
      unsigned Address; // Offset of the stack area used to pass this argument.
      ByValArgInfo() : FirstIdx(0), NumRegs(0), Address(0) {}
    };

    /// LaserCC - This class provides methods used to analyze formal and call
    /// arguments and inquire about calling convention information.
    class LaserCC {
    public:
      enum SpecialCallingConvType {
	NoSpecialCallingConv
      };

      LaserCC(CallingConv::ID CallConv, CCState &Info,
	      SpecialCallingConvType SpecialCallingConv = NoSpecialCallingConv);

      void analyzeCallOperands(const SmallVectorImpl<ISD::OutputArg> &Outs,
                               bool IsVarArg, bool IsSoftFloat,
                               const SDNode *CallNode,
                               std::vector<ArgListEntry> &FuncArgs);

      void analyzeCallResult(const SmallVectorImpl<ISD::InputArg> &Ins,
			     bool IsSoftFloat, const SDNode *CallNode,
			     const Type *RetTy) const;

      void analyzeReturn(const SmallVectorImpl<ISD::OutputArg> &Outs,
			 bool IsSoftFloat, const Type *RetTy) const;

      const CCState &getCCInfo() const { return CCInfo; }
      /// hasByValArg - Returns true if function has byval arguments.

      bool hasByValArg() const { return !ByValArgs.empty(); }
      /// reservedArgArea - The size of the area the caller reserves for
      /// register arguments. This is 16-byte if ABI is O32.

      unsigned reservedArgArea() const;

      typedef SmallVectorImpl<ByValArgInfo>::const_iterator byval_iterator;
      byval_iterator byval_begin() const { return ByValArgs.begin(); }
      byval_iterator byval_end() const { return ByValArgs.end(); }

      void analyzeFormalArguments(const SmallVectorImpl<ISD::InputArg> &Ins,
				  bool IsSoftFloat,
				  Function::const_arg_iterator FuncArg);

      /// regSize - Size (in number of bits) of integer registers.
      unsigned regSize() const { return IsO32 ? 2 : 2; }
      /// numIntArgRegs - Number of integer registers available for calls.
      unsigned numIntArgRegs() const;
      /// Return pointer to array of integer argument registers.
      const ArrayRef<MCPhysReg> intArgRegs() const;

      void handleByValArg(unsigned ValNo, MVT ValVT, MVT LocVT,
			  CCValAssign::LocInfo LocInfo,
			  ISD::ArgFlagsTy ArgFlags);

      /// useRegsForByval - Returns true if the calling convention allows the
      /// use of registers to pass byval arguments.
      bool useRegsForByval() const { return CallConv != CallingConv::Fast; }

      /// Return the function that analyzes fixed argument list functions.
      llvm::CCAssignFn *fixedArgFn() const;

      void allocateRegs(ByValArgInfo &ByVal, unsigned ByValSize,
			unsigned Align);

    private:

      /// Return the type of the register which is used to pass an argument or
      /// return a value. This function returns f64 if the argument is an i64
      /// value which has been generated as a result of softening an f128 value.
      /// Otherwise, it just returns VT.
      MVT getRegVT(MVT VT, const Type *OrigTy, const SDNode *CallNode,
		   bool IsSoftFloat) const;

      template<typename Ty>
      void analyzeReturn(const SmallVectorImpl<Ty> &RetVals, bool IsSoftFloat,
			 const SDNode *CallNode, const Type *RetTy) const;

      CCState &CCInfo;
      CallingConv::ID CallConv;
      bool IsO32;
      SmallVector<ByValArgInfo, 2> ByValArgs;
    };

    EVT getSetCCResultType(EVT VT) const;

    // Lower Operand helpers
    SDValue LowerCallResult(SDValue Chain, SDValue InFlag,
                            CallingConv::ID CallConv, bool isVarArg,
                            const SmallVectorImpl<ISD::InputArg> &Ins,
                            const SDLoc &dl, SelectionDAG &DAG,
                            SmallVectorImpl<SDValue> &InVals,
                            const SDNode *CallNode, const Type *RetTy) const;


  protected:
    const LaserSubtarget &Subtarget;

    // Cache the ABI from the TargetMachine, we use it everywhere.
    //    const LaserABIInfo &ABI;

  private:
   
    // Lower Operand specifics
    SDValue LowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const;

    //- must be exist even without function all
    SDValue LowerFormalArguments(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
				 const SmallVectorImpl<ISD::InputArg> &Ins,
				 const SDLoc &dl, SelectionDAG &DAG,
				 SmallVectorImpl<SDValue> &InVals) const override;
 
    SDValue LowerReturn(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
			const SmallVectorImpl<ISD::OutputArg> &Outs,
			const SmallVectorImpl<SDValue> &OutVals,
			const SDLoc &dl, SelectionDAG &DAG) const override;
    SDValue LowerBR_CC(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerBRCOND(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerBlockAddress(SDValue Op, SelectionDAG &DAG) const;
    SDValue LowerJumpTable(SDValue Op, SelectionDAG &DAG) const;



    // Create a TargetExternalSymbol node.
    SDValue getTargetNode(ExternalSymbolSDNode *N, EVT Ty, SelectionDAG &DAG,
                          unsigned Flag) const;

    // Create a Target GlobalAddress node.
    SDValue getTargetNode(GlobalAddressSDNode *N, EVT Ty,  SelectionDAG &DAG,
					      unsigned Flag) const; 

    // Create a TargetBlockAddress node.
    SDValue getTargetNode(BlockAddressSDNode *N, EVT Ty, SelectionDAG &DAG,
                          unsigned Flag) const;

    // Create a TargetJumpTable node.
    SDValue getTargetNode(JumpTableSDNode *N, EVT Ty, SelectionDAG &DAG,
                          unsigned Flag) const;

    SDValue getGlobalReg(SelectionDAG &DAG, EVT Ty) const;

    /// copyByValArg - Copy argument registers which were used to pass a byval
    /// argument to the stack. Create a stack frame object for the byval
    /// argument.
    void copyByValRegs(SDValue Chain, const SDLoc &DL,
                       std::vector<SDValue> &OutChains, SelectionDAG &DAG,
                       const ISD::ArgFlagsTy &Flags,
                       SmallVectorImpl<SDValue> &InVals,
                       const Argument *FuncArg,
                       const LaserCC &CC, const ByValArgInfo &ByVal) const;

    SDValue LowerCall(TargetLowering::CallLoweringInfo &CLI,
		      SmallVectorImpl<SDValue> &InVals) const override;

    /// passByValArg - Pass a byval argument in registers or on stack.
    void passByValArg(SDValue Chain, const SDLoc &DL,
                      std::deque< std::pair<unsigned, SDValue> > &RegsToPass,
                      SmallVectorImpl<SDValue> &MemOpChains, SDValue StackPtr,
                      MachineFrameInfo &MFI, SelectionDAG &DAG, SDValue Arg,
                      const LaserCC &CC, const ByValArgInfo &ByVal,
                      const ISD::ArgFlagsTy &Flags, bool isLittle) const;

    SDValue passArgOnStack(SDValue StackPtr, unsigned Offset, SDValue Chain,
                           SDValue Arg, const SDLoc &DL, bool IsTailCall,
                           SelectionDAG &DAG) const;

  protected:

  private:
    LaserCC::SpecialCallingConvType getSpecialCallingConv(SDValue Callee) const;

  };

  const LaserTargetLowering *
  createLaserTargetLowering(const LaserTargetMachine &TM, const LaserSubtarget &STI);
}
#endif // LaserISELLOWERING_H
