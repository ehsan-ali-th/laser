//===-- LaserISelLowering.cpp - Laser DAG Lowering Implementation -----------===//
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

#include "MCTargetDesc/LaserBaseInfo.h"
#include "LaserRegisterInfo.h"
#include "LaserISelLowering.h"
#include "LaserMachineFunctionInfo.h"
#include "LaserTargetMachine.h"
#include "LaserTargetObjectFile.h"

#include "LaserSubtarget.h"

#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/Support/CommandLine.h"

#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "laser-lower"

LaserTargetLowering::LaserTargetLowering(const LaserTargetMachine &TM,
					 const LaserSubtarget &STI)
  : TargetLowering(TM), Subtarget(STI) {

  // Laser does not have i1 type, so use i16 for
  // setcc operations results (slt, sgt, ...).
  setBooleanContents(ZeroOrOneBooleanContent);
  setBooleanVectorContents(ZeroOrOneBooleanContent); // FIXME: Is this correct?

  //- Set .align 2
  // It will emit .align 2 later
  setMinFunctionAlignment(2);

  // Set up the register classes
  addRegisterClass(MVT::i16, &LASER::GNPRegsRegClass);
  addRegisterClass(MVT::i16, &LASER::RTRegsRegClass);

  // setOperationAction(ISD::SDIV, MVT::i16, Expand);
  // setOperationAction(ISD::SREM, MVT::i16, Expand);  

  // setTargetDAGCombine(ISD::SDIVREM);

  //setOperationAction(ISD::SHL, MVT::i16, Expand);

  // Operations not directly supported by Laser:
  // Custome: forward the node for Lowering to LowerOperation ()
  //setOperationAction(ISD::BR_CC, MVT::Other, Custom);
  setOperationAction(ISD::BR_CC, MVT::i16, Custom);

  setOperationAction(ISD::GlobalAddress, MVT::i16, Custom);
  setOperationAction(ISD::BlockAddress, MVT::i16, Custom);
  setOperationAction(ISD::JumpTable, MVT::i16, Custom);

  

  // Used by legalize types to correctly generate the setcc result.
  // Without this, every float setcc comes with a AND/OR with the result,
  // we don't want this, since the cmp result goes to a flag register,
  // which is used implicitly by brcond and select operations.
  AddPromotedToType(ISD::SETCC, MVT::i1, MVT::i16);


  // Unsupported condition codes
  // setCondCodeAction(ISD::SETFALSE, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETOEQ, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETOGT, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETOGE, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETOLT, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETOLE, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETONE, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETO, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETUO, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETUEQ, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETUGT, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETUGE, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETULT, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETULE, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETTRUE, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETFALSE2, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETGE, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETLE, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETTRUE2, MVT::i16, Expand);
  // setCondCodeAction(ISD::SETCC_INVALID, MVT::i16, Expand);

  // must, computeRegisterProperties - Once all of the register classes are
  // added, this allows us to compute derived properties we expose.
  computeRegisterProperties(STI.getRegisterInfo());
}

SDValue LaserTargetLowering::LowerOperation(SDValue Op,SelectionDAG &DAG) const {
  switch (Op.getOpcode())
    {
    case ISD::BR_CC:              return LowerBR_CC(Op, DAG);
    case ISD::BRCOND:             return LowerBRCOND(Op, DAG);
    case ISD::BlockAddress:       return LowerBlockAddress(Op, DAG);
    case ISD::JumpTable:          return LowerJumpTable(Op, DAG);
    case ISD::GlobalAddress:      return LowerGlobalAddress(Op, DAG);
    }
  return SDValue();
}

const LaserTargetLowering *LaserTargetLowering::create(const LaserTargetMachine &TM,
						       const LaserSubtarget &STI) {
  return llvm::createLaserTargetLowering(TM, STI);
}

const char *LaserTargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  case LASERISD::LaserCall: return "LASERISD::LaserCall";
  case LASERISD::ABS_GV: return "LASERISD::ABS_GV";
  case LASERISD::GPRel: return "LASERISD::GPRel";
  case LASERISD::Ret: return "LASERISD::Ret";
  case LASERISD::EH_RETURN: return "LASERISD::EH_RETURN";
  case LASERISD::DivRem: return "LASERISD::DivRem";
  case LASERISD::DivRemU: return "LASERISD::DivRemU";
  case LASERISD::ICmp: return "LASERISD::ICmp";
  case LASERISD::Jump: return "LASERISD::Jump";
  case LASERISD::JumpSetge: return "LASERISD::JumpSetge";
  case LASERISD::Wrapper: return "LASERISD::Wrapper";
  default: return NULL;
  }
}

#include "LaserGenCallingConv.inc"

//===----------------------------------------------------------------------===//
//@ Return Value Calling  Implementation
//===----------------------------------------------------------------------===//
SDValue
LaserTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
				 const SmallVectorImpl<ISD::OutputArg> &Outs,
				 const SmallVectorImpl<SDValue> &OutVals,
				 const SDLoc &dl, SelectionDAG &DAG) const {
  // CCValAssign - represent the assignment of
  // the return value to a location
  SmallVector<CCValAssign, 16> RVLocs;

  MachineFunction &MF = DAG.getMachineFunction();

  // CCState - Info about the registers and stack slot.
  CCState CCInfo(CallConv, isVarArg, MF, RVLocs,*DAG.getContext());
  LaserCC LaserCCInfo(CallConv, CCInfo);

  // Analyze return values.
  LaserCCInfo.analyzeReturn(Outs, Subtarget.abiUsesSoftFloat(),
			   MF.getFunction().getReturnType());
  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  // Copy the result values into the output registers.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    SDValue Val = OutVals[i];
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    // if (RVLocs[i].getValVT() != RVLocs[i].getLocVT())
    //   Val = DAG.getNode(ISD::BITCAST, dl, RVLocs[i].getLocVT(), Val);

    Chain = DAG.getCopyToReg(Chain, dl, VA.getLocReg(), Val, Flag);

    // Guarantee that all emitted copies are stuck together with flags.
    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  // The Laser ABIs (not implemented) for returning structs by value requires
  // that we copy
  // the sret argument into $v0 for the return. We saved the argument into
  // a virtual register in the entry block, so now we copy the value out
  // and into $v0.

  if (MF.getFunction().hasStructRetAttr()) {
    LaserMachineFunctionInfo *LaserFI = MF.getInfo<LaserMachineFunctionInfo>();
    unsigned Reg = LaserFI->getSRetReturnReg();

    if (!Reg)
      llvm_unreachable("sret virtual register not created in the entry block");

    SDValue Val =
      DAG.getCopyFromReg(Chain, dl, Reg, getPointerTy(DAG.getDataLayout()));

    unsigned RETVAL = LASER::RETVAL;
    Chain = DAG.getCopyToReg(Chain, dl, RETVAL, Val, Flag);

    Flag = Chain.getValue(1);

    RetOps.push_back(DAG.getRegister(RETVAL, getPointerTy(DAG.getDataLayout())));
  }

  RetOps[0] = Chain; // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode())
    RetOps.push_back(Flag);

  // Return on Laser is always a "ret"
  return DAG.getNode(LASERISD::Ret, dl, MVT::Other, 
		     Chain, DAG.getRegister(LASER::RETVAL, MVT::i16));
}

template<typename Ty>
void LaserTargetLowering::LaserCC::
analyzeReturn(const SmallVectorImpl<Ty> &RetVals, bool IsSoftFloat,
	      const SDNode *CallNode, const Type *RetTy) const {
  CCAssignFn *Fn;
  Fn = RetCC_LASER;
  for (unsigned I = 0, E = RetVals.size(); I < E; ++I) {
    MVT VT = RetVals[I].VT;
    ISD::ArgFlagsTy Flags = RetVals[I].Flags;
    MVT RegVT = this->getRegVT(VT, RetTy, CallNode, IsSoftFloat);
    if (Fn(I, VT, RegVT, CCValAssign::Full, Flags, this->CCInfo)) {
#ifndef NDEBUG
      dbgs() << "Call result #" << I << " has unhandled type "
	     << EVT(VT).getEVTString() << '\n';
#endif
      llvm_unreachable(nullptr);
    }
  }
}

LaserTargetLowering::LaserCC::LaserCC(
  CallingConv::ID CC,  CCState &Info,
  LaserCC::SpecialCallingConvType SpecialCallingConv_)
  : CCInfo(Info), CallConv(CC) {
  // Pre-allocate reserved argument area.
  CCInfo.AllocateStack(reservedArgArea(), 1);
}

void LaserTargetLowering::LaserCC::
analyzeCallResult(const SmallVectorImpl<ISD::InputArg> &Ins, bool IsSoftFloat,
		  const SDNode *CallNode, const Type *RetTy) const {
  analyzeReturn(Ins, IsSoftFloat, CallNode, RetTy);
}
void LaserTargetLowering::LaserCC::
analyzeReturn(const SmallVectorImpl<ISD::OutputArg> &Outs, bool IsSoftFloat,
	      const Type *RetTy) const {
  analyzeReturn(Outs, IsSoftFloat, nullptr, RetTy);
}

unsigned LaserTargetLowering::LaserCC::reservedArgArea() const {
  //return (IsO32 && (CallConv != CallingConv::Fast)) ? 8 : 0;
  return 0;
}

MVT LaserTargetLowering::LaserCC::getRegVT(MVT VT, const Type *OrigTy,
					 const SDNode *CallNode,
					 bool IsSoftFloat) const {
  if (IsSoftFloat || IsO32)
    return VT;
  return VT;
}


LaserTargetLowering::LaserCC::SpecialCallingConvType
  LaserTargetLowering::getSpecialCallingConv(SDValue Callee) const {
  LaserCC::SpecialCallingConvType SpecialCallingConv =
    LaserCC::NoSpecialCallingConv;
  return SpecialCallingConv;
}

 const LaserTargetLowering *
   llvm::createLaserTargetLowering(const LaserTargetMachine &TM, const LaserSubtarget &STI) {
   return new LaserTargetLowering(TM, STI);
 }

static SDValue performDivRemCombine(SDNode *N, SelectionDAG& DAG,
				    TargetLowering::DAGCombinerInfo &DCI,
				    const LaserSubtarget &Subtarget) {
  if (DCI.isBeforeLegalizeOps())
    return SDValue();

//   EVT Ty = N->getValueType(0);

//   unsigned Opc = LASERISD::DivRem;

//   SDLoc DL(N);

//   SDValue DivLHS = N->getOperand(0);
//   SDValue DivRHS = N->getOperand(1);

//   // Create  LASERISD::DivRem node
//   SDValue DivRem = DAG.getNode(Opc, DL, MVT::Glue,
//   			       N->getOperand(0), N->getOperand(1));

//   SDValue InChain = DAG.getEntryNode();
//   SDValue InGlue = DivRem;

//  // '/' operator
//   if (N->hasAnyUseOfValue(0)) {
//     //    SDValue CopyFromLo = DAG.getCopyFromReg(InChain, DL, LASER::R10, Ty,
//                                             InGlue);
//   //DAG.ReplaceAllUsesOfValueWith(SDValue(N, 0), CopyFromLo);
//   //InChain = CopyFromLo.getValue(1);
//   //InGlue = CopyFromLo.getValue(2);
//   DAG.SelectNodeTo(N, Opc, MVT:i16, DivLHS, DivRHS);
//   }

//   // '%' operator
//   if (N->hasAnyUseOfValue(1)) {
//     //SDValue CopyFromHi = DAG.getCopyFromReg(InChain, DL,
//                                             LASER::R11, Ty, InGlue);
// //DAG.ReplaceAllUsesOfValueWith(SDValue(N, 1), CopyFromHi);
//   }

  return SDValue();
}

SDValue LaserTargetLowering::PerformDAGCombine(SDNode *N, DAGCombinerInfo &DCI)
  const {
  SelectionDAG &DAG = DCI.DAG;
  unsigned Opc = N->getOpcode();
  switch (Opc) {
  default: break;
  case ISD::SDIVREM:
  case ISD::UDIVREM:
    return performDivRemCombine(N, DAG, DCI, Subtarget);
  }
  return SDValue();
}


SDValue LaserTargetLowering::LowerBR_CC(SDValue Op, SelectionDAG &DAG) const
{
  SDValue Chain = Op.getOperand(0);
  SDValue Cond = Op.getOperand(1);
  SDValue LHS = Op.getOperand(2);
  SDValue RHS = Op.getOperand(3);
  SDValue Dest = Op.getOperand(4);
  SDLoc DL(Op);

  SDNode *cond_node = Cond.getNode();

  // Returns a chain & a flag for retval copy to use
  //SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);

  SDValue ICmpSDV = DAG.getNode(LASERISD::ICmp, DL, MVT::i16, LHS, RHS);
  //SDValue ICmpSDV_Chain = ICmpSDV.getValue(0);
  SDValue CompareFlag = ICmpSDV.getValue(0);

  switch (cast<CondCodeSDNode>(cond_node)->get()) {
  default: break;
  case ISD::SETGE:
    return DAG.getNode(LASERISD::JumpSetge, DL, MVT::Other, Chain, Dest, CompareFlag);
    break;
  }

  SDValue s = DAG.getNode(LASERISD::Jump, DL, MVT::Other, Chain, Dest, Cond, CompareFlag);
  return s;
}


SDValue LaserTargetLowering::LowerBRCOND(SDValue Op, SelectionDAG &DAG) const
{
  return Op;
}

SDValue LaserTargetLowering::LowerBlockAddress(SDValue Op,
                                              SelectionDAG &DAG) const {
  // BlockAddressSDNode *N = cast<BlockAddressSDNode>(Op);
  // EVT Ty = Op.getValueType();

  // if (!isPositionIndependent())
  //   return getAddrNonPIC(N, Ty, DAG);

  // return getAddrLocal(N, Ty, DAG);
  return Op;
}

SDValue LaserTargetLowering::LowerJumpTable(SDValue Op, SelectionDAG &DAG) const
{
  // JumpTableSDNode *N = cast<JumpTableSDNode>(Op);
  // EVT Ty = Op.getValueType();

  // if (!isPositionIndependent())
  //   return getAddrNonPIC(N, Ty, DAG);

  // return getAddrLocal(N, Ty, DAG);
  return Op;
}

EVT LaserTargetLowering::getSetCCResultType(EVT VT) const {
  return MVT::i16;
}

SDValue LaserTargetLowering::LowerGlobalAddress(SDValue Op,
                                                SelectionDAG &DAG) const {
  // MachineFunction &MF = DAG.getMachineFunction();

    SDLoc DL(Op);
    EVT VT = Op.getValueType();
    // GlobalAddressSDNode *N = cast<GlobalAddressSDNode>(Op);
    // const GlobalValue *GV = N->getGlobal();
    // int64_t Offset = cast<GlobalAddressSDNode>(Op)->getOffset();

    SDValue Callee;

    unsigned TF = LaserII::MO_ABS;

    if (const GlobalAddressSDNode *GA = dyn_cast<GlobalAddressSDNode>(Op)) {
      Callee = DAG.getTargetGlobalAddress(GA->getGlobal(),
					  SDLoc(GA),
					  GA->getValueType(0),
					  GA->getOffset(), TF);
    } else if (const ConstantPoolSDNode *CP = dyn_cast<ConstantPoolSDNode>(Op)) {
      Callee = DAG.getTargetConstantPool(CP->getConstVal(),
					 CP->getValueType(0),
					 CP->getAlignment(),
					 CP->getOffset(), TF);
    } else if (const BlockAddressSDNode *BA = dyn_cast<BlockAddressSDNode>(Op)) {
      Callee = DAG.getTargetBlockAddress(BA->getBlockAddress(),
				       Op.getValueType(),
				       0,
				       TF);
    } else if (const ExternalSymbolSDNode *ES = dyn_cast<ExternalSymbolSDNode>(Op)) {
      Callee = DAG.getTargetExternalSymbol(ES->getSymbol(),
					 ES->getValueType(0), TF);
    }

    
    // For now we only support small code model, using 16-bit address
    // SDValue Small = 
    //   DAG.getTargetGlobalAddress(
    // 				 GV, DL, getPointerTy(DAG.getDataLayout()), 
    // 				 Offset, LaserII::MO_CALL_FLAG);

    // SDValue IMD_Node = DAG.getNode(LaserISD::IMD, DL,
    // 				   DAG.getVTList(MVT::i16), Small);
    // SDValue LOAD_Node = DAG.getNode(LaserISD::, DL,
    // 				   DAG.getVTList(MVT::i16), Small);

    // unsigned Reg = MF.getRegInfo().createVirtualRegister(getRegClassFor(MVT::i16));
    // return DAG.getCopyToReg(DAG.getEntryNode(), DL, Reg, Small);

    // SDValue Hi = getTargetNode(N, Ty, DAG, LaserII::MO_ABS);
    return DAG.getNode(LASERISD::ABS_GV, DL, VT, Callee);
}

//===----------------------------------------------------------------------===//
//  Lower helper functions
//===----------------------------------------------------------------------===//

// addLiveIn - This helper function adds the specified physical register to the
// MachineFunction as a live in value.  It also creates a corresponding
// virtual register for it.
static unsigned
addLiveIn(MachineFunction &MF, unsigned PReg, const TargetRegisterClass *RC)
{
  unsigned VReg = MF.getRegInfo().createVirtualRegister(RC);
  MF.getRegInfo().addLiveIn(PReg, VReg);
  return VReg;
}

// Passed first two i16 arguments in registers and others in stack.
static bool CC_LaserO16(unsigned ValNo, MVT ValVT, MVT LocVT,
		       CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
		       CCState &State) {
  static const MCPhysReg IntRegs[] = 
    { LASER::R8, LASER::R9};

  // Do not process byval args here.
  if (ArgFlags.isByVal())
    return true;

  // Promote i8 
  if (LocVT == MVT::i8) {
    LocVT = MVT::i16;
    if (ArgFlags.isSExt())
      LocInfo = CCValAssign::SExt;
    else if (ArgFlags.isZExt())
      LocInfo = CCValAssign::ZExt;
    else
      LocInfo = CCValAssign::AExt;
  }

  unsigned Reg;

  unsigned OrigAlign = ArgFlags.getOrigAlign();

  if (ValVT == MVT::i16) {
     Reg = State.AllocateReg(IntRegs);
  } else
    llvm_unreachable("Cannot handle this ValVT.");

  if (!Reg) {
    unsigned Offset = State.AllocateStack(ValVT.getSizeInBits() >> 3,
					  OrigAlign);
    State.addLoc(CCValAssign::getMem(ValNo, ValVT, Offset, LocVT, LocInfo));
  } else
    State.addLoc(CCValAssign::getReg(ValNo, ValVT, Reg, LocVT, LocInfo));
  return false;
}

//===----------------------------------------------------------------------===//
// Call Calling Convention Implementation
//===----------------------------------------------------------------------===//
static const MCPhysReg O16IntRegs[] = {
  LASER::R8, LASER::R9
};


SDValue
LaserTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
			       SmallVectorImpl<SDValue> &InVals) const {
  SelectionDAG &DAG                     = CLI.DAG;
  SDLoc DL                              = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals     = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins   = CLI.Ins;
  SDValue Chain                         = CLI.Chain;
  SDValue Callee                        = CLI.Callee;
  bool &IsTailCall                      = CLI.IsTailCall;
  CallingConv::ID CallConv              = CLI.CallConv;
  bool IsVarArg                         = CLI.IsVarArg;

  // Laser target does not yet support tail call optimization.
  CLI.IsTailCall = false;

  if (IsVarArg) {
    llvm_unreachable("Unimplemented");
  }  

  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetFrameLowering *TFL = MF.getSubtarget().getFrameLowering();
  // MachineRegisterInfo &RegInfo = MF.getRegInfo();
  // LaserMachineFunctionInfo *FuncInfo = MF.getInfo<LaserMachineFunctionInfo>();
  //LaserMachineFunctionInfo *LaserFI = MF.getInfo<LaserMachineFunctionInfo>();

  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 ArgLocs, *DAG.getContext());
  LaserCC::SpecialCallingConvType SpecialCallingConv =
    getSpecialCallingConv(Callee);
  LaserCC LaserCCInfo(CallConv, CCInfo, SpecialCallingConv);
  LaserCCInfo.analyzeCallOperands(Outs, IsVarArg,
				  Subtarget.abiUsesSoftFloat(),
				  Callee.getNode(), CLI.getArgs());

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NextStackOffset = CCInfo.getNextStackOffset();

  // Chain is the output chain of the last Load/Store or CopyToReg node.
  // ByValChain is the output chain of the last Memcpy node created for copying
  // byval arguments to the stack.
  unsigned StackAlignment = TFL->getStackAlignment();
  NextStackOffset = alignTo(NextStackOffset, StackAlignment);
  SDValue NextStackOffsetVal = DAG.getIntPtrConstant(NextStackOffset, DL, true);

  Chain = DAG.getCALLSEQ_START(Chain, NextStackOffset, 0, DL);

  SDValue StackPtr =
    DAG.getCopyFromReg(Chain, DL, LASER::SP,
		       getPointerTy(DAG.getDataLayout()));

  // With EABI is it possible to have 16 args on registers.
  std::deque< std::pair<unsigned, SDValue> > RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;
  LaserCC::byval_iterator ByValArg = LaserCCInfo.byval_begin();

  // Walk the register/memloc assignments, inserting copies/loads.
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    SDValue Arg = OutVals[i];
    CCValAssign &VA = ArgLocs[i];
    MVT LocVT = VA.getLocVT();
    ISD::ArgFlagsTy Flags = Outs[i].Flags;

    //@ByVal Arg {
    if (Flags.isByVal()) {
      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValArg != LaserCCInfo.byval_end());
      assert(!IsTailCall &&
             "Do not tail-call optimize if there is a byval argument.");
      passByValArg(Chain, DL, RegsToPass, MemOpChains, StackPtr, MFI, DAG, Arg,
                   LaserCCInfo, *ByValArg, Flags, Subtarget.isLittle());
      ++ByValArg;
      continue;
    }

    // Promote the value if needed.
    switch (VA.getLocInfo()) {
    default: llvm_unreachable("Unknown loc info!");
    case CCValAssign::Full:
      break;
    case CCValAssign::SExt:
      Arg = DAG.getNode(ISD::SIGN_EXTEND, DL, LocVT, Arg);
      break;
    case CCValAssign::ZExt:
      Arg = DAG.getNode(ISD::ZERO_EXTEND, DL, LocVT, Arg);
      break;
    case CCValAssign::AExt:
      Arg = DAG.getNode(ISD::ANY_EXTEND, DL, LocVT, Arg);
      break;
    }

    // Arguments that can be passed on register must be kept at
    // RegsToPass vector
    if (VA.isRegLoc()) {
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
      continue;
    }

    // Register can't get to this point...
    assert(VA.isMemLoc());

    // emit ISD::STORE whichs stores the
    // parameter value to a stack Location
    MemOpChains.push_back(passArgOnStack(StackPtr, VA.getLocMemOffset(),
                                         Chain, Arg, DL, IsTailCall, DAG));
  } // end for loop

  // Transform all store nodes into one single node because all store
  // nodes are independent of each other.
  if (!MemOpChains.empty())
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, MemOpChains);

  // If the callee is a GlobalAddress/ExternalSymbol node (quite common, every
  // direct call is) turn it into a TargetGlobalAddress/TargetExternalSymbol
  // node so that legalize doesn't hack it.
  SDValue CalleeLo;
  // EVT Ty = Callee.getValueType();

  if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
      Callee = DAG.getTargetGlobalAddress(G->getGlobal(), DL,
                                          getPointerTy(DAG.getDataLayout()), 0,
                                          LaserII::MO_CALL_FLAG);
  }
  else if (ExternalSymbolSDNode *S = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    const char *Sym = S->getSymbol();
    Callee = DAG.getTargetExternalSymbol(Sym,
					 getPointerTy(DAG.getDataLayout()),
					 LaserII::MO_CALL_FLAG);
  }

  // SmallVector<SDValue, 8> Ops(1, Chain);
  // SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);

  // Returns a chain & a flag for retval copy to use.
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);
  SmallVector<SDValue, 8> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);

  // // First we have to move the Callee addres to a virtual register (IMD)
  // unsigned VReg = RegInfo.createVirtualRegister(&LASER::GNPRegsRegClass);
  // MF.addLiveIn(VReg, getRegClassFor(MVT::i16)); 
  // Chain = DAG.getCopyFromReg(Chain, DL, VReg, MVT::i16);
  // Ops.push_back(Chain);


  // Build a sequence of copy-to-reg nodes chained together with token
  // chain and flag operands which copy the outgoing args into registers.
  // The InFlag in necessary since all emitted instructions must be
  // stuck together.
  SDValue InFlag;

  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Chain = CLI.DAG.getCopyToReg(Chain, CLI.DL, RegsToPass[i].first,
                                 RegsToPass[i].second, InFlag);
    InFlag = Chain.getValue(1);
  }

  // Add argument registers to the end of the list so that they are
  // known live into the call.
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i)
    Ops.push_back(CLI.DAG.getRegister(RegsToPass[i].first,
                                      RegsToPass[i].second.getValueType()));

  // Add a register mask operand representing the call-preserved registers.
  const TargetRegisterInfo *TRI = Subtarget.getRegisterInfo();
  const uint32_t *Mask = 
      TRI->getCallPreservedMask(CLI.DAG.getMachineFunction(), CLI.CallConv);
  assert(Mask && "Missing call preserved mask for calling convention");
  Ops.push_back(DAG.getRegisterMask(Mask));

  if (InFlag.getNode())
    Ops.push_back(InFlag);

  // Then we issue the call instruction
  Chain = DAG.getNode(LASERISD::LaserCall, DL, NodeTys, Ops);
  InFlag = Chain.getValue(1);

  // Create the CALLSEQ_END node.
  Chain = DAG.getCALLSEQ_END(Chain, NextStackOffsetVal,
                             DAG.getIntPtrConstant(0, DL, true), InFlag, DL);
  InFlag = Chain.getValue(1);

  // Handle result values, copying them out of physregs into vregs that we
  // return.
  return LowerCallResult(Chain, InFlag, CallConv, IsVarArg,
                         Ins, DL, DAG, InVals, CLI.Callee.getNode(), CLI.RetTy);
}

//@LowerFormalArguments {
/// LowerFormalArguments - transform physical registers into virtual registers
/// and generate load operations for arguments places on the stack.
SDValue
LaserTargetLowering::LowerFormalArguments(SDValue Chain,
					  CallingConv::ID CallConv,
					  bool IsVarArg,
					  const SmallVectorImpl<ISD::InputArg> &Ins,
					  const SDLoc &DL, SelectionDAG &DAG,
					  SmallVectorImpl<SDValue> &InVals)
  const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  LaserMachineFunctionInfo *LaserFI = MF.getInfo<LaserMachineFunctionInfo>();

  LaserFI->setVarArgsFrameIndex(0);

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
  		 ArgLocs, *DAG.getContext());
  LaserCC LaserCCInfo(CallConv, CCInfo);
  LaserFI->setFormalArgInfo(CCInfo.getNextStackOffset(),
  			    LaserCCInfo.hasByValArg());

  Function::const_arg_iterator FuncArg =
    DAG.getMachineFunction().getFunction().arg_begin();
  bool UseSoftFloat = Subtarget.abiUsesSoftFloat();

  LaserCCInfo.analyzeFormalArguments(Ins, UseSoftFloat, FuncArg);

  // Used with vargs to acumulate store chains.
  std::vector<SDValue> OutChains;

  unsigned CurArgIdx = 0;
  LaserCC::byval_iterator ByValArg = LaserCCInfo.byval_begin();

  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    std::advance(FuncArg, Ins[i].OrigArgIndex - CurArgIdx);
    CurArgIdx = Ins[i].OrigArgIndex;
    EVT ValVT = VA.getValVT();
    ISD::ArgFlagsTy Flags = Ins[i].Flags;
    bool IsRegLoc = VA.isRegLoc();

    // byval pass 
    if (Flags.isByVal()) {
      assert(Flags.getByValSize() &&
             "ByVal args of size 0 should have been ignored by front-end.");
      assert(ByValArg != LaserCCInfo.byval_end());
      copyByValRegs(Chain, DL, OutChains, DAG, Flags, InVals, &*FuncArg,
                    LaserCCInfo, *ByValArg);
      ++ByValArg;
      continue;
    }

    // Arguments stored on registers
    if (IsRegLoc) {
      MVT RegVT = VA.getLocVT();
      unsigned ArgReg = VA.getLocReg();
      const TargetRegisterClass *RC = getRegClassFor(RegVT);

      // Transform the arguments stored on
      // physical registers into virtual ones
      unsigned Reg = addLiveIn(DAG.getMachineFunction(), ArgReg, RC);
      SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, RegVT);

      // If this is an 8-bit value, it has been passed promoted
      // to 16 bits. Insert an assert[sz]ext to capture this, then
      // truncate to the right size.
      if (VA.getLocInfo() != CCValAssign::Full) {
	unsigned Opcode = 0;
	if (VA.getLocInfo() == CCValAssign::SExt)
	  Opcode = ISD::AssertSext;
	else if (VA.getLocInfo() == CCValAssign::ZExt)
	  Opcode = ISD::AssertZext;
	if (Opcode)
	  ArgValue = DAG.getNode(Opcode, DL, RegVT, ArgValue,
				 DAG.getValueType(ValVT));
	ArgValue = DAG.getNode(ISD::TRUNCATE, DL, ValVT, ArgValue);
      }


      // Handle floating point arguments passed in integer registers.
      if ((RegVT == MVT::i32 && ValVT == MVT::f32) ||
          (RegVT == MVT::i64 && ValVT == MVT::f64))
        ArgValue = DAG.getNode(ISD::BITCAST, DL, ValVT, ArgValue);
      InVals.push_back(ArgValue);
    } else { // VA.isRegLoc()
      MVT LocVT = VA.getLocVT();

      // sanity check
      assert(VA.isMemLoc());

      // The stack pointer offset is relative to the caller stack frame.
      int FI = MFI.CreateFixedObject(ValVT.getSizeInBits()/8,
                                      VA.getLocMemOffset(), true);

      // Create load nodes to retrieve arguments from the stack
      SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
      SDValue Load = DAG.getLoad(
				 LocVT, DL, Chain, FIN,
				 MachinePointerInfo::getFixedStack(DAG.getMachineFunction(), FI));
      InVals.push_back(Load);
      OutChains.push_back(Load.getValue(1));
    }
  }

  //@Ordinary struct type: 1 {
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    // The cpu0 ABIs for returning structs by value requires that we copy
    // the sret argument into $v0 for the return. Save the argument into
    // a virtual register so that we can access it from the return points.
    if (Ins[i].Flags.isSRet()) {
      unsigned Reg = LaserFI->getSRetReturnReg();
      if (!Reg) {
	Reg = MF.getRegInfo().createVirtualRegister(
						    getRegClassFor(MVT::i16));
	LaserFI->setSRetReturnReg(Reg);
      }
      SDValue Copy = DAG.getCopyToReg(DAG.getEntryNode(), DL, Reg, InVals[i]);
      Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, Copy, Chain);
      break;
    }
  }

  // All stores are grouped in one node to allow the matching between
  // the size of Ins and InVals. This only happens when on varg functions
  if (!OutChains.empty()) {
    OutChains.push_back(Chain);
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, OutChains);
  }

  return Chain;
}

// @LowerFormalArguments }

void LaserTargetLowering::LaserCC::
analyzeFormalArguments(const SmallVectorImpl<ISD::InputArg> &Args,
		       bool IsSoftFloat, Function::const_arg_iterator FuncArg) {
  unsigned NumArgs = Args.size();
  llvm::CCAssignFn *FixedFn = fixedArgFn();
  unsigned CurArgIdx = 0;
  for (unsigned I = 0; I != NumArgs; ++I) {
    MVT ArgVT = Args[I].VT;
    ISD::ArgFlagsTy ArgFlags = Args[I].Flags;
    std::advance(FuncArg, Args[I].OrigArgIndex - CurArgIdx);
    CurArgIdx = Args[I].OrigArgIndex;
    if (ArgFlags.isByVal()) {
      handleByValArg(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags);
      continue;
    }
    MVT RegVT = getRegVT(ArgVT, FuncArg->getType(), nullptr, IsSoftFloat);
    if (!FixedFn(I, ArgVT, RegVT, CCValAssign::Full, ArgFlags, CCInfo))
      continue;
#ifndef NDEBUG
    dbgs() << "Formal Arg #" << I << " has unhandled type "
	   << EVT(ArgVT).getEVTString();
#endif
    llvm_unreachable(nullptr);
  }
}

void LaserTargetLowering::LaserCC::handleByValArg(unsigned ValNo, MVT ValVT,
						MVT LocVT,
						CCValAssign::LocInfo LocInfo,
						ISD::ArgFlagsTy ArgFlags) {
  assert(ArgFlags.getByValSize() && "Byval argument's size shouldn't be 0.");
  struct ByValArgInfo ByVal;
  unsigned RegSize = regSize();
  unsigned ByValSize = alignTo(ArgFlags.getByValSize(), RegSize);
  unsigned Align = std::min(std::max(ArgFlags.getByValAlign(), RegSize),
			    RegSize * 2);
  if (useRegsForByval())
    allocateRegs(ByVal, ByValSize, Align);

  // Allocate space on caller's stack.
  ByVal.Address = CCInfo.AllocateStack(ByValSize - RegSize * ByVal.NumRegs,
				       Align);
  CCInfo.addLoc(CCValAssign::getMem(ValNo, ValVT, ByVal.Address, LocVT,
				    LocInfo));
  ByValArgs.push_back(ByVal);
}

unsigned LaserTargetLowering::LaserCC::numIntArgRegs() const {
  return IsO32 ? array_lengthof(O16IntRegs) : 0;
}

const ArrayRef<MCPhysReg> LaserTargetLowering::LaserCC::intArgRegs() const {
  return makeArrayRef(O16IntRegs);
}

llvm::CCAssignFn *LaserTargetLowering::LaserCC::fixedArgFn() const {
  // if (IsO32)
    return CC_LaserO16;
  // else // IsS32
  //   return CC_LaserS16;
}

void LaserTargetLowering::LaserCC::allocateRegs(ByValArgInfo &ByVal,
					      unsigned ByValSize,
					      unsigned Align) {
  unsigned RegSize = regSize(), NumIntArgRegs = numIntArgRegs();
  const ArrayRef<MCPhysReg> IntArgRegs = intArgRegs();
  assert(!(ByValSize % RegSize) && !(Align % RegSize) &&
	 "Byval argument's size and alignment should be a multiple of"
	 "RegSize.");
  ByVal.FirstIdx = CCInfo.getFirstUnallocated(IntArgRegs);
  // If Align > RegSize, the first arg register must be even.
  if ((Align > RegSize) && (ByVal.FirstIdx % 2)) {
    CCInfo.AllocateReg(IntArgRegs[ByVal.FirstIdx]);
    ++ByVal.FirstIdx;
  }
  // Mark the registers allocated.
  for (unsigned I = ByVal.FirstIdx; ByValSize && (I < NumIntArgRegs);
       ByValSize -= RegSize, ++I, ++ByVal.NumRegs)
    CCInfo.AllocateReg(IntArgRegs[I]);
}

void LaserTargetLowering::
copyByValRegs(SDValue Chain, const SDLoc &DL, std::vector<SDValue> &OutChains,
              SelectionDAG &DAG, const ISD::ArgFlagsTy &Flags,
              SmallVectorImpl<SDValue> &InVals, const Argument *FuncArg,
              const LaserCC &CC, const ByValArgInfo &ByVal) const {

  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned RegAreaSize = ByVal.NumRegs * CC.regSize();
  unsigned FrameObjSize = std::max(Flags.getByValSize(), RegAreaSize);
  int FrameObjOffset;

  const ArrayRef<MCPhysReg> ByValArgRegs = CC.intArgRegs();

  if (RegAreaSize)
    FrameObjOffset = (int)CC.reservedArgArea() -
      (int)((CC.numIntArgRegs() - ByVal.FirstIdx) * CC.regSize());
  else
    FrameObjOffset = ByVal.Address;

  // Create frame object.
  EVT PtrTy = getPointerTy(DAG.getDataLayout());
  int FI = MFI.CreateFixedObject(FrameObjSize, FrameObjOffset, true);
  SDValue FIN = DAG.getFrameIndex(FI, PtrTy);
  InVals.push_back(FIN);

  if (!ByVal.NumRegs)
    return;

  // Copy arg registers.
  MVT RegTy = MVT::getIntegerVT(CC.regSize() * 8);
  const TargetRegisterClass *RC = getRegClassFor(RegTy);

  for (unsigned I = 0; I < ByVal.NumRegs; ++I) {
    unsigned ArgReg = ByValArgRegs[ByVal.FirstIdx + I];
    unsigned VReg = addLiveIn(MF, ArgReg, RC);
    unsigned Offset = I * CC.regSize();
    SDValue StorePtr = DAG.getNode(ISD::ADD, DL, PtrTy, FIN,
                                   DAG.getConstant(Offset, DL, PtrTy));
    SDValue Store = DAG.getStore(Chain, DL, DAG.getRegister(VReg, RegTy),
                                 StorePtr, MachinePointerInfo(FuncArg, Offset));
    OutChains.push_back(Store);
  }
}

void LaserTargetLowering::LaserCC::
analyzeCallOperands(const SmallVectorImpl<ISD::OutputArg> &Args,
                    bool IsVarArg, bool IsSoftFloat, const SDNode *CallNode,
                    std::vector<ArgListEntry> &FuncArgs) {

  assert((CallConv != CallingConv::Fast || !IsVarArg) &&
         "CallingConv::Fast shouldn't be used for vararg functions.");

  unsigned NumOpnds = Args.size();
  llvm::CCAssignFn *FixedFn = fixedArgFn();

  for (unsigned I = 0; I != NumOpnds; ++I) {
    MVT ArgVT = Args[I].VT;
    ISD::ArgFlagsTy ArgFlags = Args[I].Flags;
    bool R;

    if (ArgFlags.isByVal()) {
      handleByValArg(I, ArgVT, ArgVT, CCValAssign::Full, ArgFlags);
      continue;
    }

    MVT RegVT = getRegVT(ArgVT, FuncArgs[Args[I].OrigArgIndex].Ty, CallNode,
                           IsSoftFloat);
    R = FixedFn(I, ArgVT, RegVT, CCValAssign::Full, ArgFlags, CCInfo);

    if (R) {
#ifndef NDEBUG
      dbgs() << "Call operand #" << I << " has unhandled type "
             << EVT(ArgVT).getEVTString();
#endif
      llvm_unreachable(nullptr);
    }
  }
}

void LaserTargetLowering::
passByValArg(SDValue Chain, const SDLoc &DL,
             std::deque< std::pair<unsigned, SDValue> > &RegsToPass,
             SmallVectorImpl<SDValue> &MemOpChains, SDValue StackPtr,
             MachineFrameInfo &MFI, SelectionDAG &DAG, SDValue Arg,
             const LaserCC &CC, const ByValArgInfo &ByVal,
             const ISD::ArgFlagsTy &Flags, bool isLittle) const {

  unsigned ByValSizeInBytes = Flags.getByValSize();
  unsigned OffsetInBytes = 0; // From beginning of struct
  unsigned RegSizeInBytes = CC.regSize();
  unsigned Alignment = std::min(Flags.getByValAlign(), RegSizeInBytes);
  EVT PtrTy = getPointerTy(DAG.getDataLayout()),
      RegTy = MVT::getIntegerVT(RegSizeInBytes * 8);

  if (ByVal.NumRegs) {
    const ArrayRef<MCPhysReg> ArgRegs = CC.intArgRegs();
    bool LeftoverBytes = (ByVal.NumRegs * RegSizeInBytes > ByValSizeInBytes);
    unsigned I = 0;

    // Copy words to registers.
    for (; I < ByVal.NumRegs - LeftoverBytes;
         ++I, OffsetInBytes += RegSizeInBytes) {
      SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                    DAG.getConstant(OffsetInBytes, DL, PtrTy));
      SDValue LoadVal = DAG.getLoad(RegTy, DL, Chain, LoadPtr,
                                    MachinePointerInfo());
      MemOpChains.push_back(LoadVal.getValue(1));
      unsigned ArgReg = ArgRegs[ByVal.FirstIdx + I];
      RegsToPass.push_back(std::make_pair(ArgReg, LoadVal));
    }

    // Return if the struct has been fully copied.
    if (ByValSizeInBytes == OffsetInBytes)
      return;

    // Copy the remainder of the byval argument with sub-word loads and shifts.
    if (LeftoverBytes) {
      assert((ByValSizeInBytes > OffsetInBytes) &&
             (ByValSizeInBytes < OffsetInBytes + RegSizeInBytes) &&
             "Size of the remainder should be smaller than RegSizeInBytes.");
      SDValue Val;

      for (unsigned LoadSizeInBytes = RegSizeInBytes / 2, TotalBytesLoaded = 0;
           OffsetInBytes < ByValSizeInBytes; LoadSizeInBytes /= 2) {
        unsigned RemainingSizeInBytes = ByValSizeInBytes - OffsetInBytes;

        if (RemainingSizeInBytes < LoadSizeInBytes)
          continue;

        // Load subword.
        SDValue LoadPtr = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                                      DAG.getConstant(OffsetInBytes, DL, PtrTy));
        SDValue LoadVal = DAG.getExtLoad(
            ISD::ZEXTLOAD, DL, RegTy, Chain, LoadPtr, MachinePointerInfo(),
            MVT::getIntegerVT(LoadSizeInBytes * 8), Alignment);
        MemOpChains.push_back(LoadVal.getValue(1));

        // Shift the loaded value.
        unsigned Shamt;

        if (isLittle)
          Shamt = TotalBytesLoaded * 8;
        else
          Shamt = (RegSizeInBytes - (TotalBytesLoaded + LoadSizeInBytes)) * 8;

        SDValue Shift = DAG.getNode(ISD::SHL, DL, RegTy, LoadVal,
                                    DAG.getConstant(Shamt, DL, MVT::i32));

        if (Val.getNode())
          Val = DAG.getNode(ISD::OR, DL, RegTy, Val, Shift);
        else
          Val = Shift;

        OffsetInBytes += LoadSizeInBytes;
        TotalBytesLoaded += LoadSizeInBytes;
        Alignment = std::min(Alignment, LoadSizeInBytes);
      }

      unsigned ArgReg = ArgRegs[ByVal.FirstIdx + I];
      RegsToPass.push_back(std::make_pair(ArgReg, Val));
      return;
    }
  }

  // Copy remainder of byval arg to it with memcpy.
  unsigned MemCpySize = ByValSizeInBytes - OffsetInBytes;
  SDValue Src = DAG.getNode(ISD::ADD, DL, PtrTy, Arg,
                            DAG.getConstant(OffsetInBytes, DL, PtrTy));
  SDValue Dst = DAG.getNode(ISD::ADD, DL, PtrTy, StackPtr,
                            DAG.getIntPtrConstant(ByVal.Address, DL));
  Chain = DAG.getMemcpy(Chain, DL, Dst, Src,
                        DAG.getConstant(MemCpySize, DL, PtrTy),
                        Alignment, /*isVolatile=*/false, /*AlwaysInline=*/false,
                        /*isTailCall=*/false,
                        MachinePointerInfo(), MachinePointerInfo());
  MemOpChains.push_back(Chain);
}

SDValue
LaserTargetLowering::passArgOnStack(SDValue StackPtr, unsigned Offset,
                                   SDValue Chain, SDValue Arg, const SDLoc &DL,
                                   bool IsTailCall, SelectionDAG &DAG) const {
  if (!IsTailCall) {
    SDValue PtrOff =
        DAG.getNode(ISD::ADD, DL, getPointerTy(DAG.getDataLayout()), StackPtr,
                    DAG.getIntPtrConstant(Offset, DL));
    return DAG.getStore(Chain, DL, Arg, PtrOff, MachinePointerInfo());
  }

  MachineFrameInfo &MFI = DAG.getMachineFunction().getFrameInfo();
  int FI = MFI.CreateFixedObject(Arg.getValueSizeInBits() / 8, Offset, false);
  SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
  return DAG.getStore(Chain, DL, Arg, FIN, MachinePointerInfo(),
                      /* Alignment = */ 0, MachineMemOperand::MOVolatile);
}

SDValue LaserTargetLowering::getGlobalReg(SelectionDAG &DAG, EVT Ty) const {
  LaserMachineFunctionInfo *FI = DAG.getMachineFunction().getInfo<LaserMachineFunctionInfo>();
  return DAG.getRegister(FI->getGlobalBaseReg(), Ty);
}


/// LowerCallResult - Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
SDValue
LaserTargetLowering::LowerCallResult(SDValue Chain, SDValue InFlag,
                                    CallingConv::ID CallConv, bool IsVarArg,
                                    const SmallVectorImpl<ISD::InputArg> &Ins,
                                    const SDLoc &DL, SelectionDAG &DAG,
                                    SmallVectorImpl<SDValue> &InVals,
                                    const SDNode *CallNode,
                                    const Type *RetTy) const {
  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
		 RVLocs, *DAG.getContext());
  LaserCC LaserCCInfo(CallConv, CCInfo);

  LaserCCInfo.analyzeCallResult(Ins, Subtarget.abiUsesSoftFloat(),
                               CallNode, RetTy);

  // Copy all of the result registers out of their specified physreg.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    SDValue Val = DAG.getCopyFromReg(Chain, DL, RVLocs[i].getLocReg(),
                                     RVLocs[i].getLocVT(), InFlag);
    Chain = Val.getValue(1);
    InFlag = Val.getValue(2);

    if (RVLocs[i].getValVT() != RVLocs[i].getLocVT())
      Val = DAG.getNode(ISD::BITCAST, DL, RVLocs[i].getValVT(), Val);

    InVals.push_back(Val);
  }

  return Chain;
}

// Create a TargetExternalSymbol node.
SDValue LaserTargetLowering::getTargetNode(ExternalSymbolSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetExternalSymbol(N->getSymbol(), Ty, Flag);
}

// Create a Target GlobalAddress node.
SDValue LaserTargetLowering::getTargetNode(GlobalAddressSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetGlobalAddress(N->getGlobal(), SDLoc(N), Ty, 0, Flag);
}

// Create a TargetBlockAddress node.
SDValue LaserTargetLowering::getTargetNode(BlockAddressSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetBlockAddress(N->getBlockAddress(), Ty, 0, Flag);
}

// Create a TargetJumpTable node.
SDValue LaserTargetLowering::getTargetNode(JumpTableSDNode *N, EVT Ty,
                                          SelectionDAG &DAG,
                                          unsigned Flag) const {
  return DAG.getTargetJumpTable(N->getIndex(), Ty, Flag);
}

