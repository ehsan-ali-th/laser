//===-- LaserISelDAGToDAG.cpp - A dag to dag inst selector for Laser ------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the LASER target.
//
//===----------------------------------------------------------------------===//

#include "LaserISelDAGToDAG.h"
#include "Laser.h"

#include "LaserTargetMachine.h"
#include "LaserMachineFunctionInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "laser-isel"

bool LaserDAGToDAGISel::SelectAddr(SDNode *Parent, SDValue Addr, SDValue &Base, SDValue &Offset) {

  EVT ValTy = Addr.getValueType();
  SDLoc DL(Addr);

  // if Address is FI, get the TargetFrameIndex.
  if (FrameIndexSDNode *FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
    EVT PtrVT = getTargetLowering()->getPointerTy(CurDAG->getDataLayout());
    Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), PtrVT);
    Offset = CurDAG->getTargetConstant(0, DL, ValTy);
    // ExtraReg = CurDAG->getRegister(Reg);
    return true;
  }

  if (Addr.getOpcode() == LASERISD::Wrapper) {
    Base   = Addr.getOperand(0);
    Offset = Addr.getOperand(1);
    return true;
  }


  //@relocation_model: static
  if (Addr.getOpcode() == ISD::TargetExternalSymbol ||
      Addr.getOpcode() == ISD::TargetGlobalAddress ||
      Addr.getOpcode() == ISD::TargetGlobalTLSAddress) {
    return false; // direct calls.
  }

  Base = Addr;
  Offset = CurDAG->getTargetConstant(0, DL, ValTy);
  // ExtraReg = CurDAG->getRegister(Reg);
  return true;
}

void LaserDAGToDAGISel::Select(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();
  SDLoc dl(Node);

  // Dump information about the Node being selected
  DEBUG(errs() << "Selecting: "; Node->dump(CurDAG); errs() << "\n");

  // If we have a custom node, we already have selected!
  if (Node->isMachineOpcode()) {
    DEBUG(errs() << "== "; Node->dump(CurDAG); errs() << "\n");
    Node->setNodeId(-1);
    return;   // Already selected.
  }

  // Instruction Selection not handled by the auto-generated
  // tablegen selection should be handled here.
  switch(Opcode) {
  default: break;

  case ISD::LOAD:
  case ISD::STORE:
    break;
  case ISD::SDIVREM:
    // SDValue DivLHS = Node->getOperand(0);
    // SDValue DivRHS = Node->getOperand(1);
    
    // SDNode *Div = CurDAG->getMachineNode(LASER::DIV, dl,
    // 					 MVT::i16, MVT::Glue, DivLHS, DivRHS);
    // // '/' operator
    // if (Node->hasAnyUseOfValue(0)) {
    //   SDValue CopyFromLo = CurDAG->getCopyFromReg(InChain, DL, DivLHS, Ty,
    // 					      InGlue);
    //   DAG.ReplaceAllUsesOfValueWith(SDValue(N, 0), CopyFromLo);
    //   InChain = CopyFromLo.getValue(1);
    //   InGlue = CopyFromLo.getValue(2);
    // }

    // // '%' operator
    // if (Node->hasAnyUseOfValue(1)) {
    //   SDValue CopyFromHi = DAG.getCopyFromReg(InChain, DL,
    // 					      DivRHS, Ty, InGlue);
    //   DAG.ReplaceAllUsesOfValueWith(SDValue(N, 1), CopyFromHi);
    // }
    break;
  case ISD::GLOBAL_OFFSET_TABLE:
    ReplaceNode(Node, getGlobalBaseReg());
    return;

  }
  
  // Select the default instruction
  SelectCode(Node);

}

/// createLaserISelDag - This pass converts a legalized DAG into a
/// LASER-specific DAG, ready for instruction scheduling.
///
FunctionPass *llvm::createLaserISelDag(LaserTargetMachine &TM) {
  return new LaserDAGToDAGISel(TM);
  }

void LaserDAGToDAGISel::processFunctionAfterISel(MachineFunction &MF) {
}

std::pair<bool, SDNode*> LaserDAGToDAGISel::selectNode(SDNode *Node) {
  unsigned Opcode = Node->getOpcode();
  SDLoc DL(Node);
  ///
  // Instruction Selection not handled by the auto-generated
  // tablegen selection should be handled here.
  ///
  // SDNode *Result;
  ///
  // Instruction Selection not handled by the auto-generated
  // tablegen selection should be handled here.
  ///
  // EVT NodeTy = Node->getValueType(0);
  // unsigned MultOpc;
  switch(Opcode) {
  default: break;
  }
  return std::make_pair(false, nullptr);
}

/// getGlobalBaseReg - Output the instructions required to put the
/// GOT address into a register.
SDNode *LaserDAGToDAGISel::getGlobalBaseReg() {
  unsigned GlobalBaseReg = MF->getInfo<LaserMachineFunctionInfo>()->getGlobalBaseReg();
  return CurDAG->
    getRegister(GlobalBaseReg, 
		getTargetLowering()->getPointerTy(CurDAG->getDataLayout()))
    .getNode();
}


