//===- LaserDisassembler.cpp - Disassembler for Laser -------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file is part of the Laser Disassembler.
//
//===----------------------------------------------------------------------===//

#include "Laser.h"

#include "LaserRegisterInfo.h"
#include "LaserSubtarget.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "laser-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {


/// LaserDisassembler - a disasembler class for Laser.
class LaserDisassembler : public MCDisassembler {
public:
  /// Constructor     - Initializes the disassembler.
  ///
  LaserDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}
  virtual ~LaserDisassembler() {}

  /// getInstruction - See MCDisassembler.
  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &VStream,
                              raw_ostream &CStream) const override;
};
} // end anonymous namespace

// Decoder tables for GPR register
static const unsigned GNPRegsTable[] = {
  LASER::FLAGR,
  LASER::SP,
  LASER::FP,
  LASER::SS,
  LASER::LR,
  LASER::RETADDR,
  LASER::GP,
  LASER::RETVAL,
  LASER::R8, 
  LASER::R9, 
  LASER::R10, 
  LASER::R11, 
  LASER::R12, 
  LASER::R13, 
  LASER::R14, 
  LASER::R15,
};

static const unsigned RTRegsTable[] = {
  LASER::R8, 
  LASER::R9, 
  LASER::R10, 
  LASER::R11, 
  LASER::R12, 
  LASER::R13, 
  LASER::R14, 
  LASER::R15
};


static DecodeStatus DecodeGNPRegsRegisterClass(MCInst &Inst,
                                               unsigned RegNo,
                                               uint64_t Address,
                                               const void *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;

  Inst.addOperand(MCOperand::createReg(GNPRegsTable[RegNo]));
  return MCDisassembler::Success;

}

static DecodeStatus DecodeRTRegsRegisterClass(MCInst &Inst,
                                               unsigned RegNo,
                                               uint64_t Address,
                                               const void *Decoder) {
  if (RegNo > 8) 
    return MCDisassembler::Fail;

  Inst.addOperand(MCOperand::createReg(RTRegsTable[RegNo]));
  return MCDisassembler::Success;

}

static DecodeStatus DecodeLASERimm16(MCInst &Inst,
				     unsigned Insn,
				     uint64_t Address,
				     const void *Decoder) {
  Inst.addOperand(MCOperand::createImm(Insn));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeLASERmemsrc(MCInst &Inst,
                              unsigned RegNo,
                              uint64_t Address,
                              const void *Decoder) {


  Inst.addOperand(MCOperand::createReg(GNPRegsTable[RegNo]));

  return MCDisassembler::Success;
}

static DecodeStatus DecodeINC(MCInst &Inst, unsigned insn, uint64_t Address, const void *Decoder);



static MCDisassembler *createLaserDisassembler(
                       const Target &T,
                       const MCSubtargetInfo &STI,
                       MCContext &Ctx) {
  return new LaserDisassembler(STI, Ctx);
}

extern "C" void LLVMInitializeLaserDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheLaserTarget(),
                                         createLaserDisassembler);
}

#include "LaserGenDisassemblerTables.inc"

static DecodeStatus DecodeINC(MCInst &Inst, unsigned insn, uint64_t Address, const void *Decoder) {

  unsigned rd = fieldFromInstruction(insn, 10, 7);
  
  // Decode rd
  DecodeStatus status = DecodeGNPRegsRegisterClass(Inst, rd, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  Inst.addOperand(MCOperand::createReg(GNPRegsTable[rd]));

  return MCDisassembler::Success;
}


/// Read two bytes from the ArrayRef and return 16 bit word sorted
/// according to the given endianess
static DecodeStatus readInstruction16(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint16_t &Insn) {
  // We want to read exactly 2 Bytes of data.
  if (Bytes.size() < 2) {
    Size = 0;
    return MCDisassembler::Fail;
  }

    // Encoded as a big-endian 16-bit word in the stream.
    Insn = (Bytes[1] << 0) |
           (Bytes[0] << 8);

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction32(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {
  // We want to read exactly 2 Bytes of data.
  if (Bytes.size() < 4) {
    Size = 0;
    return MCDisassembler::Fail;
  }

    // Encoded as a big-endian 32-bit word in the stream.
  Insn = (Bytes[0] << 24) |
         (Bytes[1] << 16) |
         (Bytes[2] <<  8) |
         (Bytes[3] <<  0);

  return MCDisassembler::Success;
}


DecodeStatus LaserDisassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                               ArrayRef<uint8_t> Bytes,
                                               uint64_t Address,
                                               raw_ostream &VStream,
                                               raw_ostream &CStream) const {
  uint16_t Insn;

  DecodeStatus Result;

  Result = readInstruction16(Bytes, Address, Size, Insn);

  if (Result == MCDisassembler::Fail)
    return MCDisassembler::Fail;

  // Calling the auto-generated decoder function.
  Result = decodeInstruction(DecoderTableLASER16, Instr, Insn, Address,
                             this, STI);
  if (Result != MCDisassembler::Fail) {
    Size = 2;
    return Result;
  }
  else {
    // Match 32 bit IMD instruction
  uint32_t Insn2;
  Result = readInstruction32(Bytes, Address, Size, Insn2);

  if (Result == MCDisassembler::Fail)
    return MCDisassembler::Fail;

  Result = decodeInstruction(DecoderTableLASER32, Instr, Insn2, Address,
                             this, STI);
  if (Result != MCDisassembler::Fail) {
    Size = 4;
    return Result;
  }
  else
    return MCDisassembler::Fail;
  }

  return MCDisassembler::Fail;
}





