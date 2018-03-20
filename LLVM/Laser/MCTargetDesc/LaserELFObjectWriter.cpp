//===-- LaserELFObjectWriter.cpp - Laser ELF Writer -------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/LaserBaseInfo.h"
#include "MCTargetDesc/LaserFixupKinds.h"
#include "MCTargetDesc/LaserMCTargetDesc.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"
#include <list>

using namespace llvm;

namespace {
  class LaserELFObjectWriter : public MCELFObjectTargetWriter {
  public:
    explicit LaserELFObjectWriter(uint8_t OSABI);

    ~LaserELFObjectWriter() override = default;

  protected:
    unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
			  const MCFixup &Fixup, bool IsPCRel) const override;
    bool needsRelocateWithSymbol(const MCSymbol &Sym,
                                 unsigned Type) const override;
  };
} // end anonymous namespace

LaserELFObjectWriter::LaserELFObjectWriter(uint8_t OSABI)
  : MCELFObjectTargetWriter(/*_is64Bit=false*/ false, OSABI, ELF::EM_LASER,
                            /*HasRelocationAddend*/ true) {}

//@GetRelocType {
unsigned LaserELFObjectWriter::getRelocType(MCContext &Ctx,
                                           const MCValue &Target,
                                           const MCFixup &Fixup,
                                           bool IsPCRel) const {
  // determine the type of the relocation
  unsigned Type = (unsigned)ELF::R_LASER_NONE;
  unsigned Kind = (unsigned)Fixup.getKind();

  switch (Kind) {
  default:
    llvm_unreachable("invalid fixup kind!");
  case Laser::fixup_Laser_CALL16:
    Type = ELF::R_LASER_CALL16;
    break;
  case Laser::fixup_Laser_PC11:
    Type = ELF::R_LASER_PC11;
    break;
  case Laser::fixup_Laser_GV16:
    Type = ELF::R_LASER_GV16;
    break;
  }

  return Type;
}
//@GetRelocType }

bool
LaserELFObjectWriter::needsRelocateWithSymbol(const MCSymbol &Sym,
                                             unsigned Type) const {
    return false;
}

std::unique_ptr<MCObjectWriter>
llvm::createLaserELFObjectWriter(raw_pwrite_stream &OS,
				 uint8_t OSABI) {
  return createELFObjectWriter (llvm::make_unique<LaserELFObjectWriter>(OSABI),
				OS, /*IsLittleEndian*/ false);
}

