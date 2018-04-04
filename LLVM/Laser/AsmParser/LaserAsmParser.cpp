//===-- LaserAsmParser.cpp - Parse Laser assembly to MCInst instructions ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Laser.h"

#include "MCTargetDesc/LaserMCExpr.h"
#include "MCTargetDesc/LaserMCTargetDesc.h"
#include "LaserRegisterInfo.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "laser-asm-parser"

namespace {
class LaserAssemblerOptions {
public:
  LaserAssemblerOptions():
    reorder(true), macro(true) {
  }

  bool isReorder() {return reorder;}
  void setReorder() {reorder = true;}
  void setNoreorder() {reorder = false;}

  bool isMacro() {return macro;}
  void setMacro() {macro = true;}
  void setNomacro() {macro = false;}

private:
  bool reorder;
  bool macro;
};
}

namespace {

struct LaserOperand;

class LaserAsmParser : public MCTargetAsmParser {
  MCAsmParser &Parser;
  LaserAssemblerOptions Options;


#define GET_ASSEMBLER_HEADER
#include "LaserGenAsmMatcher.inc"

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool parseMathOperation(StringRef Name, SMLoc NameLoc,
                        OperandVector &Operands);

  bool ParseDirective(AsmToken DirectiveID) override;

  OperandMatchResultTy parseMem16Operand(OperandVector &Operands);
  OperandMatchResultTy parseImmOperand(OperandVector &Operands);
  OperandMatchResultTy parseLASERjmptarget11Operand(OperandVector &Operands);

  bool ParseOperand(OperandVector &Operands, StringRef Mnemonic);

  int tryParseRegister(StringRef Mnemonic);

  bool tryParseRegisterOperand(OperandVector &Operands,
                               StringRef Mnemonic);

  bool needsExpansion(MCInst &Inst);

  void expandInstruction(MCInst &Inst, SMLoc IDLoc,
                         SmallVectorImpl<MCInst> &Instructions);
  void expandLoadImm(MCInst &Inst, SMLoc IDLoc,
                     SmallVectorImpl<MCInst> &Instructions);
  void expandLoadAddressImm(MCInst &Inst, SMLoc IDLoc,
                            SmallVectorImpl<MCInst> &Instructions);
  void expandLoadAddressReg(MCInst &Inst, SMLoc IDLoc,
                            SmallVectorImpl<MCInst> &Instructions);
  bool reportParseError(StringRef ErrorMsg);

  bool parseMemOffset(const MCExpr *&Res);
  bool parseRelocOperand(const MCExpr *&Res);

  const MCExpr *evaluateRelocExpr(const MCExpr *Expr, StringRef RelocStr);

  bool parseDirectiveSet();

  bool parseSetAtDirective();
  bool parseSetNoAtDirective();
  bool parseSetMacroDirective();
  bool parseSetNoMacroDirective();
  bool parseSetReorderDirective();
  bool parseSetNoReorderDirective();

  int matchRegisterName(StringRef Symbol);

  int matchRegisterByNumber(unsigned RegNum, StringRef Mnemonic);

  unsigned getReg(int RC,int RegNo);

public:
  LaserAsmParser(const MCSubtargetInfo &STI, MCAsmParser &parser,
                const MCInstrInfo &MII, const MCTargetOptions &Options)
    : MCTargetAsmParser(Options, STI, MII), Parser(parser) {
    // Initialize the set of available features.
    setAvailableFeatures(ComputeAvailableFeatures(getSTI().getFeatureBits()));
    Parser.setShowParsedOperands(true);
  }

  MCAsmParser &getParser() const { return Parser; }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }

};
}

namespace {

/// LaserOperand - Instances of this class represent a parsed Laser machine
/// instruction.
class LaserOperand : public MCParsedAsmOperand {
private:

  // Here we define the operand kinds
  enum KindTy {
    k_Immediate,
    k_Memory16,
    k_Memory11,
    k_Register,
    k_Token
  } Kind;

  SMLoc StartLoc, EndLoc;

  struct Token {
    const char *Data;
    unsigned Length;
  };
  struct PhysRegOp {
    unsigned RegNum; /// Register Number
  };
  struct ImmOp {
    const MCExpr *Val;
  };

  struct MemOp16 {
    unsigned Base;
    const MCExpr *Off;
  };

  struct MemOp11 {
    unsigned Base;
    const MCExpr *Off;
  };


  union {
    struct Token Tok;
    struct PhysRegOp Reg;
    struct ImmOp Imm;
    struct MemOp16 Mem16;
    struct MemOp11 Mem11;
  };

public:
  LaserOperand(KindTy K) : MCParsedAsmOperand(), Kind(K) {}

  void print(raw_ostream &OS) const override {
    switch (Kind) {
    case k_Token:     OS << "Token: " << getToken() << "\n"; break;
    case k_Register:  OS << "Reg No. = " << getReg() << "\n"; break;
    case k_Immediate: OS << "Imm: #" << getImm() << "\n"; break;
    case k_Memory16: OS << "Mem16: " << getMemBase16() << "+"
		      <<  "\n"; break;
    case k_Memory11: OS << "Mem11: " << getMemBase11() << "+"
		      <<  "\n"; break;
    }
  }



public:
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const{
    // Add as immediate when possible.  Null MCExpr = 0.
    if (Expr == 0)
      Inst.addOperand(MCOperand::createImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    const MCExpr *Expr = getImm();
    addExpr(Inst,Expr);
  }

  void addMem16Operands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getMemBase16()));

    const MCExpr *Expr = getMemOff16();
    addExpr(Inst,Expr);
  }

  void addMem11Operands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getMemBase11()));

    const MCExpr *Expr = getMemOff11();
    addExpr(Inst,Expr);
  }

  void addLaserImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }

    void addLASERjmptarget11Operands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }


  


  bool isReg() const override { return Kind == k_Register; }
  bool isImm() const override { return Kind == k_Immediate; }
  bool isMem() const override {
    return isMem16() || isMem11();
  }

  bool isLaserImm() const { return Kind == k_Immediate; }
  bool isToken() const { return Kind == k_Token; }
  bool isMem16() const { return Kind == k_Memory16; }
  bool isMem11() const { return Kind == k_Memory11; }
  bool isLASERjmptarget11() const { return isImm() || isToken(); }


  StringRef getToken() const {
    assert(Kind == k_Token && "Invalid access!");
    return StringRef(Tok.Data, Tok.Length);
  }


  unsigned getReg() const {
    assert((Kind == k_Register) && "Invalid access!");
    return Reg.RegNum;
  }

  const MCExpr *getImm() const {
    assert((Kind == k_Immediate) && "Invalid access!");
    return Imm.Val;
  }

  unsigned getMemBase16() const {
    assert((Kind == k_Memory16) && "Invalid access!");
    return Mem16.Base;
  }

  unsigned getMemBase11() const {
    assert((Kind == k_Memory11) && "Invalid access!");
    return Mem11.Base;
  }

  const MCExpr *getMemOff16() const {
    assert((Kind == k_Memory16) && "Invalid access!");
    return Mem16.Off;
  }

  const MCExpr *getMemOff11() const {
    assert((Kind == k_Memory16) && "Invalid access!");
    return Mem16.Off;
  }

  static std::unique_ptr<LaserOperand> CreateToken(StringRef Str, SMLoc S) {
    auto Op = make_unique<LaserOperand>(k_Token);
    Op->Tok.Data = Str.data();
    Op->Tok.Length = Str.size();
    Op->StartLoc = S;
    Op->EndLoc = S;
    return Op;
  }

  /// Internal constructor for register kinds
  static std::unique_ptr<LaserOperand> CreateReg(unsigned RegNum, SMLoc S, 
                                                SMLoc E) {
    auto Op = make_unique<LaserOperand>(k_Register);
    Op->Reg.RegNum = RegNum;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<LaserOperand> CreateImm(const MCExpr *Val, SMLoc S, SMLoc E) {
    auto Op = make_unique<LaserOperand>(k_Immediate);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<LaserOperand> CreateMem16(unsigned Base, const MCExpr *Off,
                                 SMLoc S, SMLoc E) {
    auto Op = make_unique<LaserOperand>(k_Memory16);
    Op->Mem16.Base = Base;
    Op->Mem16.Off = Off;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<LaserOperand> CreateMem11(unsigned Base, const MCExpr *Off,
                                 SMLoc S, SMLoc E) {
    auto Op = make_unique<LaserOperand>(k_Memory11);
    Op->Mem11.Base = Base;
    Op->Mem11.Off = Off;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  /// getStartLoc - Get the location of the first token of this operand.
  SMLoc getStartLoc() const { return StartLoc; }
  /// getEndLoc - Get the location of the last token of this operand.
  SMLoc getEndLoc() const { return EndLoc; }

};
}

//@1 {
bool LaserAsmParser::needsExpansion(MCInst &Inst) {

  // switch(Inst.getOpcode()) {
  //   case LASER::LoadImm32Reg:
  //   case LASER::LoadAddr32Imm:
  //   case LASER::LoadAddr32Reg:
  //     return true;
  //   default:
  //     return false;
  // }
  return false;
}

void LaserAsmParser::expandInstruction(MCInst &Inst, SMLoc IDLoc,
                        SmallVectorImpl<MCInst> &Instructions){
  // switch(Inst.getOpcode()) {
  //   case LASER::LoadImm32Reg:
  //     return expandLoadImm(Inst, IDLoc, Instructions);
  //   case LASER::LoadAddr32Imm:
  //     return expandLoadAddressImm(Inst,IDLoc,Instructions);
  //   case LASER::LoadAddr32Reg:
  //     return expandLoadAddressReg(Inst,IDLoc,Instructions);
  //   }
}
//@1 }

void LaserAsmParser::expandLoadImm(MCInst &Inst, SMLoc IDLoc,
                                  SmallVectorImpl<MCInst> &Instructions){
  MCInst tmpInst;
  const MCOperand &ImmOp = Inst.getOperand(1);
  assert(ImmOp.isImm() && "expected immediate operand kind");
  const MCOperand &RegOp = Inst.getOperand(0);
  assert(RegOp.isReg() && "expected register operand kind");

  int ImmValue = ImmOp.getImm();
  tmpInst.setLoc(IDLoc);
  if ( 0 <= ImmValue && ImmValue <= 65535) {
    // for 0 <= j <= 65535.
    // li d,j => ori d,$zero,j
    tmpInst.setOpcode(LASER::IMD);
    tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
    tmpInst.addOperand(MCOperand::createImm(ImmValue));
    Instructions.push_back(tmpInst);
  } // else if ( ImmValue < 0 && ImmValue >= -32768) {
  //   // for -32768 <= j < 0.
  //   // li d,j => addiu d,$zero,j
  //   tmpInst.setOpcode(LASER::ADDiu); //TODO:no ADDiu64 in td files?
  //   tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
  //   tmpInst.addOperand(
  //             MCOperand::createReg(LASER::ZERO));
  //   tmpInst.addOperand(MCOperand::createImm(ImmValue));
  //   Instructions.push_back(tmpInst);
  // } else {
  //   // for any other value of j that is representable as a 32-bit integer.
  //   // li d,j => lui d,hi16(j)
  //   //           ori d,d,lo16(j)
  //   tmpInst.setOpcode(LASER::LUi);
  //   tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createImm((ImmValue & 0xffff0000) >> 16));
  //   Instructions.push_back(tmpInst);
  //   tmpInst.clear();
  //   tmpInst.setOpcode(LASER::ORi);
  //   tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createImm(ImmValue & 0xffff));
  //   tmpInst.setLoc(IDLoc);
  //   Instructions.push_back(tmpInst);
  // }
}

void LaserAsmParser::expandLoadAddressReg(MCInst &Inst, SMLoc IDLoc,
                                         SmallVectorImpl<MCInst> &Instructions){
  // MCInst tmpInst;
  // const MCOperand &ImmOp = Inst.getOperand(2);
  // assert(ImmOp.isImm() && "expected immediate operand kind");
  // const MCOperand &SrcRegOp = Inst.getOperand(1);
  // assert(SrcRegOp.isReg() && "expected register operand kind");
  // const MCOperand &DstRegOp = Inst.getOperand(0);
  // assert(DstRegOp.isReg() && "expected register operand kind");
  // int ImmValue = ImmOp.getImm();
  // if ( -32768 <= ImmValue && ImmValue <= 32767) {
  //   // for -32768 <= j < 32767.
  //   //la d,j(s) => addiu d,s,j
  //   tmpInst.setOpcode(LASER::ADDiu); //TODO:no ADDiu64 in td files?
  //   tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createReg(SrcRegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createImm(ImmValue));
  //   Instructions.push_back(tmpInst);
  // } else {
  //   // for any other value of j that is representable as a 32-bit integer.
  //   // la d,j(s) => lui d,hi16(j)
  //   //              ori d,d,lo16(j)
  //   //              add d,d,s
  //   tmpInst.setOpcode(LASER::LUi);
  //   tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createImm((ImmValue & 0xffff0000) >> 16));
  //   Instructions.push_back(tmpInst);
  //   tmpInst.clear();
  //   tmpInst.setOpcode(LASER::ORi);
  //   tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createImm(ImmValue & 0xffff));
  //   Instructions.push_back(tmpInst);
  //   tmpInst.clear();
  //   tmpInst.setOpcode(LASER::ADD);
  //   tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createReg(DstRegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createReg(SrcRegOp.getReg()));
  //   Instructions.push_back(tmpInst);
  // }
}

void LaserAsmParser::expandLoadAddressImm(MCInst &Inst, SMLoc IDLoc,
                                         SmallVectorImpl<MCInst> &Instructions){
  // MCInst tmpInst;
  // const MCOperand &ImmOp = Inst.getOperand(1);
  // assert(ImmOp.isImm() && "expected immediate operand kind");
  // const MCOperand &RegOp = Inst.getOperand(0);
  // assert(RegOp.isReg() && "expected register operand kind");
  // int ImmValue = ImmOp.getImm();
  // if ( -32768 <= ImmValue && ImmValue <= 32767) {
  //   // for -32768 <= j < 32767.
  //   //la d,j => addiu d,$zero,j
  //   tmpInst.setOpcode(LASER::ADDiu);
  //   tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
  //   tmpInst.addOperand(
  //             MCOperand::createReg(LASER::ZERO));
  //   tmpInst.addOperand(MCOperand::createImm(ImmValue));
  //   Instructions.push_back(tmpInst);
  // } else {
  //   // for any other value of j that is representable as a 32-bit integer.
  //   // la d,j => lui d,hi16(j)
  //   //           ori d,d,lo16(j)
  //   tmpInst.setOpcode(LASER::LUi);
  //   tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createImm((ImmValue & 0xffff0000) >> 16));
  //   Instructions.push_back(tmpInst);
  //   tmpInst.clear();
  //   tmpInst.setOpcode(LASER::ORi);
  //   tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createReg(RegOp.getReg()));
  //   tmpInst.addOperand(MCOperand::createImm(ImmValue & 0xffff));
  //   Instructions.push_back(tmpInst);
  // }
}

//@2 {
bool LaserAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                            OperandVector &Operands,
                                            MCStreamer &Out,
                                            uint64_t &ErrorInfo,
                                            bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult = MatchInstructionImpl(Operands, Inst, ErrorInfo,
                                              MatchingInlineAsm);
  switch (MatchResult) {
  default: break;
  case Match_Success: {
    if (needsExpansion(Inst)) {
      SmallVector<MCInst, 4> Instructions;
      expandInstruction(Inst, IDLoc, Instructions);
      for(unsigned i =0; i < Instructions.size(); i++){
        Out.EmitInstruction(Instructions[i], getSTI());
      }
    } else {
        Inst.setLoc(IDLoc);
        Out.EmitInstruction(Inst, getSTI());
      }
    return false;
  }
//@2 }
  case Match_MissingFeature:
    Error(IDLoc, "instruction requires a CPU feature not currently enabled");
    return true;
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(IDLoc, "too few operands for instruction");

      ErrorLoc = ((LaserOperand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc()) ErrorLoc = IDLoc;
    }

    return Error(ErrorLoc, "invalid operand for instruction");
  }
  case Match_MnemonicFail:
    return Error(IDLoc, "invalid instruction");
  }
  return true;
}

int LaserAsmParser::matchRegisterName(StringRef Name) {

   int CC;
    CC = StringSwitch<unsigned>(Name)
      .Case("r8",  LASER::R8)
      .Case("r9",  LASER::R9)
      .Case("r10",  LASER::R10)
      .Case("r11",  LASER::R11)
      .Case("r12",  LASER::R12)
      .Case("r13",  LASER::R13)
      .Case("r14",  LASER::R14)
      .Case("r15",  LASER::R15)
      .Case("flagr",  LASER::FLAGR)
      .Case("sp",  LASER::SP)
      .Case("fp",  LASER::FP)
      .Case("ss",  LASER::SS)
      .Case("lr",  LASER::LR)
      .Case("redaddr",  LASER::RETADDR)
      .Case("gp",  LASER::GP)
      .Case("retval",  LASER::RETVAL)
      .Default(-1);

  if (CC != -1)
    return CC;

  return -1;
}

unsigned LaserAsmParser::getReg(int RC,int RegNo) {
  return *(getContext().getRegisterInfo()->getRegClass(RC).begin() + RegNo);
}

int LaserAsmParser::matchRegisterByNumber(unsigned RegNum, StringRef Mnemonic) {
  if (RegNum > 15)
    return -1;

  return getReg(LASER::CPURegsRegClassID, RegNum);
}

int LaserAsmParser::tryParseRegister(StringRef Mnemonic) {
  const AsmToken &Tok = Parser.getTok();
  int RegNum = -1;

  if (Tok.is(AsmToken::Identifier)) {
    std::string lowerCase = Tok.getString().lower();
    RegNum = matchRegisterName(lowerCase);
  } else if (Tok.is(AsmToken::Integer))
    RegNum = matchRegisterByNumber(static_cast<unsigned>(Tok.getIntVal()),
                                   Mnemonic.lower());
    else
      return RegNum;  //error
  return RegNum;
}

bool LaserAsmParser::
  tryParseRegisterOperand(OperandVector &Operands,
                          StringRef Mnemonic){

  SMLoc S = Parser.getTok().getLoc();
  int RegNo = -1;

    RegNo = tryParseRegister(Mnemonic);
  if (RegNo == -1)
    return true;

  Operands.push_back(LaserOperand::CreateReg(RegNo, S,
      Parser.getTok().getLoc()));
  Parser.Lex(); // Eat register token.
  return false;
}

bool LaserAsmParser::ParseOperand(OperandVector &Operands,
				  StringRef Mnemonic) {
  DEBUG(dbgs() << "ParseOperand\n");
  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  OperandMatchResultTy ResTy = MatchOperandParserImpl(Operands, Mnemonic);
  if (ResTy == MatchOperand_Success)
    return false;
  // If there wasn't a custom match, try the generic matcher below. Otherwise,
  // there was a match, but an error occurred, in which case, just return that
  // the operand parsing failed.
  if (ResTy == MatchOperand_ParseFail)
    return true;

  DEBUG(dbgs() << ".. Generic Parser\n");

  switch (getLexer().getKind()) {
  default:
    Error(Parser.getTok().getLoc(), "unexpected token in operand");
    return true;
  case AsmToken::Percent: {
    // parse register
    SMLoc S = Parser.getTok().getLoc();
    Parser.Lex(); // Eat Percent token.
    if (!tryParseRegisterOperand(Operands, Mnemonic)) {
      S = Parser.getTok().getLoc();
    }
    return false;
  }
  case AsmToken::Amp: {
    // parse memory operand
    const MCExpr *IdVal;
    SMLoc S = Parser.getTok().getLoc();
    if (getParser().parseExpression(IdVal))
      return true;
    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    Operands.push_back(LaserOperand::CreateImm(IdVal, S, E));
    return false;
  }
  case AsmToken::Identifier:
  case AsmToken::LParen:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Integer:
  case AsmToken::String: {
    // quoted label names
    const MCExpr *IdVal;
    SMLoc S = Parser.getTok().getLoc();
    if (getParser().parseExpression(IdVal))
      return true;
    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    Operands.push_back(LaserOperand::CreateImm(IdVal, S, E));
    return false;
  }
  case AsmToken::Hash: {
    // Constant operand
    Operands.push_back(LaserOperand::CreateToken("#",
                                                 Parser.getTok().getLoc()));

    Parser.Lex(); // Eat Percent token.
    const MCExpr *ImmVal;
    SMLoc Start = Parser.getTok().getLoc(); // start location of the operand

    if (getLexer().is(AsmToken::Integer)) {
      int64_t IntVal = getTok().getIntVal();
      SMLoc End = Parser.getTok().getLoc();
      ImmVal = MCConstantExpr::create(IntVal, getContext()); 
      Operands.push_back(LaserOperand::CreateImm(ImmVal, Start, End));
      Parser.Lex(); // consume token
      return false;
    }
    else 
      return true;
  } // case AsmToken::Hash
  } // switch(getLexer().getKind())
  return true;
}

const MCExpr *LaserAsmParser::evaluateRelocExpr(const MCExpr *Expr,
						StringRef RelocStr) {
  LaserMCExpr::VariantKind Kind =
    StringSwitch<LaserMCExpr::VariantKind>(RelocStr)
    .Case("call16", LaserMCExpr::VK_LASER_CALL16)
    .Case("jmp11", LaserMCExpr::VK_LASER_PC11)
    .Case("gv16", LaserMCExpr::VK_LASER_GV16)
    .Default(LaserMCExpr::VK_LASER_NONE);

  assert(Kind != LaserMCExpr::VK_LASER_NONE);
  return LaserMCExpr::create(Kind, Expr, getContext());
}

bool LaserAsmParser::parseRelocOperand(const MCExpr *&Res) {

  Parser.Lex(); // eat @ token
  const AsmToken &Tok = Parser.getTok(); // get next token, operation
  if (Tok.isNot(AsmToken::Identifier))
    return true;

  std::string Str = Tok.getIdentifier().str();

  Parser.Lex(); // eat identifier
  // now make expression from the rest of the operand
  const MCExpr *IdVal;

  if (getLexer().getKind() == AsmToken::LBrac) {
      Parser.Lex(); // eat ']' token
      const AsmToken &nextTok = Parser.getTok();
      if (nextTok.isNot(AsmToken::Identifier))
	return true;

      if (getParser().parseExpression(IdVal))
      return true;

      if (getLexer().getKind() == AsmToken::RBrac)
      	Parser.Lex(); // eat '[' token
      else return true;


  } else
    return true; // brackets must follow reloc operand

  Res = evaluateRelocExpr(IdVal, Str);
  return false;
}

bool LaserAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                  SMLoc &EndLoc) {

  StartLoc = Parser.getTok().getLoc();
  RegNo = tryParseRegister("");
  EndLoc = Parser.getTok().getLoc();
  return (RegNo == (unsigned)-1);
}

bool LaserAsmParser::parseMemOffset(const MCExpr *&Res) {

  SMLoc S;

  switch(getLexer().getKind()) {
  default:
    return true;
  case AsmToken::Integer:
  case AsmToken::Minus:
  case AsmToken::Plus:
    return (getParser().parseExpression(Res));
  case AsmToken::Percent:
    return parseRelocOperand(Res);
  case AsmToken::LParen:
    return false;  // it's probably assuming 0
  }
  return true;
}

OperandMatchResultTy LaserAsmParser::parseImmOperand(
               OperandVector &Operands) {

  if(getLexer().getKind() == AsmToken::Hash) {
    Parser.Lex(); // Eat Percent token.
    const MCExpr *ImmVal = 0;
    SMLoc Start = Parser.getTok().getLoc(); // start location of the operand

    if (getLexer().is(AsmToken::Integer)) {
      int64_t IntVal = getTok().getIntVal();
      SMLoc End = Parser.getTok().getLoc();
      ImmVal = MCConstantExpr::create(IntVal, getContext()); 
      Operands.push_back(LaserOperand::CreateImm(ImmVal, Start, End));
      Parser.Lex(); // consume token
      return MatchOperand_Success;
    }
    return MatchOperand_ParseFail;
  }
  else if (getLexer().getKind() == AsmToken::At) {
    SMLoc Start = Parser.getTok().getLoc(); // start location of the operand
    // it is a symbol reference or constant expression
    const MCExpr *IdVal;
    if (parseRelocOperand(IdVal))
      return MatchOperand_ParseFail;
    else {
      SMLoc End = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
      Operands.push_back(LaserOperand::CreateImm(IdVal, Start, End));
      return MatchOperand_Success;
    }
  }
  else
    return MatchOperand_ParseFail;
  
}

// eg, [%r9]
OperandMatchResultTy 
LaserAsmParser::parseMem16Operand(OperandVector &Operands) {
  const MCExpr *IdVal = 0;

  SMLoc Start = Parser.getTok().getLoc(); // start location of the operand
  int RegNo = -1;

  const AsmToken &Tok = Parser.getTok(); // get next token
  if (Tok.isNot(AsmToken::LBrac)) {
    Error(Parser.getTok().getLoc(), "'[' expected");
    return MatchOperand_ParseFail;
  }

  Parser.Lex(); // Eat '[' token.

  const AsmToken &Tok1 = Parser.getTok(); // get next token
  if (Tok1.is(AsmToken::Percent)) {
    Parser.Lex(); // Eat '%' token.

    // SMLoc S = Parser.getTok().getLoc();

    //RegNo = tryParseRegister(Mnemonic);

    const AsmToken &Tok = Parser.getTok();

    if (Tok.is(AsmToken::Identifier)) {
      std::string lowerCase = Tok.getString().lower();
      RegNo = matchRegisterName(lowerCase);
    } 
    // else if (Tok.is(AsmToken::Integer))
    //   RegNo = matchRegisterByNumber(static_cast<unsigned>(Tok.getIntVal()),
    // 				     Mnemonic.lower());

    if (RegNo == -1)
      return MatchOperand_ParseFail;

    Parser.Lex(); // Eat register token.

    // if (!tryParseRegisterOperand(Operands, Mnemonic)) {
    //   Start = Parser.getTok().getLoc();
    // }
    // else {
    //   Error(Parser.getTok().getLoc(), "unexpected token in operand");
    //   return MatchOperand_ParseFail;
    // }
  }
  else 
    return MatchOperand_ParseFail;

  const AsmToken &Tok2 = Parser.getTok(); // get next token
  if (Tok2.isNot(AsmToken::RBrac)) {
    Error(Parser.getTok().getLoc(), "']' expected");
    return MatchOperand_ParseFail;
  }

  Parser.Lex(); // Eat ']' token.

  SMLoc End = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer());
  Operands.push_back(LaserOperand::CreateMem16(RegNo, IdVal, Start, End));

  // // Replace the register operand with the memory operand.
  // std::unique_ptr<LaserOperand> op(
  //     static_cast<LaserOperand *>(Operands.back().release()));
  // int RegNo = op->getReg();
  // // remove register from operands
  // Operands.pop_back();
  // // and add memory operand
  // Operands.push_back(LaserOperand::CreateMem(RegNo, IdVal, Start, End));
  return MatchOperand_Success;
}

// eg, [@BB1_1]
OperandMatchResultTy 
LaserAsmParser::parseLASERjmptarget11Operand(OperandVector &Operands) {
  const MCExpr *IdVal = 0;

  SMLoc Start = Parser.getTok().getLoc(); // start location of the operand
  int RegNo = -1;

  const AsmToken &Tok = Parser.getTok(); // get next token
  if (Tok.isNot(AsmToken::LBrac)) {
    Error(Parser.getTok().getLoc(), "'[' expected");
    return MatchOperand_ParseFail;
  }

  Parser.Lex(); // Eat '[' token.

  const AsmToken &Tok1 = Parser.getTok(); // get next token
  if (Tok1.is(AsmToken::Percent)) {
    Parser.Lex(); // Eat '%' token.

    // SMLoc S = Parser.getTok().getLoc();

    //RegNo = tryParseRegister(Mnemonic);

    const AsmToken &Tok = Parser.getTok();

    if (Tok.is(AsmToken::Identifier)) {
      std::string lowerCase = Tok.getString().lower();
      RegNo = matchRegisterName(lowerCase);
    } 
    // else if (Tok.is(AsmToken::Integer))
    //   RegNo = matchRegisterByNumber(static_cast<unsigned>(Tok.getIntVal()),
    // 				     Mnemonic.lower());

    if (RegNo == -1)
      return MatchOperand_ParseFail;

    Parser.Lex(); // Eat register token.

    // if (!tryParseRegisterOperand(Operands, Mnemonic)) {
    //   Start = Parser.getTok().getLoc();
    // }
    // else {
    //   Error(Parser.getTok().getLoc(), "unexpected token in operand");
    //   return MatchOperand_ParseFail;
    // }
  }
  else 
    return MatchOperand_ParseFail;

  const AsmToken &Tok2 = Parser.getTok(); // get next token
  if (Tok2.isNot(AsmToken::RBrac)) {
    Error(Parser.getTok().getLoc(), "']' expected");
    return MatchOperand_ParseFail;
  }

  Parser.Lex(); // Eat ']' token.

  SMLoc End = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer());
  Operands.push_back(LaserOperand::CreateMem11(RegNo, IdVal, Start, End));

  // // Replace the register operand with the memory operand.
  // std::unique_ptr<LaserOperand> op(
  //     static_cast<LaserOperand *>(Operands.back().release()));
  // int RegNo = op->getReg();
  // // remove register from operands
  // Operands.pop_back();
  // // and add memory operand
  // Operands.push_back(LaserOperand::CreateMem(RegNo, IdVal, Start, End));
  return MatchOperand_Success;
}


bool LaserAsmParser::
parseMathOperation(StringRef Name, SMLoc NameLoc,
                   OperandVector &Operands) {
  // split the format
  size_t Start = Name.find('.'), Next = Name.rfind('.');
  StringRef Format1 = Name.slice(Start, Next);
  // and add the first format to the operands
  Operands.push_back(LaserOperand::CreateToken(Format1, NameLoc));
  // now for the second format
  StringRef Format2 = Name.slice(Next, StringRef::npos);
  Operands.push_back(LaserOperand::CreateToken(Format2, NameLoc));

  // set the format for the first register
//  setFpFormat(Format1);

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    if (ParseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }

    if (getLexer().isNot(AsmToken::Comma)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");

    }
    Parser.Lex();  // Eat the comma.

    // Parse and remember the operand.
    if (ParseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    Parser.eatToEndOfStatement();
    return Error(Loc, "unexpected token in argument list");
  }

  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool LaserAsmParser::
ParseInstruction(ParseInstructionInfo &Info, StringRef Name, SMLoc NameLoc,
                 OperandVector &Operands) {

  // Create the leading tokens for the mnemonic, split by '.' characters.
  //size_t Start = 0, Next = Name.find('.');
  //StringRef Mnemonic = Name.slice(Start, Next);

  // First operand in MCInst is instruction mnemonic.
  Operands.push_back(LaserOperand::CreateToken(Name, NameLoc));
  //Operands.push_back(LaserOperand::CreateToken(Mnemonic, NameLoc));

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.n
    if (ParseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }

    while (getLexer().is(AsmToken::Comma) ) {
      Parser.Lex();  // Eat the comma.

      // Parse and remember the operand.
      if (ParseOperand(Operands, Name)) {
        SMLoc Loc = getLexer().getLoc();
        Parser.eatToEndOfStatement();
        return Error(Loc, "unexpected token in argument list");
      }
    }
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    Parser.eatToEndOfStatement();
    return Error(Loc, "unexpected token in argument list");
  }

  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool LaserAsmParser::reportParseError(StringRef ErrorMsg) {
   SMLoc Loc = getLexer().getLoc();
   Parser.eatToEndOfStatement();
   return Error(Loc, ErrorMsg);
}

bool LaserAsmParser::parseSetReorderDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token in statement");
    return false;
  }
  Options.setReorder();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool LaserAsmParser::parseSetNoReorderDirective() {
    Parser.Lex();
    // if this is not the end of the statement, report error
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      reportParseError("unexpected token in statement");
      return false;
    }
    Options.setNoreorder();
    Parser.Lex(); // Consume the EndOfStatement
    return false;
}

bool LaserAsmParser::parseSetMacroDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("unexpected token in statement");
    return false;
  }
  Options.setMacro();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool LaserAsmParser::parseSetNoMacroDirective() {
  Parser.Lex();
  // if this is not the end of the statement, report error
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    reportParseError("`noreorder' must be set before `nomacro'");
    return false;
  }
  if (Options.isReorder()) {
    reportParseError("`noreorder' must be set before `nomacro'");
    return false;
  }
  Options.setNomacro();
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}
bool LaserAsmParser::parseDirectiveSet() {

  // get next token
  const AsmToken &Tok = Parser.getTok();

  if (Tok.getString() == "reorder") {
    return parseSetReorderDirective();
  } else if (Tok.getString() == "noreorder") {
    return parseSetNoReorderDirective();
  } else if (Tok.getString() == "macro") {
    return parseSetMacroDirective();
  } else if (Tok.getString() == "nomacro") {
    return parseSetNoMacroDirective();
  }
  return true;
}

bool LaserAsmParser::ParseDirective(AsmToken DirectiveID) {

  if (DirectiveID.getString() == ".ent") {
    // ignore this directive for now
    Parser.Lex();
    return false;
  }

  if (DirectiveID.getString() == ".end") {
    // ignore this directive for now
    Parser.Lex();
    return false;
  }

  if (DirectiveID.getString() == ".frame") {
    // ignore this directive for now
    Parser.eatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".set") {
    return parseDirectiveSet();
  }

  if (DirectiveID.getString() == ".fmask") {
    // ignore this directive for now
    Parser.eatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".mask") {
    // ignore this directive for now
    Parser.eatToEndOfStatement();
    return false;
  }

  if (DirectiveID.getString() == ".gpword") {
    // ignore this directive for now
    Parser.eatToEndOfStatement();
    return false;
  }

  return true;
}

extern "C" void LLVMInitializeLaserAsmParser() {
  RegisterMCAsmParser<LaserAsmParser> X(getTheLaserTarget());
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "LaserGenAsmMatcher.inc"

