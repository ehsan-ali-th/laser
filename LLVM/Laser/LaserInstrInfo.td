//===-- LaserInstrInfo.td - Target Description for Laser Target -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the Laser instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

include "LaserInstrFormats.td"

// Instruction operand types

def LaserMemAsmOperand : AsmOperandClass {
  let Name = "Mem16";
  let ParserMethod = "parseMem16Operand";
}

def LaserImmAsmOperand : AsmOperandClass {
  let Name = "LaserImm";
  let ParserMethod = "parseImmOperand";
}

// Unsigned Operand
// class Operand<ValueType ty> : DAGOperand { ... } is defined in target.td
def uimm16      : Operand<i16> {
  // let PrintMethod = "printUnsignedImm"; 
}    

// let EncoderMethod = "getImmSrcValue" in {
// def LASERimm16 : Operand<i16>, ImmLeaf<i16, [{
// return Imm >= 0 && Imm < 65536;
// }]>;
// }

def LASERimm16 : Operand<i16> { 
    let ParserMatchClass = LaserImmAsmOperand;
    let PrintMethod = "printImm"; 
    //let EncoderMethod = "getImmSrcValue";
    let DecoderMethod= "DecodeLASERimm16";
}

def LASERjmptarget11AsmOperand : AsmOperandClass {
  let Name = "LASERjmptarget11";
  let ParserMethod = "parseLASERjmptarget11Operand"; 
}


def LASERjmptarget11 : Operand<OtherVT> {
    let ParserMatchClass = LASERjmptarget11AsmOperand;
    let PrintMethod = "printjmptarget11"; 
    let EncoderMethod = "getJumpTarget11OpValue";
    let OperandType = "OPERAND_PCREL";
 }

 //def LASERchain : Operand<OtherVT>;

 // Signed Operand
// def LASERsimm16 : Operand<i16>, ImmLeaf<i16, [{
// return Imm >= 0 && Imm < 65536;
// }]>;

def LASERimmSExt16 : PatLeaf<(imm), [{ return isInt<16>(N->getSExtValue()); }]>;

// Node immediate fits as 16-bit sign extended on target immediate.
//def LASERimmSExt16  : PatLeaf<(imm), [{ return isInt<16>(N->getSExtValue()); }]>;

def memsrc :Operand<iPTR> {
  let MIOperandInfo = (ops GNPRegs, LASERimm16);
  //let MIOperandInfo = (ops GNPRegs);
  let PrintMethod = "printAddrModeMemSrc";
  //let EncoderMethod = "getMemSrcValue";
  let ParserMatchClass = LaserMemAsmOperand;
  let DecoderMethod= "DecodeLASERmemsrc";
}

// Return
 def LaserRet : SDNode<"LASERISD::Ret", SDTNone,
     [SDNPHasChain, SDNPOptInGlue,      SDNPVariadic]>;

let isReturn=1, isTerminator=1, hasDelaySlot=1, isBarrier=1, hasCtrlDep=1 in {
    def RET_FLAG : LaserPseudo<(outs), (ins), "", [(LaserRet)]>;
}

// JUMP Pseudo instructions to support SETGE , etc..
def SDT_LaserJump : SDTypeProfile<
              0, // NumResults 
              3, // NumOperands
              [SDTCisVT<0, OtherVT>]>;

def SDT_LaserJumpSet : SDTypeProfile<
              0, // NumResults 
              2, // NumOperands
              [SDTCisVT<0, OtherVT>]>;


def LaserJumpSetge : SDNode<"LASERISD::JumpSetge", SDT_LaserJumpSet,
     [SDNPHasChain, SDNPVariadic]>;

let isBranch = 1, isTerminator = 1, isBarrier = 0, hasDelaySlot = 0, Uses = [FLAGR] in {
    def JumpSetge_FLAG : LaserPseudo<(outs), (ins LASERjmptarget11:$address), "",
         [(LaserJumpSetge bb:$address, FLAGR)]>;
}

def SDT_LaserDivRemP : SDTypeProfile<
              1, // NumResults 
              2, // NumOperands
              [SDTCisSameAs<0, 1>, SDTCisSameAs<0, 2>, SDTCisInt<0>]>;

def LaserDivRemP : SDNode<"LASERISD:DivRem", SDT_LaserDivRemP>;

def DivRemP_FLAG : LaserPseudo<(outs GNPRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), "",
         [(set i16:$rt, (sdiv i16:$rs, i16:$rd))]>;

// ICmp
def SDT_LaserICmp : SDTypeProfile<
              0, // NumResults 
              2, // NumOperands
              [SDTCisInt<0>, SDTCisSameAs<1, 0>]>;

def LaserICmp : SDNode<"LASERISD::ICmp", SDT_LaserICmp,
    []>;

let Defs = [FLAGR] in {
def ICmp_FLAG : LaserPseudo<(outs), (ins GNPRegs:$rs, GNPRegs:$rd), "", 
    // [(set (i16 FLAGR), LaserICmp i16:$rs, i16:$rd))]>;
    [(LaserICmp i16:$rs, i16:$rd)]>;
}

// Jump
def LaserJump : SDNode<"LASERISD::Jump", SDT_LaserJump,
     [SDNPInGlue, SDNPHasChain, SDNPVariadic]>;

// def Laser_FLAG : LaserPseudo<(outs), (ins LASERimm11op:$address), "", [(LaserJump bb:$address)]>
// {
// let isBranch = 1;          
// let isTerminator = 1;   
// let isBarrier = 1;
// let hasDelaySlot = 0;
// }

def SDT_LaserRet : SDTypeProfile<0, 1, [SDTCisInt<0>]>;

def SDT_LaserDivRem : SDTypeProfile<
    0,  // NumResults 
    2,  // NumOperands 
    [SDTCisInt<0>, SDTCisSameAs<0, 1>] // list<SDTypeConstraint>
    >;

def LASERDivRem : SDNode<"LASERISD::DivRem", SDT_LaserDivRem, [SDNPOutGlue]>;

// JMP
// def jmptarget    : Operand<OtherVT> {
//   let EncoderMethod = "getJumpTargetOpValue";
//   let OperandType = "OPERAND_PCREL";
// }

// Addressing modes.
// def addr : ComplexPattern<
//                 iPTR,                // ValueType ty
//                 2,                   // int numops
//                 "SelectAddr",        // string fn
//                 [frameindex],        // list<SDNode> roots = []
//                 [SDNPWantParent]     // list<SDNodeProperty> props = []
//                 >;

def addr : ComplexPattern<
                iPTR,                // ValueType ty
                2,                   // int numops
                "SelectAddr",        // string fn
                [frameindex],        // list<SDNode> roots = []
                [SDNPWantParent]                   // list<SDNodeProperty> props = []
                >;

def SDTLaserWrapper : SDTypeProfile<1, 1, [SDTCisPtrTy<0>,
    SDTCisSameAs<1, 0>]>;

def LaserWrapper : SDNode<"LASERISD::Wrapper", SDTLaserWrapper>;

def load_sym : SDNode<"LASERISD::LOAD_SYM", SDTIntUnaryOp>;

// Patterns:

//def : Pat<(brcond (i32 (setne RC:$lhs, RC:$rhs)), bb:$dst), (JNEOp (CMPOp RC:$lhs, RC:$rhs), bb:$dst)>;

// def JNZ : F3 < 0b10010, (outs), (ins LASERimm11op:$rd),
// "jnz [$rd]", [(br bb:$rd)], IIBranch> {
// let isBranch = 1;          
// let isTerminator = 1;   
// let isBarrier = 1;
// let hasDelaySlot = 0;
// }

// def calltarget  : Operand<iPTR> {
//   let EncoderMethod = "getBranch16TargetOpValue";
// }

def SDT_LaserCall : SDTypeProfile<
    0,                  // int numresults
    1,                 // int numoperands,
    [SDTCisVT<0, iPTR>] // list<SDTypeConstraint> constraints>
>;

def LaserCall : SDNode<"LASERISD::LaserCall", SDT_LaserCall,
                         [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue,
                          SDNPVariadic]>;

def SDT_LaserCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i16>]>;
def SDT_LaserCallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;


def LaserGPRel : SDNode<"LASERISD::GPRel", SDTIntUnaryOp>;

// These are target-independent nodes, but have target-specific formats.
def callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_LaserCallSeqStart,
                           [SDNPHasChain, SDNPOutGlue]>;
def callseq_end   : SDNode<"ISD::CALLSEQ_END", SDT_LaserCallSeqEnd,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

// def CALL_FLAG : LaserPseudo<(outs), (ins calltarget:$target, variable_ops),
//                                   "CALL_FLAG \t$target",
//                                   []>;

// def CALL : F3<0b11100, (outs), (ins GNPRegs:$rd),
//      "call \t$rd", [], IIBranch>;

// Jump and Link (Call)
def CALL: F3<0b11100, (outs), (ins GNPRegs:$rd),
             "call \t$rd", [], IIBranch> {
    let isCall=1;
    let  hasDelaySlot=1;
}

def IMD : F_long_1 <0b10111, (outs GNPRegs:$rd), (ins LASERimm16:$val), 
"imd $rd, $val", [(set i16:$rd, LASERimmSExt16:$val)], IIAlu> {
     let isMoveImm = 1;
} 

def : Pat<(LaserCall (i16 tglobaladdr:$dst)),
          (CALL (IMD tglobaladdr:$dst))>;
def : Pat<(LaserCall (i16 texternalsym:$dst)),
          (CALL (IMD texternalsym:$dst))>;


let Defs = [SP], Uses = [SP] in {
def ADJCALLSTACKDOWN : LaserPseudo<(outs), (ins uimm16:$amt1, uimm16:$amt2),
                                  "!ADJCALLSTACKDOWN $amt1",
                                  [(callseq_start timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP   : LaserPseudo<(outs), (ins uimm16:$amt1, uimm16:$amt2),
                                  "!ADJCALLSTACKUP $amt1",
                                  [(callseq_end timm:$amt1, timm:$amt2)]>;
}

// def JSUB    : JumpLink<0b11100, "call">;

def LD : F2 <0b01111, (outs GNPRegs:$rd), (ins memsrc:$rs), 
"ld $rd, $rs", [(set i16:$rd, (load addr:$rs))], IILoad>; 

// Store: The value inside register RS will be put into memory address 
//  saved in register (SP+RD).
// The order of ins argumetn does matter.
def ST : F2 <0b10000, (outs), (ins GNPRegs:$rs, memsrc:$rd), 
"st $rd, $rs", [(store i16:$rs, addr:$rd)], IIStore>; 


// Hi and Lo nodes are used to handle global addresses. Used on
// LaserISeltLowering to lower stuff like GlobalAddress, ExternalSymbol
// static model. (nothing to do with Laser Registers)
// def LaserHi    : SDNode<"LASERISD::Hi", SDTIntUnaryOp>;
// def LaserLo    : SDNode<"LASERISD::Lo", SDTIntUnaryOp>;
def LaserABS_GV    : SDNode<"LASERISD::ABS_GV", SDTIntUnaryOp>;

def : Pat<(LaserABS_GV tglobaladdr:$in), 
          (IMD tglobaladdr:$in)>;


def CMP : F2 < 0b10001, (outs), (ins GNPRegs:$rs, GNPRegs:$rd),
"cmp $rs, $rd", [], IIAlu>; 

def MOV : F2 < 0b00000, (outs GNPRegs:$rd), (ins GNPRegs:$rs),
"mov $rd, $rs", [], IIAlu>; 

def ADD : F1 <0b00001,  (outs RTRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), 
"add $rd, $rs, $rt", [(set i16:$rt, (add i16:$rs, i16:$rd))], IIAlu>; 

// def : Pat<(add CPURegs:$GP, (LaserGPRel tglobaladdr:$in)),
//           (ADD CPURegs:$GP, tglobaladdr:$in)>;


def ADC : F1 <0b00010,  (outs RTRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), 
"adc $rd, $rs, $rt", [(set i16:$rt, (add i16:$rs, i16:$rd))], IIAlu>{
let Uses = [FLAGR];
}


def INC : F3 <0b00101,  (outs GNPRegs:$rt_out), (ins GNPRegs:$rt), 
"inc $rt", [(set i16:$rt_out, (add i16:$rt, 1))], IIAlu> {
      let Constraints = "$rt_out = $rt";
}

def DEC : F3 <0b00110,  (outs GNPRegs:$rt_out), (ins GNPRegs:$rt), 
"dec $rt", [(set i16:$rt_out, (sub i16:$rt, 1))], IIAlu> {
      let Constraints = "$rt_out = $rt";
}

def AND : F1 <0b01001,  (outs RTRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), 
"and $rd, $rs, $rt", [(set i16:$rt, (and i16:$rs, i16:$rd))], IIAlu>; 

def OR : F1 <0b01010,  (outs RTRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), 
"or $rd, $rs, $rt", [(set i16:$rt, (or i16:$rs, i16:$rd))], IIAlu>; 

def XOR : F1 <0b01011,  (outs RTRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), 
"xor $rd, $rs, $rt", [(set i16:$rt, (xor i16:$rs, i16:$rd))], IIAlu>; 

def NOT : F2 <0b01100,  (outs GNPRegs:$rd), (ins GNPRegs:$rs), 
"not $rd, $rs", [(set i16:$rd, (not i16:$rs))], IIAlu>; 

def SRL : F2 <0b01101,  (outs GNPRegs:$rd), (ins GNPRegs:$rs), 
"srl $rd, $rs", [], IIAlu>; 

def SLL : F2 <0b01110,  (outs GNPRegs:$rd), (ins GNPRegs:$rs), 
"sll $rd, $rs", [], IIAlu>; 

// def SRL : F1 <0b01101,  (outs GNPRegs:$rd), (ins GNPRegs:$rs), 
// "srl $rd, $rs", [(set i16:$rd, (srl i16:$rs))], IIAlu>; 

// def LD : F2 <0b01111, (outs GNPRegs:$rd), (ins memsrc:$rs), 
// "ld $rd, [$rs]", [(set i16:$rd, (load addr:$rs))], IILoad>; 

// // Store: The value inside register RS will be put into memory address 
// //  saved in register (SP+RD).
// def ST : F2 <0b10000, (outs), (ins memsrc:$rd, GNPRegs:$rs), 
// "st [$rd], $rs", [(store i16:$rs, addr:$rd)], IIStore>; 

def IN : F2 <0b11000,  (outs GNPRegs:$rd), (ins GNPRegs:$rs), 
"in $rd, $rs", [], IILoad>; 

def OUT : F2 <0b11001,  (outs GNPRegs:$rs), (ins GNPRegs:$rd), 
"out $rd, $rs", [], IIStore>; 

def CLRC : F4 <0b10101, (outs), (ins), 
"clrc", [], IIAlu>; 

def SETC : F4 <0b10110, (outs), (ins), 
"setc", [], IIAlu>; 

// def : Pat <(brcc SETNE, (i16 GNPRegs:$rs), (i16 GNPRegs:$rd), (bb:$T) ),
// (ADD $rs, $rd)
// //((CMP $rs, $rd), (JNZ $T))
// >;

//===----------------------------------------------------------------------===//
// Feature predicates.
//===----------------------------------------------------------------------===//

//def MOV : F2 <0b00000, (outs GNPRegs:$rd), (ins GNPRegs:$rs), 
//"mov $rd, $rs", [(set GNPRegs:$rd, GNPRegs:$rs)], IIAlu>; 


//def ADDCrr : F1 <0b00010,  (outs GNPRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), 
//"adc $rd, $rs, $rt", [(set i16:$rt, (addc i16:$rs, i16:$rd))], IIAlu>; 

def SUB : F1 <0b00011,  (outs RTRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), 
"sub $rd, $rs, $rt", [(set i16:$rt, (sub i16:$rs, i16:$rd))], IIAlu>; 

def SBC : F1 <0b00100,  (outs RTRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), 
"sbc $rd, $rs, $rt", [(set i16:$rt, (sub i16:$rs, i16:$rd))], IIAlu> {
let Uses = [FLAGR];
}


//def SUBCrr : F1 <0b00100,  (outs GNPRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd), 
//"sbc $rd, $rs, $rt", [(set i16:$rt, (subc i16:$rs, i16:Las))], IIAlu>; 

def MUL : F1 <0b00111, (outs RTRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd),
"mul $rd, $rs, $rt", [(set i16:$rt, (mul i16:$rs, i16:$rd))], IIImul> {
let isCommutable = 1;
let isReMaterializable = 1;
}

// Signed DIV
//let Constraints = "$rt_out = $rt" in
def DIV : F1 <0b01000, (outs RTRegs:$rt), (ins GNPRegs:$rs, GNPRegs:$rd),
"div $rd, $rs, $rt" ,  [(set i16:$rt, (sdiv i16:$rs, i16:$rd))], IIIdiv>;

// let Constraints = "$rd_out = $rd" in
// def REM : F1 <0b01000, (outs GNPRegs:$rd_out), (ins GNPRegs:$rs, GNPRegs:$rd),
// "div $rd, $rs, $rd_out" ,  [(set i16:$rd_out, (srem i16:$rs, i16:$rd))], IIIdiv>;


// Default to modifying no non-operand registers
//let Defs = [HI, LO];     

// def SDIV : F1 <0b01000, (outs), (ins GNPRegs:$rs, GNPRegs:$rd),
// "div  $rd, $rs", [(LASERDivRem i16:$rs, i16:$rd)], IIIdiv>;

/// LOAD and STORE have token chains as their first operand, then the same
/// operands as an LLVM load/store instruction, then an offset node that
/// is added / subtracted from the base pointer to form the address (for
/// indexed memory ops).


// def IMD : F_long_1 <0b10101, (outs GNPRegs:$rd), (ins i16imm:$val), 
// "imd $rd, #$val", [(set i16:$rd, LASERimm16:$val)], IIAlu> {
//      let isMoveImm = 1;
// } 


// def IMD_GV : F_long_1 <0b10111, (outs GNPRegs:$rd), (ins LASERimm16:$val), 
// "imd $rd, @$val", [(set i16:$rd, LASERimmSExt16:$val)], IIAlu> {
//      let isMoveImm = 1;
// } 


// def : Pat<(add CPURegs:$GP, (LaserGPRel tglobaladdr:$in)),
//           (ADD CPURegs:$GP, (IMD tglobaladdr:$in))>;

// Return instruction
class RetBase: F4<0b11101, (outs), (ins RVR:$dst), "ret", [(brind RVR:$dst)], IIBranch> { 
let isReturn = 1;
let isCodeGenOnly = 1;
let hasCtrlDep = 1;
let hasExtraSrcRegAllocReq = 1;
let isTerminator = 1;
let isBarrier = 1;
let hasDelaySlot = 1;
let isIndirectBranch = 1;
let isPseudo = 0;
let Uses = [RETADDR];
}   

def RET: RetBase;

def RET2: F4 <0b11101, (outs), (ins), 
"ret", [], IIBranch>; 

def JMP : FJ <0b10010, (outs), (ins LASERjmptarget11:$address),
"jmp $address", [(br bb:$address)], IIBranch> {
let isBranch = 1;          
let isTerminator = 1;   
let isBarrier = 1;
let hasDelaySlot = 0;
}

def JC : FJ <0b10101, (outs), (ins LASERjmptarget11:$address),
"jc [$address]", [], IIBranch> {
let isBranch = 1;          
let isTerminator = 1;   
let isBarrier = 0;
let hasDelaySlot = 0;
let Uses = [FLAGR];
}

def JNC : FJ <0b10110, (outs), (ins LASERjmptarget11:$address),
"jnc [$address]", [], IIBranch> {                               
let isBranch = 1;          
let isTerminator = 1;   
let isBarrier = 0;
let hasDelaySlot = 0;
let Uses = [FLAGR];
}

def JZ : FJ <0b10011, (outs), (ins LASERjmptarget11:$address),
"jz [$address]", [], IIBranch> {
let isBranch = 1;          
let isTerminator = 1;   
let isBarrier = 0;
let hasDelaySlot = 0;
let Uses = [FLAGR];
}

def JNZ : FJ <0b10100, (outs), (ins LASERjmptarget11:$address),
"jnz [$address]", [], IIBranch> {
let isBranch = 1;          
let isTerminator = 1;   
let isBarrier = 0;
let hasDelaySlot = 0;
let Uses = [FLAGR];
}

def : Pat <(LaserJump (bb:$address), SETLT, FLAGR), 
           (JC (bb:$address))>;

def : Pat <(LaserJump (bb:$address), SETGT, FLAGR), 
           (JNC (bb:$address))>;

def : Pat <(LaserJump (bb:$address), SETEQ, FLAGR), 
           (JZ (bb:$address))>;

def : Pat <(LaserJump (bb:$address), SETNE, FLAGR), 
           (JNZ (bb:$address))>;

def LD16ri : F2 <0b01111, (outs GNPRegs:$dst), (ins i16imm:$src), 
    "ld $dst, $src", [(set i16:$dst, imm:$src)], IILoad> {
  let isPseudo = 1;
}

// addresses

def : Pat <(i16 (LaserWrapper tglobaladdr:$src)),
           (LD16ri tglobaladdr:$src)>;

def : Pattern<(i16 (load_sym tglobaladdr:$addr)),  [(IMD tglobaladdr:$addr)]>;


// def : Pat<(LaserHi tglobaladdr:$in), (IMD tglobaladdr:$in)>;
// def : Pat<(LaserLo tglobaladdr:$in), (IMD tglobaladdr:$in)>;


// def LD16ri : F2 <0b01111, (outs GNPRegs:$rd), (ins memsrc:$rs), 
//             "ld $rd, $rs", [(set i16:$rd, (load addr:$rs))], IILoad>; 


// def : Pat <(i16 (LaserWrapper tglobaladdr:$rs)),
//            (LD GNPRegs:$rd, tglobaladdr:$rs)>;

// def LD : F2 <0b01111, (outs GNPRegs:$rd), (ins memsrc:$rs), 
// "ld $rd, $rs", [(set i16:$rd, (load addr:$rs))], IILoad>; 


