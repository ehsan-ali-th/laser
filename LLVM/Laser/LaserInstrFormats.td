//===-- LaserInstrFormats.td - Laser Instruction Formats ---*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

class Format<bits<4> val> {
      bits<4> Value = val;
}

def Pseudo : Format<0>;
def Frm1 : Format<1>;
def Frm2 : Format<2>;
def Frm3 : Format<3>;
def Frm4 : Format<4>;
def Frm5 : Format<5>;


// F_base
class F_base <dag outs, dag ins, string asmString, list<dag> pattern, 
             InstrItinClass itin, Format f> 
      : Instruction {
  field bits<16> Inst;
  Format Form = f;
  let Size = 2;
  field bits<16> SoftFail = 0;


  let Namespace = "LASER";

  bits<5> Opcode = 0;
  let Inst{15-11} = Opcode;     

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = asmString;
  let Pattern = pattern;  
  let Itinerary = itin;

  //
  // Attributes specific to Laser instructions...
  //
  bits<4> FormBits = Form.Value;
  // TSFlags layout should be kept in sync with LaserInstrInfo.h.
  let TSFlags{3-0}  = FormBits;

  let DecoderNamespace = "LASER";
}

// Laser Pseudo Instructions Format
class LaserPseudo<dag outs, dag ins, string asmString, list<dag> pattern> 
      : F_base <outs, ins, asmString, pattern, IIPseudo, Pseudo> {
  let isCodeGenOnly = 1;
  let isPseudo = 1;
}



// F1: Opcode + RD + RS + RT
class F1<bits<5> op, dag outs, dag ins, string asmString, list<dag> pattern,
                 InstrItinClass itin> 
      : F_base <outs, ins, asmString, pattern, itin, Frm1>
      {
      bits<4> rd;
      bits<4> rs; 
      bits<3> rt; 

      let Opcode = op;

      let Inst{10-7} = rd;
      let Inst{6-3} = rs;
      let Inst{2-0} = rt;

}

// F2: Opcode + RD + RS 
class F2<bits<5> op, dag outs, dag ins, string asmString, list<dag> pattern,
                 InstrItinClass itin> 
      : F_base <outs, ins, asmString, pattern, itin, Frm2>
      {
      bits<4> rd;
      bits<4> rs; 
      bits<3> rt; 

      let Opcode = op;

      let Inst{10-7} = rd;
      let Inst{6-3} = rs;
      let Inst{2-0} = 0;
}

// F3: Opcode + RD
class F3<bits<5> op, dag outs, dag ins, string asmString, list<dag> pattern,
                 InstrItinClass itin> 
      : F_base <outs, ins, asmString, pattern, itin, Frm3>
      {
      bits<4> rd;
      bits<4> rs; 
      bits<3> rt; 

      let Opcode = op;

      let Inst{10-7} = rd;
      let Inst{6-3} = 0;
      let Inst{2-0} = 0;
}

// F4: Opcode
class F4<bits<5> op, dag outs, dag ins, string asmString, list<dag> pattern,
                 InstrItinClass itin> 
      : F_base <outs, ins, asmString, pattern, itin, Frm4>
      {
  
      bits<4> rd;
      bits<4> rs; 
      bits<3> rt; 

      let Opcode = op;

      let Inst{10-7} = 0;
      let Inst{6-3} = 0;
      let Inst{2-0} = 0;
}

// FJ: Opcode + 11-bit Jump address
class FJ<bits<5> op, dag outs, dag ins, string asmString, list<dag> pattern,
                 InstrItinClass itin> 
      : F_base <outs, ins, asmString, pattern, itin, Frm4>
      {
      bits<11> address;

      let Opcode = op;

      let Inst{10-0} = address;
}

class F_base_long <dag outs, dag ins, string asmString, list<dag> pattern, 
             InstrItinClass itin, Format f> 
      : Instruction {
  field bits<32> Inst;
  Format Form = f;
  let Size = 4;
  field bits<32> SoftFail = 0;

  let Namespace = "LASER";

  bits<5> Opcode = 0;
  let Inst{31-27} = Opcode;     

  bits<16> val;
  let Inst{15-0} = val;

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = asmString;
  let Pattern = pattern;  
  let Itinerary = itin;

  //
  // Attributes specific to Laser instructions...
  //
  bits<4> FormBits = Form.Value;
  // TSFlags layout should be kept in sync with LaserInstrInfo.h.
  let TSFlags{3-0}  = FormBits;

  let DecoderNamespace = "LASER";
}

// F_long_1: Opcode + 16 bit data
class F_long_1<bits<5> op,  dag outs, dag ins, string asmString, list<dag> pattern,
                 InstrItinClass itin> 
      : F_base_long <outs, ins, asmString, pattern, itin, Frm1>
      {
      bits<4> rd;
      bits<4> rs = 0; 
      bits<3> rt = 0; 

      let Opcode = op; // Bits: 31, 30, 29, 28, 27

      let Inst{26-23} = rd; // Bits: 26, 25, 24, 23  
      let Inst{22-19} = rs;  // Bits: 22, 21, 20, 19  
      let Inst{18-16} = rt;  // Bits: 18, 17, 16    
}


