// The encoding used for conditional codes used in BR instructions

#ifndef LLVM_LIB_TARGET_LASER_LASERCONDCODE_H
#define LLVM_LIB_TARGET_LASER_LASERCONDCODE_H

#include "llvm/ADT/StringSwitch.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {
  namespace LASERCC {
    enum CondCode {
      ICC_T = 0,   //  true
      ICC_F = 1,   //  false
      ICC_HI = 2,  //  high
      ICC_UGT = 2, //  unsigned greater than
      ICC_LS = 3,  //  low or same
      ICC_ULE = 3, //  unsigned less than or equal
      ICC_CC = 4,  //  carry cleared
      ICC_ULT = 4, //  unsigned less than
      ICC_CS = 5,  //  carry set
      ICC_UGE = 5, //  unsigned greater than or equal
      ICC_NE = 6,  //  not equal
      ICC_EQ = 7,  //  equal
      ICC_VC = 8,  //  oVerflow cleared
      ICC_VS = 9,  //  oVerflow set
      ICC_PL = 10, //  plus
      ICC_MI = 11, //  minus
      ICC_GE = 12, //  greater than or equal
      ICC_LT = 13, //  less than
      ICC_GT = 14, //  greater than
      ICC_LE = 15, //  less than or equal
      UNKNOWN
    };


    inline static StringRef lanaiCondCodeToString(LASERCC::CondCode CC) {
      switch (CC) {
      case LASERCC::ICC_T:
	return "t"; // true
      case LASERCC::ICC_F:
	return "f"; // false
      case LASERCC::ICC_NE:
	return "ne"; // not equal
      case LASERCC::ICC_EQ:
	return "eq"; // equal
      case LASERCC::ICC_VC:
	return "vc"; // oVerflow cleared
      case LASERCC::ICC_VS:
	return "vs"; // oVerflow set
      case LASERCC::ICC_PL:
	return "pl"; // plus
      case LASERCC::ICC_MI:
	return "mi"; // minus
      case LASERCC::ICC_GE:
	return "ge"; // greater than or equal
      case LASERCC::ICC_LT:
	return "lt"; // less than
      case LASERCC::ICC_GT:
	return "gt"; // greater than
      case LASERCC::ICC_LE:
	return "le"; // less than or equal
      case LASERCC::ICC_UGT:
	return "ugt"; // high | unsigned greater than
      case LASERCC::ICC_ULE:
	return "ule"; // low or same | unsigned less or equal
      case LASERCC::ICC_ULT:
	return "ult"; // carry cleared | unsigned less than
      case LASERCC::ICC_UGE:
	return "uge"; // carry set | unsigned than or equal
      default:
	llvm_unreachable("Invalid cond code");
      }
    }

    inline static CondCode suffixToLaserCondCode(StringRef S) {
      return StringSwitch<CondCode>(S)
	.EndsWith("f", LASERCC::ICC_F)
	.EndsWith("hi", LASERCC::ICC_HI)
	.EndsWith("ugt", LASERCC::ICC_UGT)
	.EndsWith("ls", LASERCC::ICC_LS)
	.EndsWith("ule", LASERCC::ICC_ULE)
	.EndsWith("cc", LASERCC::ICC_CC)
	.EndsWith("ult", LASERCC::ICC_ULT)
	.EndsWith("cs", LASERCC::ICC_CS)
	.EndsWith("uge", LASERCC::ICC_UGE)
	.EndsWith("ne", LASERCC::ICC_NE)
	.EndsWith("eq", LASERCC::ICC_EQ)
	.EndsWith("vc", LASERCC::ICC_VC)
	.EndsWith("vs", LASERCC::ICC_VS)
	.EndsWith("pl", LASERCC::ICC_PL)
	.EndsWith("mi", LASERCC::ICC_MI)
	.EndsWith("ge", LASERCC::ICC_GE)
	.EndsWith("lt", LASERCC::ICC_LT)
	.EndsWith("gt", LASERCC::ICC_GT)
	.EndsWith("le", LASERCC::ICC_LE)
	.EndsWith("t", LASERCC::ICC_T) // Has to be after others with suffix t
	.Default(LASERCC::UNKNOWN);
    }
  } // namespace LASERCC
} // namespace llvm

#endif // LLVM_LIB_TARGET_LANAI_LANAICONDCODE_H
