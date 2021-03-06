----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 06/14/2017 02:14:10 AM
-- Design Name: 
-- Module Name: CU - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity CU is
Port (  reset : in std_logic; 
        clk : in std_logic;
		halt : out std_logic;
		ROUT0 : out STD_LOGIC_VECTOR (6 downto 0)
--        ROUT1 : out STD_LOGIC_VECTOR (6 downto 0)
   );
end CU;

architecture Behavioral of CU is

	-- state type definition
	type state_type is (
		s_start,              -- 0
		s_fetch,              -- 1
		s_fetch2,             -- 2
		s_fetch3,             -- 3
		s_decode,             -- 4
		s_decode2,            -- 5
		s_decode3,            -- 
		s_inst_mov,           -- 6
		s_inst_mov2,          -- 7
		s_inst_add,           -- 8
		s_inst_add2,          -- 9
		s_inst_add3,          -- 10
		s_inst_adc,           -- 11
		s_inst_adc2,          -- 12
		s_inst_adc3,          -- 13
		s_inst_sub,           -- 14
		s_inst_sub2,          -- 15
		s_inst_sub3,          -- 16
		s_inst_sbc,           -- 17
		s_inst_sbc2,          -- 18
		s_inst_sbc3,          -- 19
		s_inst_inc,           -- 20
		s_inst_inc2,          -- 21
		s_inst_dec,           -- 22
		s_inst_dec2,	      -- 23
		s_inst_mul,           -- 24
		s_inst_mul2,          -- 25
		s_inst_mul3,          -- 26
		s_inst_div,           -- 27
		s_inst_div2,          -- 28
		s_inst_div3,          -- 29
		s_inst_and,           -- 30
		s_inst_and2,          -- 31
		s_inst_and3,          -- 32
		s_inst_or,            -- 33
		s_inst_or2,           -- 34
		s_inst_or3,           -- 35
		s_inst_xor,           -- 36
		s_inst_xor2,          -- 37
		s_inst_xor3,          -- 38
		s_inst_not,           -- 39
		s_inst_not2,          -- 40
		s_inst_not3,          -- 41
		s_inst_srl,           -- 42
		s_inst_srl2,          -- 43
		s_inst_srl3,          -- 44
		s_inst_sll,           -- 45
		s_inst_sll2,          -- 46
		s_inst_sll3,          -- 47
		s_inst_ld,            -- 48       
		s_inst_ld2,           -- 49
		s_inst_ld3,           -- 50
		s_inst_st,            -- 51
		s_inst_st2,           -- 52
		s_inst_st3,           -- 53
		s_inst_cmp,           -- 54
		s_inst_cmp2,          -- 55
		s_inst_jmp,           -- 56
		s_inst_jmp2,          -- 57
		s_inst_jz,            -- 58
		s_inst_jz2,           -- 59
		s_inst_jnz,           -- 60
		s_inst_jnz2,          -- 61  
		s_inst_jc,            -- 62
		s_inst_jnc,           -- 63
		s_inst_imd,           -- 64
		s_inst_imd2,          -- 65
		s_inst_imd3,          -- 66
		s_inst_imd4,          -- 67
		s_inst_in,            -- 68   
		s_inst_out,           -- 69
		s_inst_out2,          -- 70
		s_inst_push,          -- 71
		s_inst_push2,	      -- 72  
		s_inst_push3,         -- 73
		s_inst_push4,         -- 74        
		s_inst_pop,           -- 75
		s_inst_pop2,          -- 76
		s_inst_pop3,          -- 77
		s_inst_clrc,          -- 78
		s_inst_setc,          -- 79
		s_inst_call,          -- 80
		s_inst_call2,         -- 81
		s_inst_ret,           -- 82    
		s_inst_ret2,          -- 83
		s_inst_nop,           -- 84
		s_inst_nop2,          -- 85
		s_halt                -- 86
	);
	
	-- A pack of 16 * 16-bit registers
    -- type t_stack is array (15 downto 0) of std_logic_vector (15 downto 0);

    signal state: state_type;    -- Holds the current state

    -- Internal Registers
    signal TMP17: std_logic_vector (16 downto 0);         -- Temporary Register
    signal TMP: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000" ;         -- Temporary Register
    signal PC: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";          -- Program Couunter Register
    signal IR: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";          -- Instruction Register
    signal SP: std_logic_vector (16 downto 0):= B"0_0000_0000_0000_0000";          -- Stack Pointer Register    
    signal SS: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";          -- Stack Segment Register
    signal RETADDR: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";     -- Return Address Register
    signal RD: std_logic_vector (3 downto 0) := "0000";           -- RD Register
    signal RS: std_logic_vector (3 downto 0) := "0000";           -- RS Register
    signal RT: std_logic_vector (3 downto 0) := "0000";           -- RT Register
    signal RTA: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";         -- Temporariy A Register
    signal RTB: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";         -- Temporariy B Register
    signal RESULT: std_logic_vector (16 downto 0) := B"0_0000_0000_0000_0000";      -- ALU Result Register
    signal RESULT2: std_logic_vector (31 downto 0) 
        := B"0000_0000_0000_0000_0000_0000_0000_0000";     -- ALU Result Register2
    -- signal STACK: t_stack;
    -- FLAG Register
    --    BIT 0 :     Zero 
    --    BIT 1 :     Carry 
    --    BIT 2 :     Overflow
    --    BIT 3 : 
    --    BIT 4 : 
    --    BIT 5 : 
    --    BIT 6 : 
    --    BIT 7 : 
    --    BIT 8 : 
    --    BIT 9 : 
    --    BIT 10 : 
    --    BIT 11 : 
    --    BIT 12 : 
    --    BIT 13 : 
    --    BIT 14 : 
    --    BIT 15 : 
    signal FLAGR: std_logic_vector (15 downto 0);       -- Flag Register
    
    -- Memory Signals
    signal MEM_ADDR: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";    -- Memory Address 
    signal MEM_EN: STD_LOGIC := '0';
    signal MEM_WR: STD_LOGIC_VECTOR(0 downto 0) := "0";
    signal MEM_DATA_IN: STD_LOGIC_VECTOR(15 downto 0) := B"0000_0000_0000_0000";
    signal MEM_DATA_OUT: STD_LOGIC_VECTOR(15 downto 0) := B"0000_0000_0000_0000";
    
    -- General Purpose Registers
    signal R0: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R1: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R2: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R3: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R4: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R5: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R6: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R7: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R8: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R9: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R10: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R11: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R12: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R13: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R14: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    signal R15: std_logic_vector (15 downto 0) := B"0000_0000_0000_0000";
    
    component blk_mem_gen_0 IS
        PORT (
            clka : IN STD_LOGIC;
            ena : IN STD_LOGIC;
            wea : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
            addra : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
            dina : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
            douta : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
        );
    END component;
    
begin

  bram0: blk_mem_gen_0
      PORT MAP (
        clka => clk,
        ena => MEM_EN,
        wea => MEM_WR,
        addra => MEM_ADDR (11 downto 0),
        dina => MEM_DATA_IN,
        douta => MEM_DATA_OUT
      );
      
    process (clk) begin
        if(clk'event and clk = '1') then    -- Rising edge of clock

        if (reset = '1') then
        -- Start from s_start when reset is asserted.
            state <= s_start;
            -- Reset all signals
            TMP17 <= B"0_0000_0000_0000_0000";
            TMP <= B"0000_0000_0000_0000";
            PC <= B"0000_0000_0000_0000";
            IR <= B"0000_0000_0000_0000";
            SP <= B"0_0000_0000_0000_0000";    
            SS <= B"1111_1111_1111_1111"; -- End of RAM block
            RETADDR <= B"0000_0000_0000_0000";
            RD <= B"0000"; 
            RS <= B"0000";
            RT <= B"0000";
            RTA <= B"0000_0000_0000_0000";
            RTB <= B"0000_0000_0000_0000";
            RESULT <= B"0_0000_0000_0000_0000";
            RESULT2 <= B"0000_0000_0000_0000_0000_0000_0000_0000";
            FLAGR <= B"0000_0000_0000_0000";
            MEM_DATA_IN <= B"0000_0000_0000_0000";
            MEM_ADDR <= B"0000_0000_0000_0000";
            MEM_WR <= "0";
            MEM_EN <= '0';
            halt <= '0';
            ROUT0 <= B"000_0000";
            R0 <= B"0000_0000_0000_0000";
            R1 <= B"0000_0000_0000_0000";
            R2 <= B"0000_0000_0000_0000";
            R3 <= B"0000_0000_0000_0000";
            R4 <= B"0000_0000_0000_0000";
            R5 <= B"0000_0000_0000_0000";
            R6 <= B"0000_0000_0000_0000";
            R7 <= B"0000_0000_0000_0000";
            R8 <= B"0000_0000_0000_0000";
            R9 <= B"0000_0000_0000_0000";
            R10 <= B"0000_0000_0000_0000";
            R11 <= B"0000_0000_0000_0000";
            R12 <= B"0000_0000_0000_0000";
            R13 <= B"0000_0000_0000_0000";
            R14 <= B"0000_0000_0000_0000";
            R15 <= B"0000_0000_0000_0000";
--          ROUT1 <= (others => '0');
        else        
		      case state is
                when s_start =>
                     state <= s_fetch;
                when s_fetch => 
                    MEM_ADDR <= PC;
                    MEM_WR <= "0";
                    MEM_EN <= '1';
                    halt <= '0';
--                    ROUT0 <= B"000_0001";
                    state <= s_fetch2;
                when s_fetch2 => 
                    TMP <= PC;
                    MEM_EN <= '1'; 
                    halt <= '0';
--                    ROUT0 <= B"000_0010";
                    state <= s_fetch3;
                when s_fetch3 => 
                    MEM_EN <= '0';
                    PC <= TMP  + '1';
                    -- Wait for the data be present on MEM_DATA_OUT 
                    halt <= '0';
--                    ROUT0 <= B"000_0011";
                    state <= s_decode;
                when s_decode => 
                    MEM_EN <= '1';
                    IR <= MEM_DATA_OUT;
                    halt <= '0';
--                    ROUT0 <= B"000_0100";
                    state <= s_decode2;
                when s_decode2 => 
                    MEM_EN <= '1';
                    halt <= '0';
--                    ROUT0 <= B"000_0101";
                    state <= s_decode3;
                when s_decode3 => 
                    case IR(15 downto 11) is
                        when "00000" => -- MOV
                            state <= s_inst_mov;                        
                        when "00001" => -- ADD
                            state <= s_inst_add;                        
                        when "00010" => -- ADC
                            state <= s_inst_adc;                        
                        when "00011" => -- SUB
                            state <= s_inst_sub;                        
                        when "00100" => -- SBC
                            state <= s_inst_sbc;
                        when "00101" => -- INC
                            state <= s_inst_inc;
                        when "00110" => -- DEC
                            state <= s_inst_dec;
                        when "00111" => -- MUL
                            state <= s_inst_mul;
                        when "01000" => -- DIV
                            state <= s_inst_div;
                        when "01001" => -- AND
                            state <= s_inst_and;
                        when "01010" => -- OR
                            state <= s_inst_or;
                        when "01011" => -- XOR
                            state <= s_inst_xor;
                        when "01100" => -- NOT
                            state <= s_inst_not;
                        when "01101" => -- SRL
                            state <= s_inst_srl;
                        when "01110" => -- SLL
                            state <= s_inst_sll;
                        when "01111" => -- LD
                            state <= s_inst_ld;
                        when "10000" => -- ST
                            state <= s_inst_st;
                        when "10001" => -- CMP
                            state <= s_inst_cmp;
                        when "10010" => -- JMP
                            state <= s_inst_jmp;
                        when "10011" => -- JZ
                            state <= s_inst_jz;
                        when "10100" => -- JNZ
                            state <= s_inst_jnz;
                        when "10101" => -- JC
                            state <= s_inst_jc;
                        when "10110" => -- JNC
                            state <= s_inst_jnc;
                        when "10111" => -- IMD
                            state <= s_inst_imd;
                        when "11000" => -- IN
                            state <= s_inst_in;
                        when "11001" => -- OUT
                            state <= s_inst_out;
                        when "11010" => -- CLRC
                            state <= s_inst_clrc;
                        when "11011" => -- SETC
                            state <= s_inst_setc;
                        when "11100" => -- CALL
                            state <= s_inst_call;
                        when "11101" => -- RET
                            state <= s_inst_ret;
                        when "11110" => -- NOP
                            state <= s_inst_nop;
                        when others => state <= s_halt;
                    end case;
                when s_inst_mov =>
                    halt <= '0';
--                    ROUT0 <= B"100_0000";
                    -- Set the RTA according to the values in RS
                    --        of the instruction (IR).
                    case IR(6 downto 3) is    -- RS                    
                        when "0000" => RTA <= R0;
                        when "0001" => RTA <= R1;
                        when "0010" => RTA <= R2;
                        when "0011" => RTA <= R3;
                        when "0100" => RTA <= R4;
                        when "0101" => RTA <= R5;
                        when "0110" => RTA <= R6;
                        when "0111" => RTA <= R7;
                        when "1000" => RTA <= R8;
                        when "1001" => RTA <= R9;
                        when "1010" => RTA <= R10;
                        when "1011" => RTA <= R11;
                        when "1100" => RTA <= R12;
                        when "1101" => RTA <= R13;
                        when "1110" => RTA <= R14;
                        when "1111" => RTA <= R15;
                        when others => RTA <= B"0000_0000_0000_0000";
                    end case; 
                    state <= s_inst_mov2;
                when s_inst_mov2 =>
                    halt <= '0';
--                    ROUT0 <= B"000_0000";
                    -- Update the memoery address
                    MEM_ADDR <= PC;    
                    -- Copy into RD according to the values in RD
                    --        of the instruction (IR).
                    case IR(10 downto 7) is    -- RD
                        when "0000" => R0 <= RTA;
                        when "0001" => R1 <= RTA;
                        when "0010" => R2 <= RTA;
                        when "0011" => R3 <= RTA;
                        when "0100" => R4 <= RTA;
                        when "0101" => R5 <= RTA;
                        when "0110" => R6 <= RTA;
                        when "0111" => R7 <= RTA;
                        when "1000" => R8 <= RTA;
                        when "1001" => R9 <= RTA;
                        when "1010" => R10 <= RTA;
                        when "1011" => R11 <= RTA;
                        when "1100" => R12 <= RTA;
                        when "1101" => R13 <= RTA;
                        when "1110" => R14 <= RTA;
                        when "1111" => R15 <= RTA;
                        when others => null ;
                    end case;                  
                    state <= s_fetch;
                when s_inst_add =>
                    halt <= '0';
--                    ROUT0 <= B"000_0000";
                    -- RT <- (RD + RS)    
                    case IR(10 downto 7) is    -- RD
                        when "0000" => RTA <= R0;
                        when "0001" => RTA <= R1;
                        when "0010" => RTA <= R2;
                        when "0011" => RTA <= R3;
                        when "0100" => RTA <= R4;
                        when "0101" => RTA <= R5;
                        when "0110" => RTA <= R6;
                        when "0111" => RTA <= R7;
                        when "1000" => RTA <= R8;
                        when "1001" => RTA <= R9;
                        when "1010" => RTA <= R10;
                        when "1011" => RTA <= R11;
                        when "1100" => RTA <= R12;
                        when "1101" => RTA <= R13;
                        when "1110" => RTA <= R14;
                        when "1111" => RTA <= R15;
                        when others => null;
                    end case;    
                    case IR(6 downto 3) is    -- RS
                        when "0000" => RTB <= R0;
                        when "0001" => RTB <= R1;
                        when "0010" => RTB <= R2;
                        when "0011" => RTB <= R3;
                        when "0100" => RTB <= R4;
                        when "0101" => RTB <= R5;
                        when "0110" => RTB <= R6;
                        when "0111" => RTB <= R7;
                        when "1000" => RTB <= R8;
                        when "1001" => RTB <= R9;
                        when "1010" => RTB <= R10;
                        when "1011" => RTB <= R11;
                        when "1100" => RTB <= R12;
                        when "1101" => RTB <= R13;
                        when "1110" => RTB <= R14;
                        when "1111" => RTB <= R15;
                        when others => null;
                    end case; 
                    state <= s_inst_add2;
                when s_inst_add2 =>
                    halt <= '0';
--                    ROUT0 <= B"000_0000";
                    RESULT <=   ('0' & RTA) + -- RD
                                ('0' & RTB);     -- RS
                    state <= s_inst_add3;
                when s_inst_add3 =>
                    halt <= '0';
--                   ROUT0 <= B"000_0000";
                   -- Update the memoery address
                   MEM_ADDR <= PC;    
                   case IR(2 downto 0) is    -- RT
                       when "000" => 
                           R0 <= RESULT (15 downto 0);    -- Save  the addition result
                           FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                       when "001" => 
                           R1 <= RESULT (15 downto 0);    -- Save  the addition result
                           FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                       when "010" => 
                           R2 <= RESULT (15 downto 0);    -- Save  the addition result
                           FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                       when "011" => 
                           R3 <= RESULT (15 downto 0);    -- Save  the addition result
                           FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                       when "100" => 
                           R4 <= RESULT (15 downto 0);    -- Save  the addition result
                           FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                       when "101" => 
                           R5 <= RESULT (15 downto 0);    -- Save  the addition result
                           FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                       when "110" => 
                           R6 <= RESULT (15 downto 0);    -- Save  the addition result
                           FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                       when "111" => 
                           R7 <= RESULT (15 downto 0);    -- Save  the addition result
                           FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                       when others => null;
                   end case;
                   state <= s_fetch;
                when s_inst_adc =>
                    halt <= '0';
--                    ROUT0 <= B"000_0000";
                    -- RT <- (RD + RS)    
                    case IR(10 downto 7) is    -- RD
                        when "0000" => RTA <= R0;
                        when "0001" => RTA <= R1;
                        when "0010" => RTA <= R2;
                        when "0011" => RTA <= R3;
                        when "0100" => RTA <= R4;
                        when "0101" => RTA <= R5;
                        when "0110" => RTA <= R6;
                        when "0111" => RTA <= R7;
                        when "1000" => RTA <= R8;
                        when "1001" => RTA <= R9;
                        when "1010" => RTA <= R10;
                        when "1011" => RTA <= R11;
                        when "1100" => RTA <= R12;
                        when "1101" => RTA <= R13;
                        when "1110" => RTA <= R14;
                        when "1111" => RTA <= R15;
                        when others => null;
                    end case;    
                    case IR(6 downto 3) is    -- RS
                        when "0000" => RTB <= R0;
                        when "0001" => RTB <= R1;
                        when "0010" => RTB <= R2;
                        when "0011" => RTB <= R3;
                        when "0100" => RTB <= R4;
                        when "0101" => RTB <= R5;
                        when "0110" => RTB <= R6;
                        when "0111" => RTB <= R7;
                        when "1000" => RTB <= R8;
                        when "1001" => RTB <= R9;
                        when "1010" => RTB <= R10;
                        when "1011" => RTB <= R11;
                        when "1100" => RTB <= R12;
                        when "1101" => RTB <= R13;
                        when "1110" => RTB <= R14;
                        when "1111" => RTB <= R15;
                        when others => null;
                    end case; 
                    state <= s_inst_adc2;
                when s_inst_adc2 =>
                    halt <= '0';
--                    ROUT0 <= B"000_0000";
                    RESULT <= ('0' & RTA) + -- RD
                              ('0' & RTB) + -- RS
                    FLAGR(1);         -- Carry
                    state <= s_inst_adc3;
                when s_inst_adc3 =>
                    halt <= '0';
--                    ROUT0 <= B"000_0000";
                    -- Update the memoery address
                    MEM_ADDR <= PC;    
                    case IR(2 downto 0) is    -- RT
                        when "000" => 
                            R0 <= RESULT (15 downto 0);    -- Save  the addition result
                            FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                        when "001" => 
                            R1 <= RESULT (15 downto 0);    -- Save  the addition result
                            FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                        when "010" => 
                            R2 <= RESULT (15 downto 0);    -- Save  the addition result
                            FLAGR(1) <= RESULT (16);        --    Set Carry  flag    
                        when "011" => 
                            R3 <= RESULT (15 downto 0);    -- Save  the addition result
                            FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                        when "100" => 
                            R4 <= RESULT (15 downto 0);    -- Save  the addition result
                            FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                        when "101" => 
                            R5 <= RESULT (15 downto 0);    -- Save  the addition result
                            FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                        when "110" => 
                            R6 <= RESULT (15 downto 0);    -- Save  the addition result
                            FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                        when "111" => 
                            R7 <= RESULT (15 downto 0);    -- Save  the addition result
                            FLAGR(1) <= RESULT (16);        --    Set Carry flag    
                        when others => null;
                    end case;
                    state <= s_fetch;
                when s_inst_sub =>
                    halt <=     '0';
--                    ROUT0 <= B"000_0000";
                    -- RT <- (RS - RD)    
                    case IR(10 downto 7) is    -- RD = RTA
                        when "0000" => RTA <= R0;
                        when "0001" => RTA <= R1;
                        when "0010" => RTA <= R2;
                        when "0011" => RTA <= R3;
                        when "0100" => RTA <= R4;
                        when "0101" => RTA <= R5;
                        when "0110" => RTA <= R6;
                        when "0111" => RTA <= R7;
                        when "1000" => RTA <= R8;
                        when "1001" => RTA <= R9;
                        when "1010" => RTA <= R10;
                        when "1011" => RTA <= R11;
                        when "1100" => RTA <= R12;
                        when "1101" => RTA <= R13;
                        when "1110" => RTA <= R14;
                        when "1111" => RTA <= R15;
                        when others => null;
                    end case;    
                    case IR(6 downto 3) is    -- RS = RTB
                        when "0000" => RTB <= R0;
                        when "0001" => RTB <= R1;
                        when "0010" => RTB <= R2;
                        when "0011" => RTB <= R3;
                        when "0100" => RTB <= R4;
                        when "0101" => RTB <= R5;
                        when "0110" => RTB <= R6;
                        when "0111" => RTB <= R7;
                        when "1000" => RTB <= R8;
                        when "1001" => RTB <= R9;
                        when "1010" => RTB <= R10;
                        when "1011" => RTB <= R11;
                        when "1100" => RTB <= R12;
                        when "1101" => RTB <= R13;
                        when "1110" => RTB <= R14;
                        when "1111" => RTB <= R15;
                        when others => null;
                    end case; 
                    state <= s_inst_sub2;
                when s_inst_sub2 =>
                    halt <= '0';
--                    ROUT0 <= B"000_0000";
                    RESULT <= (('0' & RTB) + -- RS
                                 ('0' & (not(RTA) + '1')));     -- RS
                    state <= s_inst_sub3;
                when s_inst_sub3 =>
                    halt <= '0';
--                    ROUT0 <= B"000_0000";
                    -- Update the memoery address
                    MEM_ADDR <= PC;    
                    case IR(2 downto 0) is    -- RT
                        when "000" => 
                        R0 <= RESULT (15 downto 0);    -- Save the addition result
                        when "001" => 
                        R1 <= RESULT (15 downto 0);    -- Save the addition result
                        when "010" => 
                        R2 <= RESULT (15 downto 0);    -- Save the addition result
                        when "011" => 
                        R3 <= RESULT (15 downto 0);    -- Save the addition result
                        when "100" => 
                        R4 <= RESULT (15 downto 0);    -- Save the addition result
                        when "101" => 
                        R5 <= RESULT (15 downto 0);    -- Save the addition result
                        when "110" => 
                        R6 <= RESULT (15 downto 0);    -- Save the addition result
                        when "111" => 
                        R7 <= RESULT (15 downto 0);    -- Save the addition result
                        when others => null;
                    end case;    
                    if RESULT(15 downto 0) = 0 then
                        FLAGR(0) <= '1';        --    Set the zero flag    
                    else     
                        FLAGR(0) <= '0';        --    Unset the zero flag    
                    end if;
                    state <= s_fetch;
                when s_inst_sbc =>
                  halt <= '0';
--                  ROUT0 <= B"000_0000";
                  -- RT <- (RS - RD - Carry)    
                            case IR(10 downto 7) is    -- RD
                                when "0000" => RTA <= R0;
                                when "0001" => RTA <= R1;
                                when "0010" => RTA <= R2;
                                when "0011" => RTA <= R3;
                                when "0100" => RTA <= R4;
                                when "0101" => RTA <= R5;
                                when "0110" => RTA <= R6;
                                when "0111" => RTA <= R7;
                                when "1000" => RTA <= R8;
                                when "1001" => RTA <= R9;
                                when "1010" => RTA <= R10;
                                when "1011" => RTA <= R11;
                                when "1100" => RTA <= R12;
                                when "1101" => RTA <= R13;
                                when "1110" => RTA <= R14;
                                when "1111" => RTA <= R15;
                                when others => null;
                            end case;    
                            case IR(6 downto 3) is    -- RS
                                when "0000" => RTB <= R0;
                                when "0001" => RTB <= R1;
                                when "0010" => RTB <= R2;
                                when "0011" => RTB <= R3;
                                when "0100" => RTB <= R4;
                                when "0101" => RTB <= R5;
                                when "0110" => RTB <= R6;
                                when "0111" => RTB <= R7;
                                when "1000" => RTB <= R8;
                                when "1001" => RTB <= R9;
                                when "1010" => RTB <= R10;
                                when "1011" => RTB <= R11;
                                when "1100" => RTB <= R12;
                              when "1101" => RTB <= R13;
                              when "1110" => RTB <= R14;
                                when "1111" => RTB <= R15;
                                when others => null;
                            end case;    

                    state <= s_inst_sbc2;
                when s_inst_sbc2 =>
                  halt <= '0';
--                  ROUT0 <= B"000_0000";
                -- Calculate RTB - RTA - Carry = RTB - (RTA + Carry)
                --                                         = RS - (RD + Carry)        
                RESULT <= ('0' & RTB) + -- RS
                         ('0' & not(RTA + FLAGR(1)) + '1');     -- RD + Carry

                  state <= s_inst_sbc3;
                when s_inst_sbc3 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(2 downto 0) is    -- RT
                when "000" => 
                    R0 <= RESULT (15 downto 0);    -- Save  the addition result
                when "001" => 
                    R1 <= RESULT (15 downto 0);    -- Save  the addition result
                when "010" => 
                    R2 <= RESULT (15 downto 0);    -- Save  the addition result
                when "011" => 
                    R3 <= RESULT (15 downto 0);    -- Save  the addition result
                when "100" => 
                    R4 <= RESULT (15 downto 0);    -- Save  the addition result
                when "101" => 
                    R5 <= RESULT (15 downto 0);    -- Save  the addition result
                when "110" => 
                    R6 <= RESULT (15 downto 0);    -- Save  the addition result
                when "111" => 
                    R7 <= RESULT (15 downto 0);    -- Save  the addition result
                when others => null;
            end case;    
            if RESULT(15 downto 0) = 0 then
                FLAGR(0) <= '1';        --    Set the zero flag    
            else     
                FLAGR(0) <= '0';        --    Unset the zero flag    
            end if;    

                    state <= s_fetch;
                when s_inst_inc =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            case IR(10 downto 7) is        -- RD 
                when "0000" => 
                    RESULT <= ('0' & R0) + '1';
                when "0001" => 
                    RESULT <= ('0' & R1) + '1';
                when "0010" => 
                    RESULT <= ('0' & R2) + '1';
                when "0011" => 
                    RESULT <= ('0' & R3) + '1';
                when "0100" => 
                    RESULT <= ('0' & R4) + '1';
                when "0101" => 
                    RESULT <= ('0' & R5) + '1';
                when "0110" => 
                    RESULT <= ('0' & R6) + '1';
                when "0111" => 
                    RESULT <= ('0' & R7) + '1';
                when "1000" => 
                    RESULT <= ('0' & R8) + '1';
                when "1001" => 
                    RESULT <= ('0' & R9) + '1';
                when "1010" => 
                    RESULT <= ('0' & R10) + '1';
                when "1011" => 
                    RESULT <= ('0' & R11) + '1';
                when "1100" => 
                    RESULT <= ('0' & R12) + '1';
                when "1101" => 
                    RESULT <= ('0' & R13) + '1';
                when "1110" => 
                    RESULT <= ('0' & R14) + '1';
                when "1111" => 
                    RESULT <= ('0' & R15) + '1';
                when others => null;
            end case;

                    state <= s_inst_inc2;
                when s_inst_inc2 =>
            halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(10 downto 7) is        -- RD 
                when "0000" => 
                    R0 <= RESULT(15 downto 0);
                    if RESULT(16) <= '1' then
                        FLAGR(2) <= '1';    -- Set Overflow flag
                    end if;    
                when "0001" => 
                    R1 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "0010" => 
                    R2 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "0011" => 
                    R3 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "0100" => 
                    R4 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "0101" => 
                    R5 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "0110" => 
                    R6 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "0111" => 
                    R7 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "1000" => 
                    R8 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "1001" => 
                    R9 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "1010" => 
                    R10 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "1011" => 
                    R11 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "1100" => 
                    R12 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "1101" => 
                    R13 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "1110" => 
                    R14 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when "1111" => 
                    R15 <= RESULT(15 downto 0);
                    if RESULT(16) = '1' then
                        FLAGR(2) <= '1';
                    end if;    
                when others => null;
            end case;
                  
                    state <= s_fetch;
                when s_inst_dec =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            case IR(10 downto 7) is        -- RD 
                when "0000" => RESULT <= ('0' & R0) - '1';
                when "0001" => RESULT <= ('0' & R1) - '1';
                when "0010" => RESULT <= ('0' & R2) - '1';
                when "0011" => RESULT <= ('0' & R3) - '1';
                when "0100" => RESULT <= ('0' & R4) - '1';
                when "0101" => RESULT <= ('0' & R5) - '1';
                when "0110" => RESULT <= ('0' & R6) - '1';
                when "0111" => RESULT <= ('0' & R7) - '1';
                when "1000" => RESULT <= ('0' & R8) - '1';
                when "1001" => RESULT <= ('0' & R9) - '1';
                when "1010" => RESULT <= ('0' & R10) - '1';
                when "1011" => RESULT <= ('0' & R11) - '1';
                when "1100" => RESULT <= ('0' & R12) - '1';
                when "1101" => RESULT <= ('0' & R13) - '1';
                when "1110" => RESULT <= ('0' & R14) - '1';
                when "1111" => RESULT <= ('0' & R15) - '1';
                when others => null;
            end case;

                    state <= s_inst_dec2;
                when s_inst_dec2 =>
            halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(10 downto 7) is        -- RD 
                when "0000" => 
                    R0 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "0001" => 
                    R1 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "0010" => 
                    R2 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "0011" => 
                    R3 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "0100" => 
                    R4 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "0101" => 
                    R5 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "0110" => 
                    R6 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "0111" => 
                    R7 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "1000" => 
                    R8 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "1001" => 
                    R9 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "1010" => 
                    R10 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "1011" => 
                    R11 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "1100" => 
                    R12 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "1101" => 
                    R13 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "1110" => 
                    R14 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when "1111" => 
                    R15 <= RESULT(15 downto 0);
                    if RESULT(15 downto 0) = 0  then
                        FLAGR(0) <= '1';    -- Set the Zero Flag
                    end if;    
                when others => null;
            end case;
                  
                    state <= s_fetch;
                when s_inst_mul =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTA and RTB accorind to the values in RD and RS section
            --        of the instruction (IR).
            case IR(10 downto 7) is    -- RD
                when "0000" => RTA <= R0;
                when "0001" => RTA <= R1;
                when "0010" => RTA <= R2;
                when "0011" => RTA <= R3;
                when "0100" => RTA <= R4;
                when "0101" => RTA <= R5;
                when "0110" => RTA <= R6;
                when "0111" => RTA <= R7;
                when "1000" => RTA <= R8;
                when "1001" => RTA <= R9;
                when "1010" => RTA <= R10;
                when "1011" => RTA <= R11;
                when "1100" => RTA <= R12;
                when "1101" => RTA <= R13;
                when "1110" => RTA <= R14;
                when "1111" => RTA <= R15;
                when others => null;
            end case;    
            case IR(6 downto 3) is    -- RS
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;

                    state <= s_inst_mul2;
                when s_inst_mul2 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            RESULT2 <= std_logic_vector(unsigned(RTA) * unsigned(RTB));

                    state <= s_inst_mul3;
                when s_inst_mul3 =>
            halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(2 downto 0) is    -- RT
                when "000" => 
                    R0 <= RESULT2 (31 downto 16);    -- Save the result, high 16 bit
                when "001" => 
                    R1 <= RESULT2 (31 downto 16);    -- 
                when "010" => 
                    R2 <= RESULT2 (31 downto 16);    -- 
                when "011" => 
                    R3 <= RESULT2 (31 downto 16);    -- 
                when "100" => 
                    R4 <= RESULT2 (31 downto 16);    -- 
                when "101" => 
                    R5 <= RESULT2 (31 downto 16);    -- 
                when "110" => 
                    R6 <= RESULT2 (31 downto 16);    -- 
                when "111" => 
                    R7 <= RESULT2 (31 downto 16);    -- 
                when others => null;
            end case;
            case IR(10 downto 7) is    -- RD
                when "0000" => R0 <= RESULT2 (15 downto 0);
                when "0001" => R1 <= RESULT2 (15 downto 0);
                when "0010" => R2 <= RESULT2 (15 downto 0);
                when "0011" => R3 <= RESULT2 (15 downto 0);
                when "0100" => R4 <= RESULT2 (15 downto 0);
                when "0101" => R5 <= RESULT2 (15 downto 0);
                when "0110" => R6 <= RESULT2 (15 downto 0);
                when "0111" => R7 <= RESULT2 (15 downto 0);
                when "1000" => R8 <= RESULT2 (15 downto 0);
                when "1001" => R9 <= RESULT2 (15 downto 0);
                when "1010" => R10 <= RESULT2 (15 downto 0);
                when "1011" => R11 <= RESULT2 (15 downto 0);
                when "1100" => R12 <= RESULT2 (15 downto 0);
                when "1101" => R13 <= RESULT2 (15 downto 0);
                when "1110" => R14 <= RESULT2 (15 downto 0);
                when "1111" => R15 <= RESULT2 (15 downto 0);
                when others => null;
            end case;                
                  
                    state <= s_fetch;
                when s_inst_div =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTA and RTB accorind to the values in RD and RS section
            --        of the instruction (IR).
            case IR(10 downto 7) is    -- RD
                when "0000" => RTA <= R0;
                when "0001" => RTA <= R1;
                when "0010" => RTA <= R2;
                when "0011" => RTA <= R3;
                when "0100" => RTA <= R4;
                when "0101" => RTA <= R5;
                when "0110" => RTA <= R6;
                when "0111" => RTA <= R7;
                when "1000" => RTA <= R8;
                when "1001" => RTA <= R9;
                when "1010" => RTA <= R10;
                when "1011" => RTA <= R11;
                when "1100" => RTA <= R12;
                when "1101" => RTA <= R13;
                when "1110" => RTA <= R14;
                when "1111" => RTA <= R15;
                when others => null;
            end case;    
            case IR(6 downto 3) is    -- RS
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;

                    state <= s_inst_div2;
                when s_inst_div2 =>
                              halt <= '0';
                              ROUT0 <= B"000_0000";
                              RESULT2(15 downto 0) <= std_logic_vector(unsigned(RTB) / unsigned(RTA));
                              RESULT2(31 downto 16) <= std_logic_vector(unsigned(RTB) rem unsigned(RTA));

                    state <= s_inst_div3;
                when s_inst_div3 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(2 downto 0) is    -- RT, will have the division result
                when "000" => 
                    R0 <= RESULT2 (15 downto 0);    -- Save the result, low 16 bit
                when "001" => 
                    R1 <= RESULT2 (15 downto 0);    -- 
                when "010" => 
                    R2 <= RESULT2 (15 downto 0);    -- 
                when "011" => 
                    R3 <= RESULT2 (15 downto 0);    -- 
                when "100" => 
                    R4 <= RESULT2 (15 downto 0);    -- 
                when "101" => 
                    R5 <= RESULT2 (15 downto 0);    -- 
                when "110" => 
                    R6 <= RESULT2 (15 downto 0);    -- 
                when "111" => 
                    R7 <= RESULT2 (15 downto 0);    -- 
                when others => null;
            end case;
            case IR(10 downto 7) is    -- RD, will have the remainder
                when "0000" => R0 <= RESULT2 (31 downto 16);
                when "0001" => R1 <= RESULT2 (31 downto 16);
                when "0010" => R2 <= RESULT2 (31 downto 16);
                when "0011" => R3 <= RESULT2 (31 downto 16);
                when "0100" => R4 <= RESULT2 (31 downto 16);
                when "0101" => R5 <= RESULT2 (31 downto 16);
                when "0110" => R6 <= RESULT2 (31 downto 16);
                when "0111" => R7 <= RESULT2 (31 downto 16);
                when "1000" => R8 <= RESULT2 (31 downto 16);
                when "1001" => R9 <= RESULT2 (31 downto 16);
                when "1010" => R10 <= RESULT2 (31 downto 16);
                when "1011" => R11 <= RESULT2 (31 downto 16);
                when "1100" => R12 <= RESULT2 (31 downto 16);
                when "1101" => R13 <= RESULT2 (31 downto 16);
                when "1110" => R14 <= RESULT2 (31 downto 16);
                when "1111" => R15 <= RESULT2 (31 downto 16);
                when others => null;
            end case;    

                    state <= s_fetch;
                when s_inst_and =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTA and RTB accorind to the values in RD and RS section
            --        of the instruction (IR).
            case IR(10 downto 7) is    -- RD
                when "0000" => RTA <= R0;
                when "0001" => RTA <= R1;
                when "0010" => RTA <= R2;
                when "0011" => RTA <= R3;
                when "0100" => RTA <= R4;
                when "0101" => RTA <= R5;
                when "0110" => RTA <= R6;
                when "0111" => RTA <= R7;
                when "1000" => RTA <= R8;
                when "1001" => RTA <= R9;
                when "1010" => RTA <= R10;
                when "1011" => RTA <= R11;
                when "1100" => RTA <= R12;
                when "1101" => RTA <= R13;
                when "1110" => RTA <= R14;
                when "1111" => RTA <= R15;
                when others => null;
            end case;    
            case IR(6 downto 3) is    -- RS
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;

                    state <= s_inst_and2;
                when s_inst_and2 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            RESULT <= '0' & (RTB and RTA);

                    state <= s_inst_and3;
                when s_inst_and3 =>
                              halt <= '0';
            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(2 downto 0) is    -- RT, will have the and result
                when "000" => R0 <= RESULT (15 downto 0);    -- Save the result, low 16 bit
                when "001" => R1 <= RESULT (15 downto 0);    -- 
                when "010" => R2 <= RESULT (15 downto 0);    -- 
                when "011" => R3 <= RESULT (15 downto 0);    -- 
                when "100" => R4 <= RESULT (15 downto 0);    -- 
                when "101" => R5 <= RESULT (15 downto 0);    -- 
                when "110" => R6 <= RESULT (15 downto 0);    -- 
                when "111" => R7 <= RESULT (15 downto 0);    -- 
                when others => null;
            end case;

                    state <= s_fetch;
                when s_inst_or =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTA and RTB accorind to the values in RD and RS section
            --        of the instruction (IR).
            case IR(10 downto 7) is    -- RD
                when "0000" => RTA <= R0;
                when "0001" => RTA <= R1;
                when "0010" => RTA <= R2;
                when "0011" => RTA <= R3;
                when "0100" => RTA <= R4;
                when "0101" => RTA <= R5;
                when "0110" => RTA <= R6;
                when "0111" => RTA <= R7;
                when "1000" => RTA <= R8;
                when "1001" => RTA <= R9;
                when "1010" => RTA <= R10;
                when "1011" => RTA <= R11;
                when "1100" => RTA <= R12;
                when "1101" => RTA <= R13;
                when "1110" => RTA <= R14;
                when "1111" => RTA <= R15;
                when others => null;
            end case;    
            case IR(6 downto 3) is    -- RS
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;

                    state <= s_inst_or2;
                when s_inst_or2 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            RESULT <= '0' & (RTB or RTA);

                    state <= s_inst_or3;
                when s_inst_or3 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(2 downto 0) is    -- RT, will have the and result
                when "000" => R0 <= RESULT (15 downto 0);    -- Save the result, low 16 bit
                when "001" => R1 <= RESULT (15 downto 0);    -- 
                when "010" => R2 <= RESULT (15 downto 0);    -- 
                when "011" => R3 <= RESULT (15 downto 0);    -- 
                when "100" => R4 <= RESULT (15 downto 0);    -- 
                when "101" => R5 <= RESULT (15 downto 0);    -- 
                when "110" => R6 <= RESULT (15 downto 0);    -- 
                when "111" => R7 <= RESULT (15 downto 0);    -- 
                when others => null;
            end case;

                    state <= s_fetch;
                when s_inst_xor =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTA and RTB accorind to the values in RD and RS section
            --        of the instruction (IR).
            case IR(10 downto 7) is    -- RD
                when "0000" => RTA <= R0;
                when "0001" => RTA <= R1;
                when "0010" => RTA <= R2;
                when "0011" => RTA <= R3;
                when "0100" => RTA <= R4;
                when "0101" => RTA <= R5;
                when "0110" => RTA <= R6;
                when "0111" => RTA <= R7;
                when "1000" => RTA <= R8;
                when "1001" => RTA <= R9;
                when "1010" => RTA <= R10;
                when "1011" => RTA <= R11;
                when "1100" => RTA <= R12;
                when "1101" => RTA <= R13;
                when "1110" => RTA <= R14;
                when "1111" => RTA <= R15;
                when others => null;
            end case;    
            case IR(6 downto 3) is    -- RS
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;

                    state <= s_inst_xor2;
                when s_inst_xor2 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            RESULT <= '0' & (RTB xor RTA);

                    state <= s_inst_xor3;
                when s_inst_xor3 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(2 downto 0) is    -- RT, will have the and result
                when "000" => R0 <= RESULT (15 downto 0);    -- Save the result, low 16 bit
                when "001" => R1 <= RESULT (15 downto 0);    -- 
                when "010" => R2 <= RESULT (15 downto 0);    -- 
                when "011" => R3 <= RESULT (15 downto 0);    -- 
                when "100" => R4 <= RESULT (15 downto 0);    -- 
                when "101" => R5 <= RESULT (15 downto 0);    -- 
                when "110" => R6 <= RESULT (15 downto 0);    -- 
                when "111" => R7 <= RESULT (15 downto 0);    -- 
                when others => null;
            end case;

                    state <= s_fetch;
                when s_inst_not =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTB according to the values in RS section
            --        of the instruction (IR).
            case IR(6 downto 3) is    -- RS
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;

                    state <= s_inst_not2;
                when s_inst_not2 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            RESULT <= '0' & (not (RTB));

                    state <= s_inst_not3;
                when s_inst_not3 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(10 downto 7) is    -- RD
                when "0000" => R0 <= RESULT (15 downto 0);
                when "0001" => R1 <= RESULT (15 downto 0);
                when "0010" => R2 <= RESULT (15 downto 0);
                when "0011" => R3 <= RESULT (15 downto 0);
                when "0100" => R4 <= RESULT (15 downto 0);
                when "0101" => R5 <= RESULT (15 downto 0);
                when "0110" => R6 <= RESULT (15 downto 0);
                when "0111" => R7 <= RESULT (15 downto 0);
                when "1000" => R8 <= RESULT (15 downto 0);
                when "1001" => R9 <= RESULT (15 downto 0);
                when "1010" => R10 <= RESULT (15 downto 0);
                when "1011" => R11 <= RESULT (15 downto 0);
                when "1100" => R12 <= RESULT (15 downto 0);
                when "1101" => R13 <= RESULT (15 downto 0);
                when "1110" => R14 <= RESULT (15 downto 0);
                when "1111" => R15 <= RESULT (15 downto 0);
                when others => null;
            end case;    

                    state <= s_fetch;
                when s_inst_srl =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTB according to the values in RS section
            --        of the instruction (IR).
            case IR(6 downto 3) is    -- RS
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;

                    state <= s_inst_srl2;
                when s_inst_srl2 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Shift Right RTB    
            RESULT <= ('0' & ('0' & RTB(RTB'left downto 1)));

                    state <= s_inst_srl3;
                when s_inst_srl3 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(10 downto 7) is    -- RD
                when "0000" => R0 <= RESULT (15 downto 0);
                when "0001" => R1 <= RESULT (15 downto 0);
                when "0010" => R2 <= RESULT (15 downto 0);
                when "0011" => R3 <= RESULT (15 downto 0);
                when "0100" => R4 <= RESULT (15 downto 0);
                when "0101" => R5 <= RESULT (15 downto 0);
                when "0110" => R6 <= RESULT (15 downto 0);
                when "0111" => R7 <= RESULT (15 downto 0);
                when "1000" => R8 <= RESULT (15 downto 0);
                when "1001" => R9 <= RESULT (15 downto 0);
                when "1010" => R10 <= RESULT (15 downto 0);
                when "1011" => R11 <= RESULT (15 downto 0);
                when "1100" => R12 <= RESULT (15 downto 0);
                when "1101" => R13 <= RESULT (15 downto 0);
                when "1110" => R14 <= RESULT (15 downto 0);
                when "1111" => R15 <= RESULT (15 downto 0);
                when others => null;
            end case;    

                    state <= s_fetch;
                when s_inst_sll =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTB according to the values in RS section
            --        of the instruction (IR).
            case IR(6 downto 3) is    -- RS
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;

                    state <= s_inst_sll2;
                when s_inst_sll2 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Shift Left A    
            RESULT <= ('0' & (RTB(RTB'left - 1 downto 0) & '0'));

                    state <= s_inst_sll3;
                when s_inst_sll3 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(10 downto 7) is    -- RD
                when "0000" => R0 <= RESULT (15 downto 0);
                when "0001" => R1 <= RESULT (15 downto 0);
                when "0010" => R2 <= RESULT (15 downto 0);
                when "0011" => R3 <= RESULT (15 downto 0);
                when "0100" => R4 <= RESULT (15 downto 0);
                when "0101" => R5 <= RESULT (15 downto 0);
                when "0110" => R6 <= RESULT (15 downto 0);
                when "0111" => R7 <= RESULT (15 downto 0);
                when "1000" => R8 <= RESULT (15 downto 0);
                when "1001" => R9 <= RESULT (15 downto 0);
                when "1010" => R10 <= RESULT (15 downto 0);
                when "1011" => R11 <= RESULT (15 downto 0);
                when "1100" => R12 <= RESULT (15 downto 0);
                when "1101" => R13 <= RESULT (15 downto 0);
                when "1110" => R14 <= RESULT (15 downto 0);
                when "1111" => R15 <= RESULT (15 downto 0);
                when others => null;
            end case;    

                    state <= s_fetch;
                when s_inst_ld =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the MEM_ADDR according to the values in RS
            --        of the instruction (IR).
            case IR(6 downto 3) is    -- RS                    
                when "0000" => MEM_ADDR <= R0;
                when "0001" => MEM_ADDR <= R1;
                when "0010" => MEM_ADDR <= R2;
                when "0011" => MEM_ADDR <= R3;
                when "0100" => MEM_ADDR <= R4;
                when "0101" => MEM_ADDR <= R5;
                when "0110" => MEM_ADDR <= R6;
                when "0111" => MEM_ADDR <= R7;
                when "1000" => MEM_ADDR <= R8;
                when "1001" => MEM_ADDR <= R9;
                when "1010" => MEM_ADDR <= R10;
                when "1011" => MEM_ADDR <= R11;
                when "1100" => MEM_ADDR <= R12;
                when "1101" => MEM_ADDR <= R13;
                when "1110" => MEM_ADDR <= R14;
                when "1111" => MEM_ADDR <= R15;
                when others => null;
            end case;    
            MEM_WR <= "0";
            MEM_EN <= '1';

                    state <= s_inst_ld2;
                when s_inst_ld2 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- wait one cycle to fetch the data in memory 

                    state <= s_inst_ld3;
                when s_inst_ld3 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            case IR(10 downto 7) is    -- RD
                when "0000" => R0 <= MEM_DATA_OUT;
                when "0001" => R1 <= MEM_DATA_OUT;
                when "0010" => R2 <= MEM_DATA_OUT;
                when "0011" => R3 <= MEM_DATA_OUT;
                when "0100" => R4 <= MEM_DATA_OUT;
                when "0101" => R5 <= MEM_DATA_OUT;
                when "0110" => R6 <= MEM_DATA_OUT;
                when "0111" => R7 <= MEM_DATA_OUT;
                when "1000" => R8 <= MEM_DATA_OUT;
                when "1001" => R9 <= MEM_DATA_OUT;
                when "1010" => R10 <= MEM_DATA_OUT;
                when "1011" => R11 <= MEM_DATA_OUT;
                when "1100" => R12 <= MEM_DATA_OUT;
                when "1101" => R13 <= MEM_DATA_OUT;
                when "1110" => R14 <= MEM_DATA_OUT;
                when "1111" => R15 <= MEM_DATA_OUT;
                when others => null;
            end case;    

                    state <= s_fetch;
                when s_inst_st =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the MEM_ADDR according to the values in RS
            --        of the instruction (IR).
            case IR(10 downto 7) is    -- RD
                when "0000" => MEM_ADDR <= R0;
                when "0001" => MEM_ADDR <= R1;
                when "0010" => MEM_ADDR <= R2;
                when "0011" => MEM_ADDR <= R3;
                when "0100" => MEM_ADDR <= R4;
                when "0101" => MEM_ADDR <= R5;
                when "0110" => MEM_ADDR <= R6;
                when "0111" => MEM_ADDR <= R7;
                when "1000" => MEM_ADDR <= R8;
                when "1001" => MEM_ADDR <= R9;
                when "1010" => MEM_ADDR <= R10;
                when "1011" => MEM_ADDR <= R11;
                when "1100" => MEM_ADDR <= R12;
                when "1101" => MEM_ADDR <= R13;
                when "1110" => MEM_ADDR <= R14;
                when "1111" => MEM_ADDR <= R15;
                when others => null;
            end case;    
            case IR(6 downto 3) is    -- RS                    
                when "0000" => MEM_DATA_IN <= R0;
                when "0001" => MEM_DATA_IN <= R1;
                when "0010" => MEM_DATA_IN <= R2;
                when "0011" => MEM_DATA_IN <= R3;
                when "0100" => MEM_DATA_IN <= R4;
                when "0101" => MEM_DATA_IN <= R5;
                when "0110" => MEM_DATA_IN <= R6;
                when "0111" => MEM_DATA_IN <= R7;
                when "1000" => MEM_DATA_IN <= R8;
                when "1001" => MEM_DATA_IN <= R9;
                when "1010" => MEM_DATA_IN <= R10;
                when "1011" => MEM_DATA_IN <= R11;
                when "1100" => MEM_DATA_IN <= R12;
                when "1101" => MEM_DATA_IN <= R13;
                when "1110" => MEM_DATA_IN <= R14;
                when "1111" => MEM_DATA_IN <= R15;
                when others => null;
            end case;    
            MEM_WR <= "1";
            MEM_EN <= '1';

                    state <= s_inst_st2;
                when s_inst_st2 =>
            halt <= '0';
--            ROUT0 <= B"000_0000";
            -- wait one cycle to write to the memory
                  
                    state <= s_inst_st3;
                when s_inst_st3 =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Update the memoery address
            MEM_ADDR <= PC;    
            MEM_WR <= "0";
            MEM_EN <= '1';

                    state <= s_fetch;
                when s_inst_cmp =>
                              halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTA according to the values in RD section
            --        of the instruction (IR).
            case IR(10 downto 7) is    -- RD
                when "0000" => RTA <= R0;
                when "0001" => RTA <= R1;
                when "0010" => RTA <= R2;
                when "0011" => RTA <= R3;
                when "0100" => RTA <= R4;
                when "0101" => RTA <= R5;
                when "0110" => RTA <= R6;
                when "0111" => RTA <= R7;
                when "1000" => RTA <= R8;
                when "1001" => RTA <= R9;
                when "1010" => RTA <= R10;
                when "1011" => RTA <= R11;
                when "1100" => RTA <= R12;
                when "1101" => RTA <= R13;
                when "1110" => RTA <= R14;
                when "1111" => RTA <= R15;
                when others => null;
            end case;    
            case IR(6 downto 3) is    -- RS
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;    

                    state <= s_inst_cmp2;
                when s_inst_cmp2 =>
                              if  RTA = RTB then
                FLAGR(0) <= '1';    -- Set Zero FLAG
            else     
                FLAGR(0) <= '0';    -- Unset Zero FLAG
            end if;    
            -- Update  the memory address with PC value so next clock 
            --        we will have the right address fetched.
            MEM_ADDR <= PC;

                    state <= s_fetch;
                when s_inst_jmp =>
                     halt <= '1';
--                          ROUT0 <= B"000_0000";
                          -- Adjust the PC value by adding the jump adddress
                          -- * Jump address must be in two's complement form
                          if IR(10) = '0' then -- Check the sign of jump address
                              TMP <= PC + ("00000" & IR (10 downto 0));
                          else
                              TMP <= PC + ("11111" & IR (10 downto 0));
                          end if;
                    state <= s_inst_jmp2;
                when s_inst_jmp2 =>
                     halt <= '0';
--                          ROUT0 <= B"000_0000";
                          PC <= TMP;
                          -- Update  the memory address with PC value so next clock 
                          --        we will have the right address fetched.
                          MEM_ADDR <= TMP;  
                    state <= s_fetch;
                when s_inst_jz =>
            halt <= '0';
--            ROUT0 <= B"000_0000";
            if FLAGR(0) = '1' then -- Check the Zero Flag
                -- Adjust the PC value  by adding the jump adddress
                -- * Jump address must be in two's complement form
                if IR(10) = '0' then -- Check the sign of jump address
                    TMP <= PC + ("00000" & IR (10 downto 0));
                else
                    TMP <= PC + ("11111" & IR (10 downto 0));
                end if;    
            end if;
                
                    state <= s_inst_jz2;
                when s_inst_jz2 =>
            halt <= '0';
--            ROUT0 <= B"000_0000";
            PC <= TMP;
            -- Update  the memory address with PC value so next clock 
            --        we will have the right address fetched.
            MEM_ADDR <= TMP;    
                
                    state <= s_fetch;
                when s_inst_jnz =>
            halt <= '0';
--            ROUT0 <= B"000_0000";
            if FLAGR(0) = '0' then -- Check the Zero Flag
                -- Adjust the PC value  by adding the jump adddress
                -- * Jump address must be in two's complement form
                if IR(10) = '0' then -- Check the sign of jump address
                    TMP <= PC + ("00000" & IR (10 downto 0));
                else
                    TMP <= PC + ("11111" & IR (10 downto 0));
                end if;    
            end if;
                
                    state <= s_inst_jnz2;
                when s_inst_jnz2 =>
            halt <= '0';
--            ROUT0 <= B"000_0000";
            PC <= TMP;
            -- Update  the memory address with PC value so next clock 
            --        we will have the right address fetched.
            MEM_ADDR <= PC;    
                
                    state <= s_fetch;
                when s_inst_imd =>
                    halt <= '0';
                            --ROUT0 <= B"000_0111";
                            -- Fetch the next 16-bit, PC is already increased in fetch2 state
                            MEM_ADDR <= PC;
                            MEM_EN <= '1';
                    state <= s_inst_imd2;
                when s_inst_imd2 =>
                     halt <= '0';
--                          ROUT0 <= B"000_1000";
                          TMP <= PC + '1'; 
                    state <= s_inst_imd3;
                when s_inst_imd3 =>
                     halt <= '0';
--                          ROUT0 <= B"000_1001";
                          PC <= TMP;
                          MEM_EN <= '0';
                    state <= s_inst_imd4;
                when s_inst_imd4 =>
                     halt <= '0';
--                          ROUT0 <= B"000_1010";
                          case IR(10 downto 7) is -- RD
                              when "0000" => R0 <= MEM_DATA_OUT;
                              when "0001" => R1 <= MEM_DATA_OUT;
                              when "0010" => R2 <= MEM_DATA_OUT;
                              when "0011" => R3 <= MEM_DATA_OUT;
                              when "0100" => R4 <= MEM_DATA_OUT;
                              when "0101" => R5 <= MEM_DATA_OUT;
                              when "0110" => R6 <= MEM_DATA_OUT;
                              when "0111" => R7 <= MEM_DATA_OUT;
                              when "1000" => R8 <= MEM_DATA_OUT;
                              when "1001" => R9 <= MEM_DATA_OUT;
                              when "1010" => R10 <= MEM_DATA_OUT;
                              when "1011" => R11 <= MEM_DATA_OUT;
                              when "1100" => R12 <= MEM_DATA_OUT;
                              when "1101" => R13 <= MEM_DATA_OUT;
                              when "1110" => R14 <= MEM_DATA_OUT;
                              when "1111" => R15 <= MEM_DATA_OUT;
                              when others => null;
                          end case; 
                    state <= s_fetch;
                when s_inst_out =>
            halt <= '0';
--            ROUT0 <= B"000_0000";
            -- Set the RTA according to the values in RD field
            --        of the instruction (IR).
            case IR(6 downto 3) is    -- RS
                when "0000" => RTA <= R0;
                when "0001" => RTA <= R1;
                when "0010" => RTA <= R2;
                when "0011" => RTA <= R3;
                when "0100" => RTA <= R4;
                when "0101" => RTA <= R5;
                when "0110" => RTA <= R6;
                when "0111" => RTA <= R7;
                when "1000" => RTA <= R8;
                when "1001" => RTA <= R9;
                when "1010" => RTA <= R10;
                when "1011" => RTA <= R11;
                when "1100" => RTA <= R12;
                when "1101" => RTA <= R13;
                when "1110" => RTA <= R14;
                when "1111" => RTA <= R15;
                when others => null;
            end case;    
            case IR(10 downto 7) is -- RD
                when "0000" => RTB <= R0;
                when "0001" => RTB <= R1;
                when "0010" => RTB <= R2;
                when "0011" => RTB <= R3;
                when "0100" => RTB <= R4;
                when "0101" => RTB <= R5;
                when "0110" => RTB <= R6;
                when "0111" => RTB <= R7;
                when "1000" => RTB <= R8;
                when "1001" => RTB <= R9;
                when "1010" => RTB <= R10;
                when "1011" => RTB <= R11;
                when "1100" => RTB <= R12;
                when "1101" => RTB <= R13;
                when "1110" => RTB <= R14;
                when "1111" => RTB <= R15;
                when others => null;
            end case;    
                
                    state <= s_inst_out2;
                when s_inst_out2 =>
        halt <= '0';
                -- Update the memoery address
                MEM_ADDR <= PC;
                -- For now we support only two output registers.
                -- "0000" = 0 = ROUT0
                -- "0001" = 1 = ROUT1
                case RTB(3 downto 0) is    
                    when "0000" => ROUT0 <= RTA (6 downto 0);
                    --when "0001" => ROUT1 <= RTA (6 downto 0);
                    when others => null;
                end case;
            ROUT0 <= RTA (6 downto 0);
                
                    state <= s_fetch;
                when s_inst_push =>
                halt <= '0';
--                ROUT0 <= B"000_0000";
                -- Set the RTA according to the values in RD field
                --        of the instruction (IR).
                case IR(10 downto 7) is    -- RD
                    when "0000" => RTA <= R0;
                    when "0001" => RTA <= R1;
                    when "0010" => RTA <= R2;
                    when "0011" => RTA <= R3;
                    when "0100" => RTA <= R4;
                    when "0101" => RTA <= R5;
                    when "0110" => RTA <= R6;
                    when "0111" => RTA <= R7;
                    when "1000" => RTA <= R8;
                    when "1001" => RTA <= R9;
                    when "1010" => RTA <= R10;
                    when "1011" => RTA <= R11;
                    when "1100" => RTA <= R12;
                    when "1101" => RTA <= R13;
                    when "1110" => RTA <= R14;
                    when "1111" => RTA <= R15;
                    when others => null;
                end case;    
                
                    if SP = B"10000_0000_0000_0000" then    -- Stack Overflow
                        state <= s_halt;
                    else 
                        state <= s_inst_push2;
                    end if;    
                when s_inst_push2 =>
                                halt <= '0';
--                ROUT0 <= B"000_0000";
                MEM_ADDR <= SS - SP(15 downto 0);
                MEM_DATA_IN <= RTA;        
                TMP17 <= SP + '1';
                MEM_WR <= "1";
                MEM_EN <= '1';

                        state <= s_inst_push3;
                when s_inst_push3 =>
                                halt <= '0';
--                ROUT0 <= B"000_0000";
                SP <= TMP17;
                -- wait one  cycle to write to stack memory

                        state <= s_inst_push4;
                when s_inst_push4 =>
                                halt <= '0';
--                ROUT0 <= B"000_0000";
                -- Update the memoery address
                MEM_ADDR <= PC;    
                MEM_WR <= "0";
                MEM_EN <= '1';

                        state <= s_fetch;
                when s_inst_pop =>
                halt <= '0';
--                ROUT0 <= B"000_0000";
                MEM_ADDR <= SS - (SP(15 downto 0) - '1');
                MEM_WR <= "0";    -- read
                MEM_EN <= '1';
                    if SP = B"00000_0000_0000_0000" then    -- Stack Underflow
                        state <= s_halt;
                    else 
                        state <= s_inst_pop2;
                    end if;    
                when s_inst_pop2 =>
                                halt <= '0';
--                ROUT0 <= B"000_0000";
                -- wait one cycle to read the stack memory 

                        state <= s_inst_pop3;
                when s_inst_pop3 =>
                                halt <= '0';
--                ROUT0 <= B"000_0000";
                -- Update the memoery address
                MEM_ADDR <= PC;
                case IR(10 downto 7) is    -- RD
                    when "0000" => R0 <= MEM_DATA_OUT;
                    when "0001" => R1 <= MEM_DATA_OUT;
                    when "0010" => R2 <= MEM_DATA_OUT;
                    when "0011" => R3 <= MEM_DATA_OUT;
                    when "0100" => R4 <= MEM_DATA_OUT;
                    when "0101" => R5 <= MEM_DATA_OUT;
                    when "0110" => R6 <= MEM_DATA_OUT;
                    when "0111" => R7 <= MEM_DATA_OUT;
                    when "1000" => R8 <= MEM_DATA_OUT;
                    when "1001" => R9 <= MEM_DATA_OUT;
                    when "1010" => R10 <= MEM_DATA_OUT;
                    when "1011" => R11 <= MEM_DATA_OUT;
                    when "1100" => R12 <= MEM_DATA_OUT;
                    when "1101" => R13 <= MEM_DATA_OUT;
                    when "1110" => R14 <= MEM_DATA_OUT;
                    when "1111" => R15 <= MEM_DATA_OUT;
                    when others => null;
                end case;    
                SP <= SP - '1';

                    state <= s_fetch;
                when s_inst_clrc =>
                                halt <= '0';
--                ROUT0 <= B"000_0000";
            -- Update the memoery address
                MEM_ADDR <= PC;
                FLAGR(1) <= '0';    -- Clear carry 

                    state <= s_fetch;
                when s_inst_setc =>
                                halt <= '0';
--                ROUT0 <= B"000_0000";
                -- Update the memoery address
                MEM_ADDR <= PC;
                FLAGR(1) <= '1';    -- Set carry 

                    state <= s_fetch;
                when s_inst_call =>
                halt <= '0';
--                ROUT0 <= B"000_0000";
                -- Save the return address
                RETADDR <= PC;
                -- Set the PC according to the values in RD field
                --        of the instruction (IR).
                case IR(10 downto 7) is    -- RD
                    when "0000" => PC <= R0;
                    when "0001" => PC <= R1;
                    when "0010" => PC <= R2;
                    when "0011" => PC <= R3;
                    when "0100" => PC <= R4;
                    when "0101" => PC <= R5;
                    when "0110" => PC <= R6;
                    when "0111" => PC <= R7;
                    when "1000" => PC <= R8;
                    when "1001" => PC <= R9;
                    when "1010" => PC <= R10;
                    when "1011" => PC <= R11;
                    when "1100" => PC <= R12;
                    when "1101" => PC <= R13;
                    when "1110" => PC <= R14;
                    when "1111" => PC <= R15;
                    when others => null;
                end case;
                
                    state <= s_inst_call2;
                when s_inst_call2 =>
                                halt <= '0';
--                ROUT0 <= B"000_0000";
                -- wait for PC to be updated. Then:
                -- Update the memoery address
                MEM_ADDR <= PC;

                    state <= s_fetch;
                when s_inst_ret =>
                halt <= '0';
--                ROUT0 <= B"000_0000";
                -- Restore the PC
                PC <= RETADDR;
                
                    state <= s_inst_ret2;
                when s_inst_ret2 =>
                halt <= '0';
--                ROUT0 <= B"000_0000";
                -- wait for PC to be updated. Then:
                -- Update the memoery address
                MEM_ADDR <= PC;
                
                    state <= s_fetch;
                when s_inst_nop =>
                    halt <= '0';
                                --ROUT0 <= B"000_1011";
                                MEM_ADDR <= PC;
                                 halt <= '1';
                    state <= s_inst_nop2;
                when s_inst_nop2 =>
                     halt <= '0';
                             --ROUT0 <= B"000_1100";
                    state <= s_fetch;
                when s_halt =>
                      halt <= '1';
--                            ROUT0 <= B"000_1101";
                             PC <= B"0000_0000_0000_0000";
                            IR <= B"0000_0000_0000_0000";
             --               SP <= B"00000_0000_0000_0000";
             --               SS <= B"1111_1111_1111_1111"; -- End of RAM block
             --               FLAGR <= B"0000_0000_0000_0000";
                            MEM_ADDR <= B"0000_0000_0000_0000";
                            MEM_WR <= "0";
                            MEM_EN <= '0';                                 
                    state <= s_halt;
                when others => 
                     halt <= '0';
                              --ROUT0 <= B"111_1111";
                              PC <= B"0000_0000_0000_0000";
                              IR <= B"0000_0000_0000_0000";
              --                SP <= B"00000_0000_0000_0000";
              --                SS <= B"1111_1111_1111_1111"; -- End of RAM block
              --                FLAGR <= B"0000_0000_0000_0000";
                              MEM_ADDR <= B"0000_0000_0000_0000";
                              MEM_WR <= "0";
                              MEM_EN <= '0';
                             MEM_EN <= '0';
                    state <= s_halt;
            end case;
        end if;    
    end if;
end process;

  
--    	-- This process is responsible to set the control signal and registers
--    output_logic: process (state) begin
----        output_logic: process (state) begin
--        case state is
--            when s_start =>
--                -- Reset all signals
----                TMP17 <= B"0_0000_0000_0000_0000";
--                TMP <= B"0000_0000_0000_0000";
--                PC <= B"0000_0000_0000_0000";
--                    IR <= B"0000_0000_0000_0000";
----                SP <= B"0_0000_0000_0000_0000";    
----                SS <= B"1111_1111_1111_1111"; -- End of RAM block
----                RETADDR <= B"0000_0000_0000_0000";
----                RD <= B"0000"; 
----                RS <= B"0000";
----                RT <= B"0000";
----                RTA <= B"0000_0000_0000_0000";
----                RTB <= B"0000_0000_0000_0000";
----                RESULT <= B"0_0000_0000_0000_0000";
----                RESULT2 <= B"0000_0000_0000_0000_0000_0000_0000_0000";
----                FLAGR <= B"0000_0000_0000_0000";
--                MEM_DATA_IN <= B"0000_0000_0000_0000";
--                MEM_ADDR <= B"0000_0000_0000_0000";
--                MEM_WR <= "0";
--                MEM_EN <= '0';
--                halt <= '0';
--                ROUT0 <= B"000_0000";
--                R0 <= B"0000_0000_0000_0000";
--                R1 <= B"0000_0000_0000_0000";
--                R2 <= B"0000_0000_0000_0000";
--                R3 <= B"0000_0000_0000_0000";
--                R4 <= B"0000_0000_0000_0000";
--                R5 <= B"0000_0000_0000_0000";
--                R6 <= B"0000_0000_0000_0000";
--                R7 <= B"0000_0000_0000_0000";
--                R8 <= B"0000_0000_0000_0000";
--                R9 <= B"0000_0000_0000_0000";
--                R10 <= B"0000_0000_0000_0000";
--                R11 <= B"0000_0000_0000_0000";
--                R12 <= B"0000_0000_0000_0000";
--                R13 <= B"0000_0000_0000_0000";
--                R14 <= B"0000_0000_0000_0000";
--                R15 <= B"0000_0000_0000_0000";
----                ROUT1 <= (others => '0');
--            when s_fetch => -- Z2
--                MEM_ADDR <= PC;
--                MEM_WR <= "0";
--                MEM_EN <= '1';
--                halt <= '0';
--                ROUT0 <= B"000_0001";
--            when s_fetch2 =>  -- Z3
--                -- Update the PC to fetch the next instruction
--                TMP <= PC;
--                MEM_EN <= '1'; 
--                halt <= '0';
--                ROUT0 <= B"000_0010";
--            when s_fetch3 => -- Z4
--                MEM_EN <= '1';
--                PC <= TMP  + '1';
--                -- Wait for the data be present on MEM_DATA_OUT 
--                halt <= '0';
--                ROUT0 <= B"000_0011";
--            when s_decode => -- Z5
--                MEM_EN <= '1';
--                IR <= MEM_DATA_OUT;
--                halt <= '0';
--                ROUT0 <= B"000_0100";
--            when s_decode2 => -- Z6
--                MEM_EN <= '1';
--                halt <= '0';
--                ROUT0 <= B"000_0101";
--            when s_decode3 =>
--                MEM_EN <= '0';
--                halt <= '0';
--                ROUT0 <= B"000_0110";
----            when s_inst_mov =>
----                halt <= '0';
----                ROUT0 <= B"100_0000";
----        -- Set the RTA according to the values in RS
----            --        of the instruction (IR).
----            case IR(6 downto 3) is    -- RS                    
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => RTA <= B"0000_0000_0000_0000";
----            end case;    
----            when s_inst_mov2 =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- Update the memoery address
----                MEM_ADDR <= PC;    
----            -- Copy into RD according to the values in RD
----            --        of the instruction (IR).
----            case IR(10 downto 7) is    -- RD
----                when "0000" => R0 <= RTA;
----                when "0001" => R1 <= RTA;
----                when "0010" => R2 <= RTA;
----                when "0011" => R3 <= RTA;
----                when "0100" => R4 <= RTA;
----                when "0101" => R5 <= RTA;
----                when "0110" => R6 <= RTA;
----                when "0111" => R7 <= RTA;
----                when "1000" => R8 <= RTA;
----                when "1001" => R9 <= RTA;
----                when "1010" => R10 <= RTA;
----                when "1011" => R11 <= RTA;
----                when "1100" => R12 <= RTA;
----                when "1101" => R13 <= RTA;
----                when "1110" => R14 <= RTA;
----                when "1111" => R15 <= RTA;
----                when others => null ;
----            end case;    
----        when s_inst_add =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- RT <- (RD + RS)    
----            case IR(10 downto 7) is    -- RD
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;    
----        when s_inst_add2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            RESULT <= ('0' & RTA) + -- RD
----                         ('0' & RTB);     -- RS
----        when s_inst_add3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(2 downto 0) is    -- RT
----                when "000" => 
----                    R0 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "001" => 
----                    R1 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "010" => 
----                    R2 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "011" => 
----                    R3 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "100" => 
----                    R4 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "101" => 
----                    R5 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "110" => 
----                    R6 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "111" => 
----                    R7 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when others => null;
----            end case;
----        when s_inst_adc =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----        -- RT <- (RD + RS)    
----            case IR(10 downto 7) is    -- RD
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;    
----        when s_inst_adc2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            RESULT <= ('0' & RTA) + -- RD
----                         ('0' & RTB) + -- RS
----                         FLAGR(1);         -- Carry
----        when s_inst_adc3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(2 downto 0) is    -- RT
----                when "000" => 
----                    R0 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "001" => 
----                    R1 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "010" => 
----                    R2 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry  flag    
----                when "011" => 
----                    R3 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "100" => 
----                    R4 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "101" => 
----                    R5 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "110" => 
----                    R6 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when "111" => 
----                    R7 <= RESULT (15 downto 0);    -- Save  the addition result
----                    FLAGR(1) <= RESULT (16);        --    Set Carry flag    
----                when others => null;
----            end case;
----        when s_inst_sub =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----        -- RT <- (RS - RD)    
----            case IR(10 downto 7) is    -- RD = RTA
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS = RTB
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;    
----        when s_inst_sub2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            RESULT <= (('0' & RTB) + -- RS
----                         ('0' & (not(RTA) + '1')));     -- RS
                         
----        when s_inst_sub3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(2 downto 0) is    -- RT
----                when "000" => 
----                    R0 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "001" => 
----                    R1 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "010" => 
----                    R2 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "011" => 
----                    R3 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "100" => 
----                    R4 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "101" => 
----                    R5 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "110" => 
----                    R6 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "111" => 
----                    R7 <= RESULT (15 downto 0);    -- Save  the addition result
----                when others => null;
----            end case;    
----            if RESULT(15 downto 0) = 0 then
----                FLAGR(0) <= '1';        --    Set the zero flag    
----            else     
----                FLAGR(0) <= '0';        --    Unset the zero flag    
----            end if;    
----        when s_inst_sbc =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----        -- RT <- (RS - RD - Carry)    
----            case IR(10 downto 7) is    -- RD
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;    
----        when s_inst_sbc2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----                -- Calculate RTB - RTA - Carry = RTB - (RTA + Carry)
----                --                                         = RS - (RD + Carry)        
----                RESULT <= ('0' & RTB) + -- RS
----                         ('0' & not(RTA + FLAGR(1)) + '1');     -- RD + Carry
----        when s_inst_sbc3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(2 downto 0) is    -- RT
----                when "000" => 
----                    R0 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "001" => 
----                    R1 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "010" => 
----                    R2 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "011" => 
----                    R3 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "100" => 
----                    R4 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "101" => 
----                    R5 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "110" => 
----                    R6 <= RESULT (15 downto 0);    -- Save  the addition result
----                when "111" => 
----                    R7 <= RESULT (15 downto 0);    -- Save  the addition result
----                when others => null;
----            end case;    
----            if RESULT(15 downto 0) = 0 then
----                FLAGR(0) <= '1';        --    Set the zero flag    
----            else     
----                FLAGR(0) <= '0';        --    Unset the zero flag    
----            end if;    

----        when s_inst_inc =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            case IR(10 downto 7) is        -- RD 
----                when "0000" => 
----                    RESULT <= ('0' & R0) + '1';
----                when "0001" => 
----                    RESULT <= ('0' & R1) + '1';
----                when "0010" => 
----                    RESULT <= ('0' & R2) + '1';
----                when "0011" => 
----                    RESULT <= ('0' & R3) + '1';
----                when "0100" => 
----                    RESULT <= ('0' & R4) + '1';
----                when "0101" => 
----                    RESULT <= ('0' & R5) + '1';
----                when "0110" => 
----                    RESULT <= ('0' & R6) + '1';
----                when "0111" => 
----                    RESULT <= ('0' & R7) + '1';
----                when "1000" => 
----                    RESULT <= ('0' & R8) + '1';
----                when "1001" => 
----                    RESULT <= ('0' & R9) + '1';
----                when "1010" => 
----                    RESULT <= ('0' & R10) + '1';
----                when "1011" => 
----                    RESULT <= ('0' & R11) + '1';
----                when "1100" => 
----                    RESULT <= ('0' & R12) + '1';
----                when "1101" => 
----                    RESULT <= ('0' & R13) + '1';
----                when "1110" => 
----                    RESULT <= ('0' & R14) + '1';
----                when "1111" => 
----                    RESULT <= ('0' & R15) + '1';
----                when others => null;
----            end case;
----        when s_inst_inc2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(10 downto 7) is        -- RD 
----                when "0000" => 
----                    R0 <= RESULT(15 downto 0);
----                    if RESULT(16) <= '1' then
----                        FLAGR(2) <= '1';    -- Set Overflow flag
----                    end if;    
----                when "0001" => 
----                    R1 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "0010" => 
----                    R2 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "0011" => 
----                    R3 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "0100" => 
----                    R4 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "0101" => 
----                    R5 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "0110" => 
----                    R6 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "0111" => 
----                    R7 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "1000" => 
----                    R8 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "1001" => 
----                    R9 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "1010" => 
----                    R10 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "1011" => 
----                    R11 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "1100" => 
----                    R12 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "1101" => 
----                    R13 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "1110" => 
----                    R14 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when "1111" => 
----                    R15 <= RESULT(15 downto 0);
----                    if RESULT(16) = '1' then
----                        FLAGR(2) <= '1';
----                    end if;    
----                when others => null;
----            end case;
----        when s_inst_dec =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            case IR(10 downto 7) is        -- RD 
----                when "0000" => RESULT <= ('0' & R0) - '1';
----                when "0001" => RESULT <= ('0' & R1) - '1';
----                when "0010" => RESULT <= ('0' & R2) - '1';
----                when "0011" => RESULT <= ('0' & R3) - '1';
----                when "0100" => RESULT <= ('0' & R4) - '1';
----                when "0101" => RESULT <= ('0' & R5) - '1';
----                when "0110" => RESULT <= ('0' & R6) - '1';
----                when "0111" => RESULT <= ('0' & R7) - '1';
----                when "1000" => RESULT <= ('0' & R8) - '1';
----                when "1001" => RESULT <= ('0' & R9) - '1';
----                when "1010" => RESULT <= ('0' & R10) - '1';
----                when "1011" => RESULT <= ('0' & R11) - '1';
----                when "1100" => RESULT <= ('0' & R12) - '1';
----                when "1101" => RESULT <= ('0' & R13) - '1';
----                when "1110" => RESULT <= ('0' & R14) - '1';
----                when "1111" => RESULT <= ('0' & R15) - '1';
----                when others => null;
----            end case;
----        when s_inst_dec2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(10 downto 7) is        -- RD 
----                when "0000" => 
----                    R0 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "0001" => 
----                    R1 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "0010" => 
----                    R2 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "0011" => 
----                    R3 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "0100" => 
----                    R4 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "0101" => 
----                    R5 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "0110" => 
----                    R6 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "0111" => 
----                    R7 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "1000" => 
----                    R8 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "1001" => 
----                    R9 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "1010" => 
----                    R10 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "1011" => 
----                    R11 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "1100" => 
----                    R12 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "1101" => 
----                    R13 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "1110" => 
----                    R14 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when "1111" => 
----                    R15 <= RESULT(15 downto 0);
----                    if RESULT(15 downto 0) = 0  then
----                        FLAGR(0) <= '1';    -- Set the Zero Flag
----                    end if;    
----                when others => null;
----            end case;
----        when s_inst_mul =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTA and RTB accorind to the values in RD and RS section
----            --        of the instruction (IR).
----            case IR(10 downto 7) is    -- RD
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;
----        when s_inst_mul2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            RESULT2 <= std_logic_vector(unsigned(RTA) * unsigned(RTB));
----        when s_inst_mul3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(2 downto 0) is    -- RT
----                when "000" => 
----                    R0 <= RESULT2 (31 downto 16);    -- Save the result, high 16 bit
----                when "001" => 
----                    R1 <= RESULT2 (31 downto 16);    -- 
----                when "010" => 
----                    R2 <= RESULT2 (31 downto 16);    -- 
----                when "011" => 
----                    R3 <= RESULT2 (31 downto 16);    -- 
----                when "100" => 
----                    R4 <= RESULT2 (31 downto 16);    -- 
----                when "101" => 
----                    R5 <= RESULT2 (31 downto 16);    -- 
----                when "110" => 
----                    R6 <= RESULT2 (31 downto 16);    -- 
----                when "111" => 
----                    R7 <= RESULT2 (31 downto 16);    -- 
----                when others => null;
----            end case;
----            case IR(10 downto 7) is    -- RD
----                when "0000" => R0 <= RESULT2 (15 downto 0);
----                when "0001" => R1 <= RESULT2 (15 downto 0);
----                when "0010" => R2 <= RESULT2 (15 downto 0);
----                when "0011" => R3 <= RESULT2 (15 downto 0);
----                when "0100" => R4 <= RESULT2 (15 downto 0);
----                when "0101" => R5 <= RESULT2 (15 downto 0);
----                when "0110" => R6 <= RESULT2 (15 downto 0);
----                when "0111" => R7 <= RESULT2 (15 downto 0);
----                when "1000" => R8 <= RESULT2 (15 downto 0);
----                when "1001" => R9 <= RESULT2 (15 downto 0);
----                when "1010" => R10 <= RESULT2 (15 downto 0);
----                when "1011" => R11 <= RESULT2 (15 downto 0);
----                when "1100" => R12 <= RESULT2 (15 downto 0);
----                when "1101" => R13 <= RESULT2 (15 downto 0);
----                when "1110" => R14 <= RESULT2 (15 downto 0);
----                when "1111" => R15 <= RESULT2 (15 downto 0);
----                when others => null;
----            end case;                
----        when s_inst_div =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTA and RTB accorind to the values in RD and RS section
----            --        of the instruction (IR).
----            case IR(10 downto 7) is    -- RD
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;
----        when s_inst_div2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            RESULT2(15 downto 0) <= std_logic_vector(unsigned(RTB) / unsigned(RTA));
----            RESULT2(31 downto 16) <= std_logic_vector(unsigned(RTB) rem unsigned(RTA));
----        when s_inst_div3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(2 downto 0) is    -- RT, will have the division result
----                when "000" => 
----                    R0 <= RESULT2 (15 downto 0);    -- Save the result, low 16 bit
----                when "001" => 
----                    R1 <= RESULT2 (15 downto 0);    -- 
----                when "010" => 
----                    R2 <= RESULT2 (15 downto 0);    -- 
----                when "011" => 
----                    R3 <= RESULT2 (15 downto 0);    -- 
----                when "100" => 
----                    R4 <= RESULT2 (15 downto 0);    -- 
----                when "101" => 
----                    R5 <= RESULT2 (15 downto 0);    -- 
----                when "110" => 
----                    R6 <= RESULT2 (15 downto 0);    -- 
----                when "111" => 
----                    R7 <= RESULT2 (15 downto 0);    -- 
----                when others => null;
----            end case;
----            case IR(10 downto 7) is    -- RD, will have the remainder
----                when "0000" => R0 <= RESULT2 (31 downto 16);
----                when "0001" => R1 <= RESULT2 (31 downto 16);
----                when "0010" => R2 <= RESULT2 (31 downto 16);
----                when "0011" => R3 <= RESULT2 (31 downto 16);
----                when "0100" => R4 <= RESULT2 (31 downto 16);
----                when "0101" => R5 <= RESULT2 (31 downto 16);
----                when "0110" => R6 <= RESULT2 (31 downto 16);
----                when "0111" => R7 <= RESULT2 (31 downto 16);
----                when "1000" => R8 <= RESULT2 (31 downto 16);
----                when "1001" => R9 <= RESULT2 (31 downto 16);
----                when "1010" => R10 <= RESULT2 (31 downto 16);
----                when "1011" => R11 <= RESULT2 (31 downto 16);
----                when "1100" => R12 <= RESULT2 (31 downto 16);
----                when "1101" => R13 <= RESULT2 (31 downto 16);
----                when "1110" => R14 <= RESULT2 (31 downto 16);
----                when "1111" => R15 <= RESULT2 (31 downto 16);
----                when others => null;
----            end case;    
----        when s_inst_and =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTA and RTB accorind to the values in RD and RS section
----            --        of the instruction (IR).
----            case IR(10 downto 7) is    -- RD
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;
----        when s_inst_and2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            RESULT <= '0' & (RTB and RTA);
----        when s_inst_and3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(2 downto 0) is    -- RT, will have the and result
----                when "000" => R0 <= RESULT (15 downto 0);    -- Save the result, low 16 bit
----                when "001" => R1 <= RESULT (15 downto 0);    -- 
----                when "010" => R2 <= RESULT (15 downto 0);    -- 
----                when "011" => R3 <= RESULT (15 downto 0);    -- 
----                when "100" => R4 <= RESULT (15 downto 0);    -- 
----                when "101" => R5 <= RESULT (15 downto 0);    -- 
----                when "110" => R6 <= RESULT (15 downto 0);    -- 
----                when "111" => R7 <= RESULT (15 downto 0);    -- 
----                when others => null;
----            end case;
----        when s_inst_or =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTA and RTB accorind to the values in RD and RS section
----            --        of the instruction (IR).
----            case IR(10 downto 7) is    -- RD
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;
----        when s_inst_or2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            RESULT <= '0' & (RTB or RTA);
----        when s_inst_or3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(2 downto 0) is    -- RT, will have the and result
----                when "000" => R0 <= RESULT (15 downto 0);    -- Save the result, low 16 bit
----                when "001" => R1 <= RESULT (15 downto 0);    -- 
----                when "010" => R2 <= RESULT (15 downto 0);    -- 
----                when "011" => R3 <= RESULT (15 downto 0);    -- 
----                when "100" => R4 <= RESULT (15 downto 0);    -- 
----                when "101" => R5 <= RESULT (15 downto 0);    -- 
----                when "110" => R6 <= RESULT (15 downto 0);    -- 
----                when "111" => R7 <= RESULT (15 downto 0);    -- 
----                when others => null;
----            end case;
----        when s_inst_xor =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTA and RTB accorind to the values in RD and RS section
----            --        of the instruction (IR).
----            case IR(10 downto 7) is    -- RD
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;
----        when s_inst_xor2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            RESULT <= '0' & (RTB xor RTA);
----        when s_inst_xor3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(2 downto 0) is    -- RT, will have the and result
----                when "000" => R0 <= RESULT (15 downto 0);    -- Save the result, low 16 bit
----                when "001" => R1 <= RESULT (15 downto 0);    -- 
----                when "010" => R2 <= RESULT (15 downto 0);    -- 
----                when "011" => R3 <= RESULT (15 downto 0);    -- 
----                when "100" => R4 <= RESULT (15 downto 0);    -- 
----                when "101" => R5 <= RESULT (15 downto 0);    -- 
----                when "110" => R6 <= RESULT (15 downto 0);    -- 
----                when "111" => R7 <= RESULT (15 downto 0);    -- 
----                when others => null;
----            end case;
----        when s_inst_not =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTB according to the values in RS section
----            --        of the instruction (IR).
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;
----        when s_inst_not2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            RESULT <= '0' & (not (RTB));
----        when s_inst_not3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(10 downto 7) is    -- RD
----                when "0000" => R0 <= RESULT (15 downto 0);
----                when "0001" => R1 <= RESULT (15 downto 0);
----                when "0010" => R2 <= RESULT (15 downto 0);
----                when "0011" => R3 <= RESULT (15 downto 0);
----                when "0100" => R4 <= RESULT (15 downto 0);
----                when "0101" => R5 <= RESULT (15 downto 0);
----                when "0110" => R6 <= RESULT (15 downto 0);
----                when "0111" => R7 <= RESULT (15 downto 0);
----                when "1000" => R8 <= RESULT (15 downto 0);
----                when "1001" => R9 <= RESULT (15 downto 0);
----                when "1010" => R10 <= RESULT (15 downto 0);
----                when "1011" => R11 <= RESULT (15 downto 0);
----                when "1100" => R12 <= RESULT (15 downto 0);
----                when "1101" => R13 <= RESULT (15 downto 0);
----                when "1110" => R14 <= RESULT (15 downto 0);
----                when "1111" => R15 <= RESULT (15 downto 0);
----                when others => null;
----            end case;    
----        when s_inst_srl =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTB according to the values in RS section
----            --        of the instruction (IR).
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;
----        when s_inst_srl2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Shift Right RTB    
----            RESULT <= ('0' & ('0' & RTB(RTB'left downto 1)));
----        when s_inst_srl3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(10 downto 7) is    -- RD
----                when "0000" => R0 <= RESULT (15 downto 0);
----                when "0001" => R1 <= RESULT (15 downto 0);
----                when "0010" => R2 <= RESULT (15 downto 0);
----                when "0011" => R3 <= RESULT (15 downto 0);
----                when "0100" => R4 <= RESULT (15 downto 0);
----                when "0101" => R5 <= RESULT (15 downto 0);
----                when "0110" => R6 <= RESULT (15 downto 0);
----                when "0111" => R7 <= RESULT (15 downto 0);
----                when "1000" => R8 <= RESULT (15 downto 0);
----                when "1001" => R9 <= RESULT (15 downto 0);
----                when "1010" => R10 <= RESULT (15 downto 0);
----                when "1011" => R11 <= RESULT (15 downto 0);
----                when "1100" => R12 <= RESULT (15 downto 0);
----                when "1101" => R13 <= RESULT (15 downto 0);
----                when "1110" => R14 <= RESULT (15 downto 0);
----                when "1111" => R15 <= RESULT (15 downto 0);
----                when others => null;
----            end case;    
----        when s_inst_sll =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTB according to the values in RS section
----            --        of the instruction (IR).
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;
----        when s_inst_sll2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Shift Left A    
----            RESULT <= ('0' & (RTB(RTB'left - 1 downto 0) & '0'));
----        when s_inst_sll3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(10 downto 7) is    -- RD
----                when "0000" => R0 <= RESULT (15 downto 0);
----                when "0001" => R1 <= RESULT (15 downto 0);
----                when "0010" => R2 <= RESULT (15 downto 0);
----                when "0011" => R3 <= RESULT (15 downto 0);
----                when "0100" => R4 <= RESULT (15 downto 0);
----                when "0101" => R5 <= RESULT (15 downto 0);
----                when "0110" => R6 <= RESULT (15 downto 0);
----                when "0111" => R7 <= RESULT (15 downto 0);
----                when "1000" => R8 <= RESULT (15 downto 0);
----                when "1001" => R9 <= RESULT (15 downto 0);
----                when "1010" => R10 <= RESULT (15 downto 0);
----                when "1011" => R11 <= RESULT (15 downto 0);
----                when "1100" => R12 <= RESULT (15 downto 0);
----                when "1101" => R13 <= RESULT (15 downto 0);
----                when "1110" => R14 <= RESULT (15 downto 0);
----                when "1111" => R15 <= RESULT (15 downto 0);
----                when others => null;
----            end case;    
----        when s_inst_ld =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the MEM_ADDR according to the values in RS
----            --        of the instruction (IR).
----            case IR(6 downto 3) is    -- RS                    
----                when "0000" => MEM_ADDR <= R0;
----                when "0001" => MEM_ADDR <= R1;
----                when "0010" => MEM_ADDR <= R2;
----                when "0011" => MEM_ADDR <= R3;
----                when "0100" => MEM_ADDR <= R4;
----                when "0101" => MEM_ADDR <= R5;
----                when "0110" => MEM_ADDR <= R6;
----                when "0111" => MEM_ADDR <= R7;
----                when "1000" => MEM_ADDR <= R8;
----                when "1001" => MEM_ADDR <= R9;
----                when "1010" => MEM_ADDR <= R10;
----                when "1011" => MEM_ADDR <= R11;
----                when "1100" => MEM_ADDR <= R12;
----                when "1101" => MEM_ADDR <= R13;
----                when "1110" => MEM_ADDR <= R14;
----                when "1111" => MEM_ADDR <= R15;
----                when others => null;
----            end case;    
----            MEM_WR <= "0";
----            MEM_EN <= '1';
----        when s_inst_ld2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- wait one cycle to fetch the data in memory 
----        when s_inst_ld3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            case IR(10 downto 7) is    -- RD
----                when "0000" => R0 <= MEM_DATA_OUT;
----                when "0001" => R1 <= MEM_DATA_OUT;
----                when "0010" => R2 <= MEM_DATA_OUT;
----                when "0011" => R3 <= MEM_DATA_OUT;
----                when "0100" => R4 <= MEM_DATA_OUT;
----                when "0101" => R5 <= MEM_DATA_OUT;
----                when "0110" => R6 <= MEM_DATA_OUT;
----                when "0111" => R7 <= MEM_DATA_OUT;
----                when "1000" => R8 <= MEM_DATA_OUT;
----                when "1001" => R9 <= MEM_DATA_OUT;
----                when "1010" => R10 <= MEM_DATA_OUT;
----                when "1011" => R11 <= MEM_DATA_OUT;
----                when "1100" => R12 <= MEM_DATA_OUT;
----                when "1101" => R13 <= MEM_DATA_OUT;
----                when "1110" => R14 <= MEM_DATA_OUT;
----                when "1111" => R15 <= MEM_DATA_OUT;
----                when others => null;
----            end case;    
----        when s_inst_st =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the MEM_ADDR according to the values in RS
----            --        of the instruction (IR).
----            case IR(10 downto 7) is    -- RD
----                when "0000" => MEM_ADDR <= R0;
----                when "0001" => MEM_ADDR <= R1;
----                when "0010" => MEM_ADDR <= R2;
----                when "0011" => MEM_ADDR <= R3;
----                when "0100" => MEM_ADDR <= R4;
----                when "0101" => MEM_ADDR <= R5;
----                when "0110" => MEM_ADDR <= R6;
----                when "0111" => MEM_ADDR <= R7;
----                when "1000" => MEM_ADDR <= R8;
----                when "1001" => MEM_ADDR <= R9;
----                when "1010" => MEM_ADDR <= R10;
----                when "1011" => MEM_ADDR <= R11;
----                when "1100" => MEM_ADDR <= R12;
----                when "1101" => MEM_ADDR <= R13;
----                when "1110" => MEM_ADDR <= R14;
----                when "1111" => MEM_ADDR <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS                    
----                when "0000" => MEM_DATA_IN <= R0;
----                when "0001" => MEM_DATA_IN <= R1;
----                when "0010" => MEM_DATA_IN <= R2;
----                when "0011" => MEM_DATA_IN <= R3;
----                when "0100" => MEM_DATA_IN <= R4;
----                when "0101" => MEM_DATA_IN <= R5;
----                when "0110" => MEM_DATA_IN <= R6;
----                when "0111" => MEM_DATA_IN <= R7;
----                when "1000" => MEM_DATA_IN <= R8;
----                when "1001" => MEM_DATA_IN <= R9;
----                when "1010" => MEM_DATA_IN <= R10;
----                when "1011" => MEM_DATA_IN <= R11;
----                when "1100" => MEM_DATA_IN <= R12;
----                when "1101" => MEM_DATA_IN <= R13;
----                when "1110" => MEM_DATA_IN <= R14;
----                when "1111" => MEM_DATA_IN <= R15;
----                when others => null;
----            end case;    
----            MEM_WR <= "1";
----            MEM_EN <= '1';
----        when s_inst_st2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- wait one cycle to write to the memory
----        when s_inst_st3 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Update the memoery address
----            MEM_ADDR <= PC;    
----            MEM_WR <= "0";
----            MEM_EN <= '1';
----        when s_inst_cmp =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTA according to the values in RD section
----            --        of the instruction (IR).
----            case IR(10 downto 7) is    -- RD
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;    
----        when s_inst_cmp2 =>    
----            if  RTA = RTB then
----                FLAGR(0) <= '1';    -- Set Zero FLAG
----            else     
----                FLAGR(0) <= '0';    -- Unset Zero FLAG
----            end if;    
----            -- Update  the memory address with PC value so next clock 
----            --        we will have the right address fetched.
----            MEM_ADDR <= PC;
--        when s_inst_jmp =>
--            halt <= '1';
--            ROUT0 <= B"000_0000";
--            -- Adjust the PC value by adding the jump adddress
--            -- * Jump address must be in two's complement form
--            if IR(10) = '0' then -- Check the sign of jump address
--                TMP <= PC + ("00000" & IR (10 downto 0));
--            else
--                TMP <= PC + ("11111" & IR (10 downto 0));
--            end if;    
--        when s_inst_jmp2 =>
--            halt <= '0';
--            ROUT0 <= B"000_0000";
--            PC <= TMP;
--            -- Update  the memory address with PC value so next clock 
--            --        we will have the right address fetched.
--            MEM_ADDR <= TMP;  
----        when s_inst_jz =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            if FLAGR(0) = '1' then -- Check the Zero Flag
----                -- Adjust the PC value  by adding the jump adddress
----                -- * Jump address must be in two's complement form
----                if IR(10) = '0' then -- Check the sign of jump address
----                    TMP <= PC + ("00000" & IR (10 downto 0));
----                else
----                    TMP <= PC + ("11111" & IR (10 downto 0));
----                end if;    
----            end if;
----        when s_inst_jz2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            PC <= TMP;
----            -- Update  the memory address with PC value so next clock 
----            --        we will have the right address fetched.
----            MEM_ADDR <= TMP;    
----        when s_inst_jnz =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            if FLAGR(0) = '0' then -- Check the Zero Flag
----                -- Adjust the PC value  by adding the jump adddress
----                -- * Jump address must be in two's complement form
----                if IR(10) = '0' then -- Check the sign of jump address
----                    TMP <= PC + ("00000" & IR (10 downto 0));
----                else
----                    TMP <= PC + ("11111" & IR (10 downto 0));
----                end if;    
----            end if;
----        when s_inst_jnz2 =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            PC <= TMP;
----            -- Update  the memory address with PC value so next clock 
----            --        we will have the right address fetched.
----            MEM_ADDR <= PC;    
--        when s_inst_imd =>    
--            halt <= '0';
--            ROUT0 <= B"000_0111";
--            -- Fetch the next 16-bit, PC is already increased in fetch2 state
--            MEM_ADDR <= PC;
--            MEM_EN <= '1';
--        when s_inst_imd2 =>   
--            halt <= '0';
--            ROUT0 <= B"000_1000";
--            TMP <= PC + '1'; 
--        when s_inst_imd3 =>    
--            halt <= '0';
--            ROUT0 <= B"000_1001";
--            PC <= TMP;
--            MEM_EN <= '0';
--        when s_inst_imd4 =>    
--            halt <= '0';
--            ROUT0 <= B"000_1010";
--            case IR(10 downto 7) is -- RD
--                when "0000" => R0 <= MEM_DATA_OUT;
--                when "0001" => R1 <= MEM_DATA_OUT;
--                when "0010" => R2 <= MEM_DATA_OUT;
--                when "0011" => R3 <= MEM_DATA_OUT;
--                when "0100" => R4 <= MEM_DATA_OUT;
--                when "0101" => R5 <= MEM_DATA_OUT;
--                when "0110" => R6 <= MEM_DATA_OUT;
--                when "0111" => R7 <= MEM_DATA_OUT;
--                when "1000" => R8 <= MEM_DATA_OUT;
--                when "1001" => R9 <= MEM_DATA_OUT;
--                when "1010" => R10 <= MEM_DATA_OUT;
--                when "1011" => R11 <= MEM_DATA_OUT;
--                when "1100" => R12 <= MEM_DATA_OUT;
--                when "1101" => R13 <= MEM_DATA_OUT;
--                when "1110" => R14 <= MEM_DATA_OUT;
--                when "1111" => R15 <= MEM_DATA_OUT;
--                when others => null;
--            end case;    
----        when s_inst_out =>
----            halt <= '0';
----            ROUT0 <= B"000_0000";
----            -- Set the RTA according to the values in RD field
----            --        of the instruction (IR).
----            case IR(6 downto 3) is    -- RS
----                when "0000" => RTA <= R0;
----                when "0001" => RTA <= R1;
----                when "0010" => RTA <= R2;
----                when "0011" => RTA <= R3;
----                when "0100" => RTA <= R4;
----                when "0101" => RTA <= R5;
----                when "0110" => RTA <= R6;
----                when "0111" => RTA <= R7;
----                when "1000" => RTA <= R8;
----                when "1001" => RTA <= R9;
----                when "1010" => RTA <= R10;
----                when "1011" => RTA <= R11;
----                when "1100" => RTA <= R12;
----                when "1101" => RTA <= R13;
----                when "1110" => RTA <= R14;
----                when "1111" => RTA <= R15;
----                when others => null;
----            end case;    
----            case IR(10 downto 7) is -- RD
----                when "0000" => RTB <= R0;
----                when "0001" => RTB <= R1;
----                when "0010" => RTB <= R2;
----                when "0011" => RTB <= R3;
----                when "0100" => RTB <= R4;
----                when "0101" => RTB <= R5;
----                when "0110" => RTB <= R6;
----                when "0111" => RTB <= R7;
----                when "1000" => RTB <= R8;
----                when "1001" => RTB <= R9;
----                when "1010" => RTB <= R10;
----                when "1011" => RTB <= R11;
----                when "1100" => RTB <= R12;
----                when "1101" => RTB <= R13;
----                when "1110" => RTB <= R14;
----                when "1111" => RTB <= R15;
----                when others => null;
----            end case;    
----    when s_inst_out2 =>
----        halt <= '0';
----                -- Update the memoery address
----                MEM_ADDR <= PC;
----                -- For now we support only two output registers.
----                -- "0000" = 0 = ROUT0
----                -- "0001" = 1 = ROUT1
------                case RTB(3 downto 0) is    
------                    when "0000" => ROUT0 <= RTA (6 downto 0);
------                    when "0001" => ROUT1 <= RTA (6 downto 0);
------                    when others => null;
------                end case;
----            ROUT0 <= RTA (6 downto 0);
----            when s_inst_push =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- Set the RTA according to the values in RD field
----                --        of the instruction (IR).
----                case IR(10 downto 7) is    -- RD
----                    when "0000" => RTA <= R0;
----                    when "0001" => RTA <= R1;
----                    when "0010" => RTA <= R2;
----                    when "0011" => RTA <= R3;
----                    when "0100" => RTA <= R4;
----                    when "0101" => RTA <= R5;
----                    when "0110" => RTA <= R6;
----                    when "0111" => RTA <= R7;
----                    when "1000" => RTA <= R8;
----                    when "1001" => RTA <= R9;
----                    when "1010" => RTA <= R10;
----                    when "1011" => RTA <= R11;
----                    when "1100" => RTA <= R12;
----                    when "1101" => RTA <= R13;
----                    when "1110" => RTA <= R14;
----                    when "1111" => RTA <= R15;
----                    when others => null;
----                end case;    
----            when s_inst_push2 =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                MEM_ADDR <= SS - SP(15 downto 0);
----                MEM_DATA_IN <= RTA;        
----                TMP17 <= SP + '1';
----                MEM_WR <= "1";
----                MEM_EN <= '1';
----            when s_inst_push3 =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                SP <= TMP17;
----                -- wait one  cycle to write to stack memory
----            when s_inst_push4 =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- Update the memoery address
----                MEM_ADDR <= PC;    
----                MEM_WR <= "0";
----                MEM_EN <= '1';
----            when s_inst_pop =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                MEM_ADDR <= SS - (SP(15 downto 0) - '1');
----                MEM_WR <= "0";    -- read
----                MEM_EN <= '1';
----            when s_inst_pop2 =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- wait one cycle to read the stack memory 
----            when s_inst_pop3 =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- Update the memoery address
----                MEM_ADDR <= PC;
----                case IR(10 downto 7) is    -- RD
----                    when "0000" => R0 <= MEM_DATA_OUT;
----                    when "0001" => R1 <= MEM_DATA_OUT;
----                    when "0010" => R2 <= MEM_DATA_OUT;
----                    when "0011" => R3 <= MEM_DATA_OUT;
----                    when "0100" => R4 <= MEM_DATA_OUT;
----                    when "0101" => R5 <= MEM_DATA_OUT;
----                    when "0110" => R6 <= MEM_DATA_OUT;
----                    when "0111" => R7 <= MEM_DATA_OUT;
----                    when "1000" => R8 <= MEM_DATA_OUT;
----                    when "1001" => R9 <= MEM_DATA_OUT;
----                    when "1010" => R10 <= MEM_DATA_OUT;
----                    when "1011" => R11 <= MEM_DATA_OUT;
----                    when "1100" => R12 <= MEM_DATA_OUT;
----                    when "1101" => R13 <= MEM_DATA_OUT;
----                    when "1110" => R14 <= MEM_DATA_OUT;
----                    when "1111" => R15 <= MEM_DATA_OUT;
----                    when others => null;
----                end case;    
----                SP <= SP - '1';
----            when s_inst_clrc =>    
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----            -- Update the memoery address
----                MEM_ADDR <= PC;
----                FLAGR(1) <= '0';    -- Clear carry 
----            when s_inst_setc =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- Update the memoery address
----                MEM_ADDR <= PC;
----                FLAGR(1) <= '1';    -- Set carry 
----            when s_inst_call =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- Save the return address
----                RETADDR <= PC;
----                -- Set the PC according to the values in RD field
----                --        of the instruction (IR).
----                case IR(10 downto 7) is    -- RD
----                    when "0000" => PC <= R0;
----                    when "0001" => PC <= R1;
----                    when "0010" => PC <= R2;
----                    when "0011" => PC <= R3;
----                    when "0100" => PC <= R4;
----                    when "0101" => PC <= R5;
----                    when "0110" => PC <= R6;
----                    when "0111" => PC <= R7;
----                    when "1000" => PC <= R8;
----                    when "1001" => PC <= R9;
----                    when "1010" => PC <= R10;
----                    when "1011" => PC <= R11;
----                    when "1100" => PC <= R12;
----                    when "1101" => PC <= R13;
----                    when "1110" => PC <= R14;
----                    when "1111" => PC <= R15;
----                    when others => null;
----                end case;
----            when s_inst_call2 =>
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- wait for PC to be updated. Then:
----                -- Update the memoery address
----                MEM_ADDR <= PC;
----            when s_inst_ret =>    
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- Restore the PC
----                PC <= RETADDR;
----            when s_inst_ret2 =>    
----                halt <= '0';
----                ROUT0 <= B"000_0000";
----                -- wait for PC to be updated. Then:
----                -- Update the memoery address
----                MEM_ADDR <= PC;
--            when s_inst_nop =>
--                halt <= '0';
--                ROUT0 <= B"000_1011";
--                MEM_ADDR <= PC;
--                 halt <= '1';
--           when s_inst_nop2 =>
--                halt <= '0';
--               ROUT0 <= B"000_1100";
--           when s_halt =>
--                halt <= '1';
--               ROUT0 <= B"000_1101";
--                PC <= B"0000_0000_0000_0000";
--               IR <= B"0000_0000_0000_0000";
----               SP <= B"00000_0000_0000_0000";
----               SS <= B"1111_1111_1111_1111"; -- End of RAM block
----               FLAGR <= B"0000_0000_0000_0000";
--               MEM_ADDR <= B"0000_0000_0000_0000";
--               MEM_WR <= "0";
--               MEM_EN <= '0';
--            when others => 
--                halt <= '0';
--                ROUT0 <= B"111_1111";
--                PC <= B"0000_0000_0000_0000";
--                IR <= B"0000_0000_0000_0000";
----                SP <= B"00000_0000_0000_0000";
----                SS <= B"1111_1111_1111_1111"; -- End of RAM block
----                FLAGR <= B"0000_0000_0000_0000";
--                MEM_ADDR <= B"0000_0000_0000_0000";
--                MEM_WR <= "0";
--                MEM_EN <= '0';
--        end case;    
--    end process;
end Behavioral;
