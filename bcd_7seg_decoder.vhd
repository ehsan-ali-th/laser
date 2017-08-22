----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 06/11/2017 10:17:44 PM
-- Design Name: 
-- Module Name: sevenseg_dec - Behavioral
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
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity bcd_7seg_decoder is
port(
        bcd : in std_logic_vector(3 downto 0);  --BCD input
        segment7 : out std_logic_vector(6 downto 0)  -- 7 bit decoded output.
    );
end bcd_7seg_decoder;

architecture Behavioral of bcd_7seg_decoder is
begin

process (bcd)
begin
    case  bcd is
    when "0000"=> segment7 <="0000001";  -- '0'
    when "0001"=> segment7 <="1001111";  -- '1'
    when "0010"=> segment7 <="0010010";  -- '2'
    when "0011"=> segment7 <="0000110";  -- '3'
    when "0100"=> segment7 <="1001100";  -- '4'
    when "0101"=> segment7 <="0100100";  -- '5'
    when "0110"=> segment7 <="0100000";  -- '6'
    when "0111"=> segment7 <="0001111";  -- '7'
    when "1000"=> segment7 <="0000000";  -- '8'
    when "1001"=> segment7 <="0000100";  -- '9'
     --nothing is displayed when a number more than 9 is given as input.
    when others=> segment7 <="1111111";
    end case;
end process;

end Behavioral;
