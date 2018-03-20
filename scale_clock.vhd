----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 06/13/2017 01:59:51 PM
-- Design Name: 
-- Module Name: scale_clock - Behavioral
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

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity scale_clock is
    Port (  reset : in STD_LOGIC;
            clk_in : in STD_LOGIC;
            clk_out : out STD_LOGIC);
end scale_clock;

architecture Behavioral of scale_clock is
     signal prescaler : unsigned(27 downto 0);
     signal clk_prescaled : STD_LOGIC;
begin

gen_clk : process (clk_in, clk_prescaled)
begin  -- process gen_clk
    if rising_edge(clk_in) then   -- rising clock edge
        
        if reset = '1' then 
            clk_prescaled <= '0';
            prescaler   <= (others => '0');
        else    
            prescaler <= prescaler + "1";
        end if;
        
        if prescaler = X"0989680" then     -- 10 000 000 in hex
            clk_prescaled <= not clk_prescaled;
            prescaler  <= (others => '0');
        end if;
        
    end if;
    
    clk_out <= clk_prescaled;
end process gen_clk;

end Behavioral;
