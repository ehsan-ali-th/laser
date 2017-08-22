----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 06/13/2017 07:55:10 AM
-- Design Name: 
-- Module Name: main - Behavioral
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

entity main is
Port (  main_reset : in STD_LOGIC;
        main_clk : in STD_LOGIC;
        led : out STD_LOGIC;
        halt : out STD_LOGIC;
        GP_ROUT0 : out STD_LOGIC_VECTOR (6 downto 0)
--        GP_ROUT1 : out STD_LOGIC_VECTOR (6 downto 0)
     );
end main;

architecture Behavioral of main is

    component bcd_7seg_decoder is
    port(
            bcd : in std_logic_vector(3 downto 0);  --BCD input
            segment7 : out std_logic_vector(6 downto 0)  -- 7 bit decoded output.
        );
    end component;
    
    component scale_clock is
        Port (  reset : in STD_LOGIC;
                clk_in : in STD_LOGIC;
                clk_out : out STD_LOGIC);
    end component;
    
    component CU is
    Port (  reset : in std_logic := 'X'; 
            clk : in std_logic := 'X';
            halt : out std_logic := 'X';
            ROUT0 : out STD_LOGIC_VECTOR (6 downto 0)
--            ROUT1 : out STD_LOGIC_VECTOR (6 downto 0)
       );
    end component;

    signal clk_1_HZ : STD_LOGIC;
    signal internal_rout0 : std_logic_vector(6 downto 0);

begin

    clk_prescaler0: scale_clock
        port map (
            reset => main_reset,
            clk_in => main_clk,
            clk_out => clk_1_HZ
        );

    seg0: bcd_7seg_decoder 
        port map (
        bcd => internal_rout0 (3 downto 0),
        segment7 => GP_ROUT0   
        );
        
    control_unit: CU
        port map (
            reset => main_reset,
            clk => clk_1_HZ,
            halt => halt,    
            ROUT0 => internal_rout0
--            rout1 => GP_ROUT1
        );
        
--    reset_process : process (main_reset)
--    begin  -- process gen_clk
--    end process reset_process;
        
     led <= clk_1_HZ;   
end Behavioral;
