-------------------------------------------------------------------------
--
-- R8 PROCESSOR TESTBENCH -  GOLD VERSION   -  02/MAY/2003
--
-- It must be observed that the processor is hold in reset
-- (rstR8 <= '1') at the start of simulation, being activated
-- (rstR8 <= '0') just after the end of the object file reading be the
-- testbench.
-- In the vectaddr8.txt the processor execution simulation occurs after
-- 120 ns.
--
-------------------------------------------------------------------------

--------------------------------------------------------------------------
-- Module implementing the simulation of an ASYNCHRONOUS MEMORY INTERFACE
--------------------------------------------------------------------------
library IEEE;
use ieee.STD_LOGIC_UNSIGNED.all;
use ieee.std_logic_1164.all;
use work.R8.all;

entity memRAM is
      port( ce_n, we_n, oe_n: in std_logic;
            address: in    reg16;
            data:    inout reg16 );
end memRAM;

architecture a1 of memRAM is 
   -- 16-bit words, word-addressable memory - Just 1024 WORDS defined - ATTENTION!!!!
   constant address_SP : integer := 1023;     
   type mem1 is array (0 to address_SP) of reg16;
   signal RAM : mem1;
	signal flag : std_logic;
   begin     
      -- writes in memory ASYNCHRONOUSLY
      process(ce_n, we_n, address)
        begin
          if ce_n='0' and we_n='0' then    
             if CONV_INTEGER(address)>=0 and CONV_INTEGER(address)<=address_SP then
                  RAM(CONV_INTEGER(address)) <= data;
             end if;
          end if;   
       end process;   
      
	  flag <= '1' when CONV_INTEGER(address)>=0 	and CONV_INTEGER(address)<=address_SP else '0'; 
	  
     -- memory reading
     data <=  RAM(CONV_INTEGER(address)) when ce_n='0' and oe_n='0' and flag = '1' else
              (others=>'Z');
end a1;

-------------------------------------------------------------------------
--  R8 PROCESSOR SIMULATION TESTBENCH
-------------------------------------------------------------------------
library ieee;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;          
use STD.TEXTIO.all;
use work.R8.all;

entity R8_tb is
end R8_tb;

architecture TB_ARCHITECTURE of R8_tb is
    
      component processor
        port(ck,rst: in std_logic;
            dataIN:  in reg16;
				intr_in: in std_logic; -- MOD: new int_r signal
            dataOUT: out reg16;
            address: out reg16;
            ce,rw: out std_logic
            );
      end component;   
      
      component memRAM 
        port( ce_n, we_n, oe_n: in std_logic;
              address: in  reg16;
              data:    inout reg16 );
      end component;
		
		component MPU
		    port(
				  ce_n, we_n, oe_n  : in std_logic;
				  intr             : out std_logic;
				  address          : in reg16;
				  dataIN				 : in  reg16;
				  dataOUT          : out reg16
			 );
		end component;
        
      signal go, rw, ce, ck, rst, rstR8, ce_n, we_n, oe_n, intr_in : std_logic;
      signal dataR8, data, ad, adR8, address, ins : reg16;    
    
      file ARQ : TEXT open READ_MODE is "../../Code_R8.txt";

    begin
    
    UUT : processor  port map
        (ck=>ck, rst=>rstR8, dataIN=>data, intr_in=>intr_in, dataOUT=>dataR8, address=>adR8, ce=>ce, rw=>rw);
    
    R1 : memRAM    port map
        (ce_n=>ce_n, we_n=>we_n, oe_n=>oe_n, data=>data, address=>ad);
		  
	 M : MPU    port map
		  (ce_n=>ce_n, we_n=>we_n, oe_n=>oe_n, intr=>intr_in, address=>ad, dataIN=>data, dataOUT=>data);

    rst <='1', '0' after 5ns;        -- generates the reset signal 
                         
    process                          -- generates the clock signal 
        begin
        ck <= '1', '0' after 5ns;
        wait for 10ns;
    end process;
    
    -- memory access control signal   
    
    ce_n <= '0' when (ce='1' or go='1') else '1';
    oe_n <= '0' when (ce='1' and rw='1') else '1';       
    we_n <= '0' when (ce='1' and rw='0' and ck='1') or go='1' else '1';    
        
    data <= dataR8 when ce='1' and rw='0' else     -- WRITING
            ins    when go='1' else
            (others => 'Z');    
    
    ad <=  adR8 when go='0' else address;
         
    -- this process loads the programa/data memory during reset
    process
        variable ARQ_LINE : LINE;
        variable line_arq : string(1 to 9);
        variable bin      : STD_LOGIC_VECTOR(1 to 4);
        
        begin  
            
        rstR8 <= '1';   -- hold the processor during file reading
        wait until rst = '1';
        
        while NOT (endfile(ARQ)) loop    -- end file checking
            readline(ARQ, ARQ_LINE);    -- read line of a file
            read(ARQ_LINE, line_arq);
                    
            for w in 1 to 9 loop
                case line_arq(w) is  
                    when '0' => bin := "0000";
                    when '1' => bin := "0001";
                    when '2' => bin := "0010";
                    when '3' => bin := "0011";
                    when '4' => bin := "0100";
                    when '5' => bin := "0101";
                    when '6' => bin := "0110";
                    when '7' => bin := "0111";
                    when '8' => bin := "1000";
                    when '9' => bin := "1001";
                    when 'A' => bin := "1010";
                    when 'B' => bin := "1011";
                    when 'C' => bin := "1100";
                    when 'D' => bin := "1101";
                    when 'E' => bin := "1110";
                    when 'F' => bin := "1111";
                    when others => null;
                end case;
                
                case w is
                    when 1 => address(15 downto 12) <= bin;
                    when 2 => address(11 downto 8)  <= bin;
                    when 3 => address(7 downto 4)   <= bin;
                    when 4 => address(3 downto 0)   <= bin; 
                    when 5 => null;
                    when 6 => ins(15 downto 12) <= bin;
                    when 7 => ins(11 downto 8)  <= bin;
                    when 8 => ins(7 downto 4)   <= bin;
                    when 9 => ins(3 downto 0)   <= bin;
                end case;
                
            end loop;  -- end of the ASCII to binary conversion loop
            
            wait for 2 ns;
            go <= '1';    -- the go signal enables memory writing
            wait for 2 ns;
            go <= '0';    -- removes the go signal
            
        end loop; -- end of file reading loop
        
        rstR8 <= '0';   -- release the processor to execute
        wait for 2 ns;   -- To activate the RST R8 signal
        wait until rst = '1';  -- to Hold again!
    end process;
    
end TB_ARCHITECTURE;