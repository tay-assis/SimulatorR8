-------------------------------------------------------------------------
--
--  R8 PROCESSOR   -  GOLD VERSION  -  02/MAY/2003
--
--  moraes - 30/09/2001  - project start
--  moraes - 22/11/2001  - instruction decoding bug correction
--  moraes - 22/03/2002  - store instruction correction            
--  moraes - 05/04/2003  - SIDLE state inclusion in the control unit
--  calazans - 02/05/2003  - translation of comments to English. Names of
--    some signals, entities, etc have been changed accordingly
--  larry, taynara, vitor - 28/10/2024 - created co-processor MPU
--	 taynara, vitor - 10/11/2024 - modified R8 processor to include interrupt instruction
--	 taynara - 25/11/2024 - marged co-processor with R8 processor
--
--  Notes: 1) In this version, the register bank is designed using 
--    for-generate VHDL construction
--         2) The top-level processor entity is
--
--      entity processor is
--            port( ck,rst: in std_logic;
--                  dataIN:  in  reg16;     -- There is no bidirectional
--                  dataOUT: out reg16;	  -- port for the data bus.
--                  address: out reg16;	  -- This is not in accordance
--                  ce,rw: out std_logic ); -- to the specification
--      end processor;
--	 		  3) The new modifications to this project are commented as MOD
-- 
-------------------------------------------------------------------------

--------------------------------------------------------------------------
-- package with basic types
--------------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;

package R8 is  
    
  subtype reg16  is std_logic_vector(15 downto 0);
  subtype reg4   is std_logic_vector(3 downto 0);
   
  -- instruction indicates the decoded by the control unit  -- 28 INSTRUCTIONS
  -- the 15 conditional jumps are abstracted as just 3 instruction classes: jumpR, jump, jumpD
 type instruction is  
    ( add, sub, and_i, or_i, xor_i, addi, subi, ldl, ldh, ld, st, sl0, sl1, sr0, sr1,
      notA, nop, halt, ldsp, rts, pop, push, jumpR, jump, jumpD, jsrr, jsr, jsrd, int_r); -- MOD (new instruction int_r)
  
  type microinstruction is record
     mpc:   std_logic_vector(1 downto 0);  -- PC input mux control
     msp:   std_logic;                     -- SP input mux control (stack-pointer)
     mad:   std_logic_vector(1 downto 0);  -- mux to select source for the memory address
     mreg:  std_logic;                     -- mux to select source for the register bank input
     ms2:   std_logic;                     -- register bank source2 mux control
     ma:    std_logic;                     -- ALU A operand mux control
     mb:    std_logic_vector(1 downto 0);  -- ALU B operand mux control
     wpc:   std_logic;                     -- PC write enable
     wsp:   std_logic;                     -- SP write enable
     wir:   std_logic;                     -- IR write enable
     wab:   std_logic;                     -- RegA and RegB write enable
	  -- MOD
	  write_intr:   std_logic;              -- REG_int_r write enable
	  rst_on_intr:  std_logic;					 -- Reset REG_int_r 
	  -- MOD
	  walu:  std_logic;                     -- RALU write enable
     wreg:  std_logic;                     -- Register Bank write enable  
     wnz:   std_logic;                     -- N and Z flags write enable
     wcv:   std_logic;                     -- C and V flags write enable
     ce,rw: std_logic;                     -- Chip enable and R_W controls
     alu:   instruction;                   -- ALU operation specification
  end record;
         
  -- 16-bit register, with asynchronous reset and write enable
  component register16
       port( ck,rst,ce:in std_logic;
             D:in  reg16;
             Q:out reg16 );
  end component;
  
  procedure addAB  (  signal A,B: in  reg16;   signal Cin: in STD_LOGIC;
                       signal S:   out reg16;   signal Cout, Ov:out STD_LOGIC);
                       
  function is_zero  (  signal A:  in  reg16  ) return std_logic ;
  
end R8;

package body R8 is
 
  procedure addAB  (  signal A,B: in  reg16;   signal Cin: in STD_LOGIC;
                      signal S:   out reg16;   signal Cout, Ov:out STD_LOGIC) is             
     variable cy_prev, carry : STD_LOGIC;
  begin    
             for w in 0 to 15 loop
                  if w=0 then carry:=Cin;  end if;
                  S(w) <= A(w) xor B(w) xor carry;
                  cy_prev  := carry;
                  carry := (A(w) and B(w)) or (A(w) and carry) or (B(w) and carry);
             end loop;
             Cout <= carry;
             Ov <= cy_prev xor carry;
  end addAB;
 
  function is_zero  (  signal A:  in  reg16  ) return std_logic is           
      begin     
             if A=x"0000" then return '1'; else return '0';  end if;
  end is_zero;
  
end R8;

--------------------------------------------------------------------------
-- General-purpose register -   SENSITIVE TO THE CLOCK FALLING EDGE
--------------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;
use work.R8.all;

entity register16 is
      port( ck,rst,ce:in std_logic;
            D:in  reg16;
            Q:out reg16 );
end register16;

architecture register16 of register16 is
   begin
      process (ck, rst)
        begin
            if rst = '1' then
                  Q <= (others => '0');
            elsif ck'event and ck = '0' then
                  if ce = '1' then  Q <= D; end if;
            end if;
      end process;
end register16;

--------------------------------------------------------------------------
-- Register BanK (R0..R15) - ALL 16 REGISTERS ARE USED
--------------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;
use ieee.STD_LOGIC_UNSIGNED.all;   
use work.R8.all;

entity reg_bank is
      port(  ck, rst, wreg, rs2: in   std_logic;
             ir, inREG:          in   reg16;
             source1, source2, int_r:   out  reg16     );
end entity;

architecture reg_bank of reg_bank is   
   type bank is array (0 to 15) of reg16;
   signal reg : bank;
   signal wen:   reg16;
   signal destB: std_logic_vector(3 downto 0);
begin            
     
    r1:for i in 0 to 15 generate   
        wen(i) <= '1' when  ir(11 downto 8)=i  and wreg='1'  else '0';
        rx: register16 PORT MAP(ck=>ck, rst=>rst, ce=>wen(i), d=>inREG, q=>reg(i));               
    end generate r1;       
  
    -- source1 selection 
    source1 <=  reg( CONV_INTEGER(ir(7 downto 4)) );

    -- source2 selection 
    destB <= ir(3 downto 0) when rs2 = '0' else ir(11 downto 8);
    source2 <=   reg( CONV_INTEGER(destB) );
   
end reg_bank;

--------------------------------------------------------------------------
--------------------------------------------------------------------------
--  Datapath structural description
--------------------------------------------------------------------------
--------------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;
use IEEE.Std_Logic_unsigned.all;
use work.R8.all;
   
entity datapath is
      port( uins: in microinstruction;
            ck, rst: in std_logic;
            instruction, address : out reg16;
            dataIN:  in  reg16;
            dataOUT: out reg16;
				int_r: out reg16;
            flag: out reg4 );
end datapath;

architecture datapath of datapath is

    component reg_bank
          port( ck, rst, wreg, rs2: in std_logic;
                ir, inREG:           in reg16;
                source1, source2:    out reg16 );
    end component;

    signal dtreg, dtpc, dtsp, s1, s2, outalu, pc, sp, ir, rA, rB, ralu, out_int_r, -- MOD value REG_int_r (out_int_r) --
           opA, opB, addA, addB, add: reg16;
    signal cin, cout, overflow, rst_intr: std_logic;
begin

  --  IR register to instruction output signal
  instruction <= ir;

  --
  --  datapath registers: register bank, pc, sp, ir, rA, rB, ralu
  --
  REGS: reg_bank port map( ck=>ck, rst=>rst, wreg=>uins.wreg, rs2=>uins.ms2,
                         ir=>ir, inREG=> dtreg, source1=>s1, source2=>s2);

  RPC:      register16 port map(ck=>ck, rst=>rst, ce=>uins.wpc,   d=>dtpc,   q=>pc );
                                                                                           
  RSP:      register16 port map(ck=>ck, rst=>rst, ce=>uins.wsp,   d=>dtsp,   q=>sp );

  RIR:      register16 port map(ck=>ck, rst=>rst, ce=>uins.wir,   d=>dataIN, q=>ir );

  REG_A:    register16 port map(ck=>ck, rst=>rst, ce=>uins.wab,   d=>s1,     q=>rA );

  REG_B:    register16 port map(ck=>ck, rst=>rst, ce=>uins.wab,   d=>s2,     q=>rB );

  REG_alu:  register16 port map(ck=>ck, rst=>rst, ce=>uins.walu,  d=>outalu, q=>ralu );
  -- MOD
  REG_int_r:    register16 port map(ck=>ck, rst=>rst_intr, ce=>uins.write_intr,   d=>rA,     q=>out_int_r ); 
  int_r <= out_int_r;
  -- MOD
  -- status flags
  process (ck, rst)
   begin
     if rst = '1' then
           flag <= (others => '0');
     elsif ck'event and ck = '0' then
         
        if uins.wnz='1' then
           flag(0) <= outalu(15);       -- negative flag 
           flag(1) <= is_zero(outalu);  
        end if;
        
        if uins.wcv='1' then
           flag(2) <= cout;      
           flag(3) <= overflow;   
        end if; 
             
     end if;
  end process;

  --
  --  multiplexers
  --
  dtpc <= ralu   when uins.mpc = "01" else       -- operand selection for PC register
          dataIN when uins.mpc = "00" else
			 out_int_r when uins.mpc = "11" else 	 -- MOD data the REG_int_r for PC register
          pc+1;                                  -- by default the PC is incremented;

  dtsp <= sp-1 when uins.msp = '1' else        -- operand selection for SP register
          ralu;

  address  <= ralu  when uins.mad = "00" else  -- selection of who addresses the external RAM
              pc    when uins.mad = "01" else
              sp;

  opA <= ir     when uins.ma = '1' else ra;     -- A operand for the ALU   
      
  opB <= sp     when uins.mb = "01" else    -- B operand for the ALU, or memory
         pc     when uins.mb = "10" else
         rb;    

  dtreg <= dataIN when uins.mreg = '1' else ralu; 
	  -- register bank writing contents selection
      
  -- moraes 21/March - multiplexer for the output data - bug corrected
  dataOUT <=  s2 when ir(15 downto 12)="1010" else opB;    
  
  ---
  ---  ALU - operation depend only on the current instruction 
  ---       (decoded in the control unit)
  ---

  addA <= "00000000" & opA(7 downto 0)       when uins.alu=addi else
           not ("00000000" & opA(7 downto 0)) when uins.alu=subi else           
           opA(11) & opA(11) & opA(11) & opA(11) & opA(11 downto 0)             when  uins.alu=jsrd else
           opA(9) & opA(9) & opA(9) & opA(9) & opA(9) & opA(9)& opA(9 downto 0) when uins.alu=jumpD else
           opA; -- add, sub, ld, st
           
  addB <= not opB  when uins.alu=sub  else
           opB; 
           
  cin <= '1' when uins.alu=sub or uins.alu=subi  else '0';
  
  -- MOD: When rst_intr = 1, reset the data in REG_int_r; otherwise, do not reset.
  rst_intr <= '1' when ((uins.rst_on_intr or rst)='1') else '0';
  
  addAB( addA, addB, cin, add, cout, overflow); 
  
  outalu <= opA and opB                        when uins.alu = and_i else  
            opA or  opB                        when uins.alu = or_i  else   
            opA xor opB                        when uins.alu = xor_i else  
            opB(15 downto 8) & opA(7 downto 0) when uins.alu = ldl   else
            opA(7 downto 0)  & opB(7 downto 0) when uins.alu = ldh   else
            opA(14 downto 0) & '0'             when uins.alu = sl0   else
            opA(14 downto 0) & '1'             when uins.alu = sl1   else
            '0' & opA(15 downto 1)             when uins.alu = sr0   else
            '1' & opA(15 downto 1)             when uins.alu = sr1   else
             not opA                           when uins.alu = notA  else 
             opB + 1                           when uins.alu = rts or uins.alu=pop else  
             RA                                when uins.alu = jump or uins.alu=jsr  or uins.alu=ldsp else      
             add;     -- by default the ALU operation is add!!
  
    
end datapath;

--------------------------------------------------------------------------
--------------------------------------------------------------------------
--  Control Unit behavioral description 
--------------------------------------------------------------------------
--------------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;
use IEEE.Std_Logic_unsigned.all;
use work.R8.all;
entity control_unit is
      port (uins:    out microinstruction;
            rst,ck: in std_logic;
            flag:   in reg4;
				in_intr: in reg16;
				intr_in: in std_logic;
            ir:     in reg16 );
end control_unit;

architecture control_unit of control_unit is

  type type_state  is (Sidle, Sfetch, Srreg, Shalt, Salu,
  Srts, Spop, Sldsp, Sld, Sst, Swbk, Sjmp, Ssbrt, Spush, Sint_r1, Sint_r2, Sint_r3); -- MOD (new states: Sint_r1, Sint_r2 and Sint_r3) 
  -- 13 states
  signal EA, PE :  type_state;

  signal fn, fz, fc, fv, inst_la1, inst_la2, reg_is_not_zero:  std_logic;
  signal i : instruction; 
begin
   
   fn <= flag(0);          -- status flags
   fz <= flag(1);
   fc <= flag(2);
   fv <= flag(3);
   
  ----------------------------------------------------------------------------------------
  -- BLOCK (1/3) - INSTRUCTION DECODING
  ----------------------------------------------------------------------------------------
  i <= add   when ir(15 downto 12)=0  else
       sub   when ir(15 downto 12)=1  else
       and_i when ir(15 downto 12)=2  else
       or_i  when ir(15 downto 12)=3  else
       xor_i when ir(15 downto 12)=4  else
       addi  when ir(15 downto 12)=5  else
       subi  when ir(15 downto 12)=6  else
       ldl   when ir(15 downto 12)=7  else
       ldh   when ir(15 downto 12)=8  else
       ld    when ir(15 downto 12)=9  else
       st    when ir(15 downto 12)=10 else
       sl0   when ir(15 downto 12)=11 and ir(3 downto 0)=0  else
       sl1   when ir(15 downto 12)=11 and ir(3 downto 0)=1  else
       sr0   when ir(15 downto 12)=11 and ir(3 downto 0)=2  else
       sr1   when ir(15 downto 12)=11 and ir(3 downto 0)=3  else
       notA  when ir(15 downto 12)=11 and ir(3 downto 0)=4  else
       nop   when ir(15 downto 12)=11 and ir(3 downto 0)=5  else
       halt  when ir(15 downto 12)=11 and ir(3 downto 0)=6  else
       ldsp  when ir(15 downto 12)=11 and ir(3 downto 0)=7  else
       rts   when ir(15 downto 12)=11 and ir(3 downto 0)=8  else
       pop   when ir(15 downto 12)=11 and ir(3 downto 0)=9  else
       push  when ir(15 downto 12)=11 and ir(3 downto 0)=10 else
		 -- MOD: address of instruction int_r
		 int_r  when ir(15 downto 12)=11 and ir(3 downto 0)=11 else
               
       -- jump instructions ** It is here that the status flags are tested to jump or not to jump *************

       jumpR when ir(15 downto 12)=12 and  ( ir(3 downto 0)=0 or 
                   (ir(3 downto 0)=1 and fn='1') or (ir(3 downto 0)=2 and fz='1') or 
                   (ir(3 downto 0)=3 and fc='1') or (ir(3 downto 0)=4 and fv='1') )   else 
                   
       jump  when ir(15 downto 12)=12 and  ( ir(3 downto 0)=5 or 
                   (ir(3 downto 0)=6 and fn='1') or (ir(3 downto 0)=7 and fz='1') or 
                   (ir(3 downto 0)=8 and fc='1') or (ir(3 downto 0)=9 and fv='1') )   else 

                   
       jumpD  when ir(15 downto 12)=13 or ( ir(15 downto 12)=14 and  
                   ( (ir(11 downto 10)=0 and fn='1') or (ir(11 downto 10)=1 and fz='1') or 
                     (ir(11 downto 10)=2 and fc='1') or (ir(11 downto 10)=3 and fv='1') ) )  else 
                   
       jsrr  when ir(15 downto 12)=12 and ir(3 downto 0)=10 else
       jsr   when ir(15 downto 12)=12 and ir(3 downto 0)=11 else
       jsrd  when ir(15 downto 12)=15 else
       
       nop ;  -- IMPORTANT: default condition in case flag values are '1';
        
  uins.alu <= i;       -- operation that the alu is about to execute

  -- logic and arithmetic instructions (la) of type 1:
  inst_la1 <= '1' when i=add or i=sub or i=and_i or i=or_i or i=xor_i or i=notA or i=sl0 or i=sr0 or i=sl1 or i=sr1 else
              '0';

  -- logic and arithmetic instructions (la) of type 2 (rt on the right side of the expression):
  inst_la2 <= '1' when i=addi or i=subi or i=ldl or i=ldh else
              '0'; 
              
  ----------------------------------------------------------------------------------------
  -- BLOCK (2/3) -  7 multiplexers control generation 
  ----------------------------------------------------------------------------------------

  uins.mpc <= "10" when EA=Sfetch else
              "00" when EA=Srts  else
				  "11" when EA=Sint_r2 else -- MOD: data REG_int_r mux
              "01";                     -- alu mux

  uins.msp <= '1' when  i=jsrr or i=jsr or i=jsrd or i=push or (i=int_r and EA=Sint_r2) else  '0'; -- MOD: Update SP address when executing the instruction intr_r and during stage five.
  -- the SP register employs post-decrement for push / pre-increment for pop
              

  uins.mad <= "10" when EA=Spush or  EA=Ssbrt or EA=Sint_r2 else 	-- MOD: in a subroutine mem is addressed by SP
              "01" when EA=Sfetch else                            -- in an instruction fetch mem is addressed by the PC
              "00";                                 					-- default used is for LD/ST instructions

  -- in which register to write contents just arrived from memory, if any
  uins.mreg <= '1' when i=ld or i=pop else '0';
       
  -- the second source operand (source2) receives the destination register address
  -- when the instruction in question is an arithmetic/logic type 2, a push or memory write,
  -- since register contents must be sent to external memory
  uins.ms2 <= '1' when inst_la2='1' or i=push or EA=Sst or EA=Sint_r2 else '0'; -- MOD insert interrupt state
 
  -- first alu operand is the IR register when instruction is an 
  -- arithmetic/logic type 2 or a jump/jsr with short displacement 
  uins.ma <= '1' when inst_la2='1' or i=jumpD or i=jsrd else '0';  

  uins.mb <= "01"  when  i=rts or i=pop   else      -- to increment the SP register
             "10"  when  i=jumpR or i=jump or i=jumpD or i=jsrr or i=jsr or i=jsrd or EA=Sint_r1  else 
             	-- MOD: in jumps, jsr and int_r instructions, the PC resgister is the ALU second operand
             "00" ;      

  ---------------------------------------------------------------------------------------------
  -- BLOCK (3/3) -  CONTROL FSM - generates the write enable and RAM access commands
  --------------------------------------------------------------------------------------------- 
  
  -- MOD: Update PC data when executing the instruction intr_r and during stage five.
  uins.wpc  <= '1' when (EA=Sfetch or EA=Sjmp  or EA=Ssbrt or EA=Srts or EA=Sint_r2)  else '0';
  
  uins.rst_on_intr <= '1' when EA=Sint_r3 else '0';
  
  reg_is_not_zero <= '1' when is_zero(in_intr) = '0' else '0'; -- MOD: Set the flag if REG_int_r is zero (0) or not zero (1).
	
  uins.wsp  <= '1' when EA=Sldsp  or EA=Srts   or EA=Ssbrt or EA=Spush or EA=Spop or EA=Sint_r2  else '0'; -- NOVO ESTADO 5
  uins.wir  <= '1' when EA=Sfetch                                                  else '0';
  uins.wab  <= '1' when EA=Srreg                                                   else '0';
  -- MOD: write REG_int_r
  uins.write_intr  <= '1' when EA=Salu and i=int_r          			              else '0';
  -- MOD
  uins.walu <= '1' when EA=Salu                                                    else '0';
  uins.wreg <= '1' when EA=Swbk  or EA=Sld   or EA=Spop                            else '0';
  uins.wnz  <= '1' when EA=Salu and (inst_la1='1' or  i=addi or i=subi)            else '0';
  uins.wcv  <= '1' when EA=Salu and (i=add or i=addi or i=sub or i=subi)           else '0';
  
      ---  IMPORTANT !!!!!!!!!!!!!
  uins.ce<='1' when rst='0' and (EA=Sfetch or EA=Srts or EA=Spop or EA=Sld or EA=Ssbrt or EA=Spush or EA=Sst or EA=Sint_r2) else '0'; -- MOD: In stage five of the int_r operation, the flag ce_n needs to be set to 1.
  uins.rw<='1' when EA=Sfetch or EA=Srts or EA=Spop or EA=Sld else '0';
  
  process(rst, ck)
   begin
      if rst='1' then 
		  EA <= Sidle; -- SIDLE is the state the machine stays while processor is being reset
       elsif ck'event and ck='1' then
          if EA=Sidle then
                EA <= Sfetch;
          else
                EA <= PE;
          end if;
     end if;
  end process;
  
  process(EA,i)     -- NEXT state: depends on the PRESENT state and on the current instruction
   begin

    case EA is
                
         when Sidle => PE <=Sidle; -- reset being active, the processor do nothing!       
     --
     -- first clock cycle after reset and after each instruction ends execution
     --
     when Sfetch =>  if i=halt then      -- found HALT => stop generating microinstructions
                         PE <= Shalt;
                     else
                         PE <= Srreg;
                     end if;

     --
     -- second clock cycle of every instruction
     --
     when Shalt  =>  PE <= Shalt;    -- When instruction is HALT lock processor until external reset occurs

     when Srreg =>   PE <= Salu;

     --
     -- third clock cycle of every instruction - there are 9 distinct destinations from here
     --
     when Salu => if i=pop                                 then   PE <= Spop;
                    elsif i=rts                            then   PE <= Srts;
                    elsif i=ldsp                           then   PE <= Sldsp;
                    elsif i=ld                             then   PE <= Sld;
                    elsif i=st                             then   PE <= Sst;
                    elsif inst_la1='1' or inst_la2='1'     then   PE <= Swbk;
                    elsif i=jumpR or i=jump or i=jumpD     then   PE <= Sjmp;
                    elsif i=jsrr or i=jsr or i=jsrd        then   PE <= Ssbrt;
                    elsif i=push                           then   PE <= Spush;
						  elsif i=int_r                          then   PE <= Sint_r1; -- MOD: If executing the int_r operation, go to the next stage (stage four).
                    else  PE <= Sfetch;   -- nop and jumps with flag=0 execute in just 3 clock cycles ** ATTENTION **
                   end if;
		
     -- MOD: New state of interrupt operation.  
	  -- If REG_int_r is not zero and the int_r operation is executing, go to the next stage (stage five).  
	  -- Else, go to the idle stage.
	  when Sint_r1 => 
			if (reg_is_not_zero='1' and intr_in='1') then PE <= Sint_r2;
			else PE <= Sfetch;
			end if;
	  
	  -- MOD: Update the SP address and PC data with REG_int_r data after moving to the next stage (stage six).
	  when Sint_r2 => PE <= Sint_r3;

     --
     -- fourth clock cycle of every instruction - GO BACK TO FETCH
     -- 
     when Spop | Srts | Sldsp | Sld | Sst | Swbk | Sjmp | Ssbrt | Spush | Sint_r3 =>  PE <= Sfetch; -- MOD: In stage six, reset the REG_int_r data and move to the idle stage.

  
   end case;

  end process;

end control_unit;

--------------------------------------------------------------------------
-- Top-level instantiation of the R8 Datapath and Control Unit
--------------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;
use work.R8.all;

entity processor is
      port( ck,rst: in std_logic;
            dataIN:  in  reg16;
				intr_in: in std_logic; -- MOD: Insert the inst_r instruction flag.
            dataOUT: out reg16;
            address: out reg16;
            ce,rw: out std_logic );
end processor;

architecture processor of processor is

      component control_unit
            port( uins:   out microinstruction;
                  ck,rst: in std_logic;
                  flag:   in reg4;
						in_intr: in reg16; -- MOD: Insert REG_int_r data.
						intr_in: in std_logic; -- MOD: Insert inst_r instruction flag.
                  ir:     in reg16 );
      end component;

      component datapath
            port( uins: in microinstruction;
                  ck, rst: in std_logic;
                  instruction, address : out reg16;
                  dataIN:  in  reg16;
                  dataOUT: out reg16;
						int_r: out reg16; -- MOD: Insert inst_r instruction flag.
                  flag: out reg4 );
      end component;

      signal flag: reg4;
      signal uins: microinstruction;
      signal ir: reg16;
		signal int_r: reg16;

begin

    dp: datapath   port map(uins=>uins, ck=>ck, rst=>rst,instruction=>ir,
                            address=>address,dataIN=>dataIN, dataOUT=>dataOUT, int_r=>int_r, flag=>flag);

    ctrl: control_unit port map(uins=>uins, ck=>ck, rst=>rst, flag=>flag, in_intr=>int_r, intr_in=>intr_in, ir=>ir);

    
	 ce <= uins.ce;
    rw <= uins.rw;

end processor;