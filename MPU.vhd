-------------------------------------------------------------------------
--
-- CO-PROCESSOR MPU taynara - 25/11/2024 (NEW UPDATE) 
--						  larry, taynara, vitor - 28/10/2024
-- The function of the MPU: 
-- 1) Some operations need the enable signal (ce_n) to be active to start. 
-- 2) For load operations, the output signal needs to be active. 
-- 3) The write operations (writing data in matrices) begin when the write signal (we_n) is active. 
-- 4) The read operations (reading data in matrices) begin when the write signal (we_n) is inactive. 
-- 5) All 9 operations of the MPU are defined in the control label (address range 1024 to 1033). 
-- 6) The matrices are mapped from address 1039 to 1087.

--------------------------------------------------------------------------
-- Tri Stage Data
--------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.STD_LOGIC_1164.all;
use work.R8.all;

ENTITY Tri_Stage IS
    PORT(
        dataIN : in reg16;
        oe_n : in std_logic; 
        dataOUT : out reg16 
    );
END Tri_Stage;

ARCHITECTURE logica OF Tri_Stage IS 
BEGIN
    dataOUT <= dataIN when oe_n = '0' else (others => 'Z');
END logica;

--------------------------------------------------------------------------
-- Co-processor MPU
--------------------------------------------------------------------------
library IEEE;
use IEEE.Std_Logic_1164.all;
use ieee.STD_LOGIC_UNSIGNED.all;
use IEEE.NUMERIC_STD.ALL;
use work.R8.all;

entity MPU is
    port(
        ce_n, we_n, oe_n  : in std_logic;
        intr             : out std_logic;
        address          : in reg16;
        dataIN				 : in  reg16;
        dataOUT          : out reg16
    );
end MPU;

architecture logica of MPU is
    type matriz is array (0 to 15) of reg16; -- defined matriz as vector of register (one register of 16 bits each vector element)
    signal A, B, C: matriz; -- create matriz A, B and C
	 -- maps MPU address
	 type mem1 is array (1024 to (address_SP)) of reg16; 
    constant address_SP: integer := 1087; -- max MPU address 
    
	 
	 -- create stages of stages machine
    type tipo_estado is (idle, finalizado, add, sub, fill_A, fill_B, fill_C, identidade_A, identidade_B, identidade_C, store_A, store_B, store_C, load_A, load_B, load_C, mul_calc, mul_result, mac_calc);
    
	 -- signals of the MPU
	 signal result_32: std_logic_vector(31 downto 0) := (others => '0');
    signal index: integer range 0 to 15 := 0;
	 signal estado: tipo_estado := idle;
	 signal mac_flag: boolean := false;
	 signal data_signal: reg16 := (others => 'Z'); 
	 signal linha: reg4 := (others => 'Z');
	 signal ck: std_logic;
	 
	 -- functions to convert each bollean number to intereger for better manipulation 
    function vetor_para_inteiro(v : std_logic_vector) return integer is
        variable b : integer;
    begin
        b := to_integer(signed(v));
        return b;
    end function;
	 
	 -- create component Tri_Stage
	 component Tri_Stage
		  PORT(
			  dataIN : in reg16;
			  oe_n : in std_logic; 
			  dataOUT : out reg16 
		  );
	 end component;

begin

	 RegData: Tri_Stage port map (data_signal, oe_n, dataOUT);
	
    -- Defined intern clock
	 process
    begin
       ck <= '1', '0' after 50ps;
       wait for 100ps;
    end process;
	 
    process (ck)
    begin
        if ck'event and ck = '0' then
            case estado is
				
					 -- Idle stage
                when idle =>
						  index <= 0;
                    mac_flag <= false;
						  d ata_signal <= (others => 'Z');
						  linha <= (others => 'Z');
                    if ce_n = '0' then
                        if we_n = '0' then -- Write operation
                            case address is
                                when "0000010000000010" => estado <= fill_A;
                                when "0000010000000011" => estado <= fill_B;
                                when "0000010000000100" => estado <= fill_C;
                                when "0000010000000111" => estado <= identidade_A;
                                when "0000010000001000" => estado <= identidade_B;
                                when "0000010000001001" => estado <= identidade_C;
                                when others => 
												if vetor_para_inteiro(address) >= 1039 and vetor_para_inteiro(address) <= 1054 then
													linha <= address(3 downto 0);
													estado <= store_C;
												elsif vetor_para_inteiro(address) >= 1055 and vetor_para_inteiro(address) <= 1071 then
													linha <= address(3 downto 0);
													estado <= store_A;
												elsif vetor_para_inteiro(address) >= 1072 and vetor_para_inteiro(address) <= address_SP then
													linha <= address(3 downto 0);
													estado <= store_B;
												else estado <= idle;
												end if;	
                            end case;
                        elsif we_n = '1' then -- Read operation
                            case address is
                                when "0000010000000000" => estado <= add;
                                when "0000010000000001" => estado <= sub;
                                when "0000010000000101" => estado <= mul_calc;
                                when "0000010000000110" => estado <= mac_calc;
                                when others => 
												if oe_n = '0' then
													 if vetor_para_inteiro(address) >= 1039 and vetor_para_inteiro(address) <= 1054 then
														 linha <= address(3 downto 0);
														 estado <= load_C;
													 elsif vetor_para_inteiro(address) >= 1055 and vetor_para_inteiro(address) <= 1071 then
														 linha <= address(3 downto 0);
														 estado <= load_A;
													 elsif vetor_para_inteiro(address) >= 1072 and vetor_para_inteiro(address) <= address_SP then
														 linha <= address(3 downto 0);
														 estado <= load_B;
													 else estado <= idle;
													 end if;
												end if; 
                            end case;									 
                        end if;
                    end if;
					 
					 -- SUM each element of matrix A and B and store the result in matrix C
                when add =>
                    for i in 0 to 15 loop
                        C(i) <= std_logic_vector(signed(A(i)) + signed(B(i)));
                    end loop;
                    estado <= finalizado;
					 
					 -- SUB each element of matrix A and B and store the result in matrix C
                when sub =>
                    for i in 0 to 15 loop
                        C(i) <= std_logic_vector(signed(A(i)) - signed(B(i)));
                    end loop;
                    estado <= finalizado;
					 
					 -- Store any data value in matrix A
                when fill_A =>
                    for i in 0 to 15 loop
                        A(i) <= dataIN;
                    end loop;
                    estado <= idle;
					 
					 -- Store any data value in matrix B
                when fill_B =>
                    for i in 0 to 15 loop
                        B(i) <= dataIN;
                    end loop;
                    estado <= idle;
					
					 -- Store any data value in matrix C
                when fill_C =>
                    for i in 0 to 15 loop
                        C(i) <= dataIN;
                    end loop;
                    estado <= idle;
					 -- Store any data value in identity matriz A 
                when identidade_A =>
                    for i in 0 to 15 loop
                        if (i mod 5) = 0 then
                            A(i) <= dataIN;
                        else
                            A(i) <= (others => '0');
                        end if;
                    end loop;
                    estado <= idle;
					 
					 -- Store any data value in identity matriz B
                when identidade_B =>
                    for i in 0 to 15 loop
                        if (i mod 5) = 0 then
                            B(i) <= dataIN;
                        else
                            B(i) <= (others => '0');
                        end if;
                    end loop;
                    estado <= idle;
					 
					 -- Store any data value in identity matriz C 
                when identidade_C =>
                    for i in 0 to 15 loop
                        if (i mod 5) = 0 then
                            C(i) <= dataIN;
                        else
                            C(i) <= (others => '0');
                        end if;
                    end loop;
                    estado <= idle;					
					
					-- Load a data value into a specific address sent from the CPU to the MPU in matrix A.
					when load_A =>
							case linha is
								when "0000" => data_signal <= A(0);
                        when "0001" => data_signal <= A(1);
							   when "0010" => data_signal <= A(2);
							   when "0011" => data_signal <= A(3);
							   when "0100" => data_signal <= A(4);
							   when "0101" => data_signal <= A(5);
							   when "0110" => data_signal <= A(6);
							   when "0111" => data_signal <= A(7);
							   when "1000" => data_signal <= A(8);
							   when "1001" => data_signal <= A(9);
							   when "1010" => data_signal <= A(10);
							   when "1011" => data_signal <= A(11);
							   when "1100" => data_signal <= A(12);
							   when "1101" => data_signal <= A(13);
							   when "1110" => data_signal <= A(14);
							   when "1111" => data_signal <= A(15);
                        when others => null;
                    end case;
						  estado <= idle;
					
					-- Load a data value into a specific address sent from the CPU to the MPU in matrix B. 
				   when load_B =>
							case linha is
								when "0000" => data_signal <= B(0);
                        when "0001" => data_signal <= B(1);
							   when "0010" => data_signal <= B(2);
							   when "0011" => data_signal <= B(3);
							   when "0100" => data_signal <= B(4);
							   when "0101" => data_signal <= B(5);
							   when "0110" => data_signal <= B(6);
							   when "0111" => data_signal <= B(7);
							   when "1000" => data_signal <= B(8);
							   when "1001" => data_signal <= B(9);
							   when "1010" => data_signal <= B(10);
							   when "1011" => data_signal <= B(11);
							   when "1100" => data_signal <= B(12);
							   when "1101" => data_signal <= B(13);
							   when "1110" => data_signal <= B(14);
							   when "1111" => data_signal <= B(15);
                        when others => null;
                    end case;
						  estado <= idle;
					
					-- Load a data value into a specific address sent from the CPU to the MPU in matrix C.
					when load_C =>
							case linha is
								when "0000" => data_signal <= C(0);
                        when "0001" => data_signal <= C(1);
							   when "0010" => data_signal <= C(2);
							   when "0011" => data_signal <= C(3);
							   when "0100" => data_signal <= C(4);
							   when "0101" => data_signal <= C(5);
							   when "0110" => data_signal <= C(6);
							   when "0111" => data_signal <= C(7);
							   when "1000" => data_signal <= C(8);
							   when "1001" => data_signal <= C(9);
							   when "1010" => data_signal <= C(10);
							   when "1011" => data_signal <= C(11);
							   when "1100" => data_signal <= C(12);
							   when "1101" => data_signal <= C(13);
							   when "1110" => data_signal <= C(14);
							   when "1111" => data_signal <= C(15);
                        when others => null;
                    end case;
						  estado <= idle;
					 
					 -- Store a data value into a specific address sent from the CPU to the MPU in matrix A.
                when store_A =>
                    case linha is
								when "0000" => A(0) <= dataIN;
                        when "0001" => A(1) <= dataIN;
							   when "0010" => A(2) <= dataIN;
							   when "0011" => A(3) <= dataIN;
							   when "0100" => A(4) <= dataIN;
							   when "0101" => A(5) <= dataIN;
							   when "0110" => A(6) <= dataIN;
							   when "0111" => A(7) <= dataIN;
							   when "1000" => A(8) <= dataIN;
							   when "1001" => A(9) <= dataIN;
							   when "1010" => A(10) <= dataIN;
							   when "1011" => A(11) <= dataIN;
							   when "1100" => A(12) <= dataIN;
							   when "1101" => A(13) <= dataIN;
							   when "1110" => A(14) <= dataIN;
							   when "1111" => A(15) <= dataIN;
                        when others => null;
                    end case;
						  estado <= idle;
					 
					 -- Store a data value into a specific address sent from the CPU to the MPU in matrix B.
                when store_B =>
                    case linha is
								when "0000" => B(0) <= dataIN;
                        when "0001" => B(1) <= dataIN;
							   when "0010" => B(2) <= dataIN;
							   when "0011" => B(3) <= dataIN;
							   when "0100" => B(4) <= dataIN;
							   when "0101" => B(5) <= dataIN;
							   when "0110" => B(6) <= dataIN;
							   when "0111" => B(7) <= dataIN;
							   when "1000" => B(8) <= dataIN;
							   when "1001" => B(9) <= dataIN;
							   when "1010" => B(10) <= dataIN;
							   when "1011" => B(11) <= dataIN;
							   when "1100" => B(12) <= dataIN;
							   when "1101" => B(13) <= dataIN;
							   when "1110" => B(14) <= dataIN;
							   when "1111" => B(15) <= dataIN;
                        when others => null;
                    end case;
						  estado <= idle;
					 
					 -- Store a data value into a specific address sent from the CPU to the MPU in matrix C.
					when store_C =>
							case linha is
								when "0000" => C(0) <= dataIN;
                        when "0001" => C(1) <= dataIN;
							   when "0010" => C(2) <= dataIN;
							   when "0011" => C(3) <= dataIN;
							   when "0100" => C(4) <= dataIN;
							   when "0101" => C(5) <= dataIN;
							   when "0110" => C(6) <= dataIN;
							   when "0111" => C(7) <= dataIN;
							   when "1000" => C(8) <= dataIN;
							   when "1001" => C(9) <= dataIN;
							   when "1010" => C(10) <= dataIN;
							   when "1011" => C(11) <= dataIN;
							   when "1100" => C(12) <= dataIN;
							   when "1101" => C(13) <= dataIN;
							   when "1110" => C(14) <= dataIN;
							   when "1111" => C(15) <= dataIN;
                        when others => null;
                    end case;
						  estado <= idle;
					 
					 -- Make multiplication operation element for element in matrices A, B, or C.
					 when mul_calc =>
							  case index is
									 when 0 =>
										  result_32 <= std_logic_vector((signed(A(0)) * signed(B(0))) + (signed(A(1)) * signed(B(4))) + (signed(A(2)) * signed(B(8))) + (signed(A(3)) * signed(B(12))));
									 when 1 =>
										  result_32 <= std_logic_vector((signed(A(0)) * signed(B(1))) + (signed(A(1)) * signed(B(5))) + (signed(A(2)) * signed(B(9))) + (signed(A(3)) * signed(B(13))));
									 when 2 =>
										  result_32 <= std_logic_vector((signed(A(0)) * signed(B(2))) + (signed(A(1)) * signed(B(6))) + (signed(A(2)) * signed(B(10))) + (signed(A(3)) * signed(B(14))));
									 when 3 =>
										  result_32 <= std_logic_vector((signed(A(0)) * signed(B(3))) + (signed(A(1)) * signed(B(7))) + (signed(A(2)) * signed(B(11))) + (signed(A(3)) * signed(B(15))));
									 when 4 =>
										  result_32 <= std_logic_vector((signed(A(4)) * signed(B(0))) + (signed(A(5)) * signed(B(4))) + (signed(A(6)) * signed(B(8))) + (signed(A(7)) * signed(B(12))));
									 when 5 =>
										  result_32 <= std_logic_vector((signed(A(4)) * signed(B(1))) + (signed(A(5)) * signed(B(5))) + (signed(A(6)) * signed(B(9))) + (signed(A(7)) * signed(B(13))));
									 when 6 =>
										  result_32 <= std_logic_vector((signed(A(4)) * signed(B(2))) + (signed(A(5)) * signed(B(6))) + (signed(A(6)) * signed(B(10))) + (signed(A(7)) * signed(B(14))));
									 when 7 =>
										  result_32 <= std_logic_vector((signed(A(4)) * signed(B(3))) + (signed(A(5)) * signed(B(7))) + (signed(A(6)) * signed(B(11))) + (signed(A(7)) * signed(B(15))));
									 when 8 =>
										  result_32 <= std_logic_vector((signed(A(8)) * signed(B(0))) + (signed(A(9)) * signed(B(4))) + (signed(A(10)) * signed(B(8))) + (signed(A(11)) * signed(B(12))));
									 when 9 =>
										  result_32 <= std_logic_vector((signed(A(8)) * signed(B(1))) + (signed(A(9)) * signed(B(5))) + (signed(A(10)) * signed(B(9))) + (signed(A(11)) * signed(B(13))));
									 when 10 =>
										  result_32 <= std_logic_vector((signed(A(8)) * signed(B(2))) + (signed(A(9)) * signed(B(6))) + (signed(A(10)) * signed(B(10))) + (signed(A(11)) * signed(B(14))));
									 when 11 =>
										  result_32 <= std_logic_vector((signed(A(8)) * signed(B(3))) + (signed(A(9)) * signed(B(7))) + (signed(A(10)) * signed(B(11))) + (signed(A(11)) * signed(B(15))));
									 when 12 =>
										  result_32 <= std_logic_vector((signed(A(12)) * signed(B(0))) + (signed(A(13)) * signed(B(4))) + (signed(A(14)) * signed(B(8))) + (signed(A(15)) * signed(B(12))));
									 when 13 =>
										  result_32 <= std_logic_vector((signed(A(12)) * signed(B(1))) + (signed(A(13)) * signed(B(5))) + (signed(A(14)) * signed(B(9))) + (signed(A(15)) * signed(B(13))));
									 when 14 =>
										  result_32 <= std_logic_vector((signed(A(12)) * signed(B(2))) + (signed(A(13)) * signed(B(6))) + (signed(A(14)) * signed(B(10))) + (signed(A(15)) * signed(B(14))));
									 when 15 =>
										  result_32 <= std_logic_vector((signed(A(12)) * signed(B(3))) + (signed(A(13)) * signed(B(7))) + (signed(A(14)) * signed(B(11))) + (signed(A(15)) * signed(B(15))));
									 when others =>
										  result_32 <= (others => '0'); 
							  end case;
						     estado <= mul_result;
					 
					 -- When mac_flag = false, perform a multiplication operation (store the result of the multiplication of matrix A and B in matrix C).
					 -- But when mac_flag = true, perform a MAC operation (store the result of the multiplication of matrix A and B added to the elements of matrix C).
					 when mul_result =>
						  if mac_flag = true then
								C(index) <= std_logic_vector(signed(C(index)) + signed(result_32(15 downto 0)));
						  else
								C(index) <= result_32(15 downto 0);
						  end if;
						  
						  if index = 15 then
								estado <= finalizado;
						  else
								index <= index + 1;
								estado <= mul_calc;
						  end if;
					 
					 -- Activate the MAC operation flag.
					 when mac_calc =>
						  mac_flag <= true;
						  estado <= mul_calc;
					 
					 -- Finaly stage
                when finalizado =>
						  estado <= idle;

                when others =>
                    null;
            end case;
        end if;
    end process;
	 
	 -- Control intr signal
    process (estado)
    begin
        case estado is
            when idle =>
                intr <= '1';
            when finalizado =>
                intr <= '0';
            when others =>
                null;
        end case;
    end process;

end logica;