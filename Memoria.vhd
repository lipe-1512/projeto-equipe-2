
--------------------------------------------------------------------------------
-- Title		: Memória da CPU
-- Project		: CPU Multi-ciclo
--------------------------------------------------------------------------------
-- File			: Memoria.vhd
-- Author		: Emannuel Gomes Macêdo <egm@cin.ufpe.br>
--				  Fernando Raposo Camara da Silva <frcs@cin.ufpe.br>
--				  Pedro Machado Manhães de Castro <pmmc@cin.ufpe.br>
--				  Rodrigo Alves Costa <rac2@cin.ufpe.br>
-- Organization : Universidade Federal de Pernambuco
-- Created		: 26/07/2002
-- Last update	: 23/11/2002
-- Plataform	: Flex10K
-- Simulators	: Altera Max+plus II
-- Synthesizers	: 
-- Targets		: 
-- Dependency	: 
--------------------------------------------------------------------------------
-- Description	: Entidade responsável pela leitura e escrita em memória.
--------------------------------------------------------------------------------
-- Copyright (c) notice
--		Universidade Federal de Pernambuco (UFPE).
--		CIn - Centro de Informatica.
--		Developed by computer science undergraduate students.
--		This code may be used for educational and non-educational purposes as 
--		long as its copyright notice remains unchanged. 
--------------------------------------------------------------------------------
-- Revisions		: 1
-- Revision Number	: 1.0
-- Version			: 1.1
-- Date				: 23/11/2002
-- Modifier			: Marcus Vinicius Lima e Machado <mvlm@cin.ufpe.br>
--				  	  Paulo Roberto Santana Oliveira Filho <prsof@cin.ufpe.br>
--					  Viviane Cristina Oliveira Aureliano <vcoa@cin.ufpe.br>
-- Description		:
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Revisions		: 2
-- Revision Number	: 2
-- Version			: 1.2
-- Date				: 31/01/2021
-- Modifier			: André Soares da Silva Filho <assf@cin.ufpe.br>
-- Description		: Simplified the code to work better in ModelSim 20.1.1
--					  -Retirada biblioteca STD_LOGIC_ARITH que conflita com a mais recente NUMERIC_STD
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Revisions		: 3
-- Revision Number	: 3
-- Version			: 1.3
-- Date				: 23/02/2023
-- Modifier			: João Victor da Silva <jvs2@cin.ufpe.br>
-- Description		: Changed byte distribution logic throughout memories
--					  -Cada memória armazena apenas endereços no formato 4n ou 4n + 1 ou 4n + 2 ou 4n + 3,
--						evitando assim que a leitura seja feita em memórias erradas
--------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

LIBRARY lpm;
USE lpm.lpm_components.ALL;

LIBRARY work;
USE work.ram_constants.ALL;

ENTITY Memoria IS
	PORT (
		Address : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		Clock : IN STD_LOGIC;
		Wr : IN STD_LOGIC;
		Datain : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
		Dataout : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
	);
END Memoria;

ARCHITECTURE behavioral_arch OF Memoria IS

	SIGNAL addrL0, addrL1, addrL2, addrL3 : STD_LOGIC_VECTOR (ADDR_WIDTH - 1 DOWNTO 0);
	SIGNAL addrN0, addrN1, addrN2, addrN3 : STD_LOGIC_VECTOR (ADDR_WIDTH - 1 DOWNTO 0);
	
	SIGNAL dataoutS : STD_LOGIC_VECTOR (31 DOWNTO 0);
	SIGNAL datainS : STD_LOGIC_VECTOR (31 DOWNTO 0);
	SIGNAL dataoutS0, dataoutS1, dataoutS2, dataoutS3 : STD_LOGIC_VECTOR (DATA_WIDTH - 1 DOWNTO 0);
	SIGNAL datainS0, datainS1, datainS2, datainS3 : STD_LOGIC_VECTOR (DATA_WIDTH - 1 DOWNTO 0);

	SIGNAL wrS, clockS : STD_LOGIC;
	SIGNAL addr : INTEGER;
	SIGNAL rotdelay0, rotdelay1, rotdelay2 : INTEGER;

	-- FUNÇÃO DE SEGURANÇA: Converte vector para integer ignorando 'X' ou 'U'
	-- Isso evita que a simulação trave no tempo 0.
	function safe_to_integer(vec : std_logic_vector) return integer is
	begin
		if (is_x(vec)) then
			return 0; 
		else
			return to_integer(unsigned(vec));
		end if;
	end function;

BEGIN

	wrS <= wr;
	clockS <= clock;

	-- 1. Conversão Segura do Endereço Base
	addr <= safe_to_integer(Address(7 DOWNTO 0));

	-- 2. Cálculo com PROTEÇÃO MODULO 256
	-- Se o PC passar do endereço 255 (fim do MIF), ele volta para o 0.
	-- Isso impede o "Fatal Error: Index 256 out of range".
	addrL0 <= STD_LOGIC_VECTOR(to_unsigned((addr + 3) mod 256, ADDR_WIDTH));
	addrL1 <= STD_LOGIC_VECTOR(to_unsigned((addr + 2) mod 256, ADDR_WIDTH));
	addrL2 <= STD_LOGIC_VECTOR(to_unsigned((addr + 1) mod 256, ADDR_WIDTH));
	addrL3 <= STD_LOGIC_VECTOR(to_unsigned((addr) mod 256, ADDR_WIDTH));

	addrN0 <= addrL0(ADDR_WIDTH - 1 DOWNTO 2) & "00";
	addrN1 <= addrL1(ADDR_WIDTH - 1 DOWNTO 2) & "01";
	addrN2 <= addrL2(ADDR_WIDTH - 1 DOWNTO 2) & "10";
	addrN3 <= addrL3(ADDR_WIDTH - 1 DOWNTO 2) & "11";

	datainS0 <= datainS(7 DOWNTO 0);
	datainS1 <= datainS(15 DOWNTO 8);
	datainS2 <= datainS(23 DOWNTO 16);
	datainS3 <= datainS(31 DOWNTO 24);

	dataoutS(7 DOWNTO 0) <= dataoutS0;
	dataoutS(15 DOWNTO 8) <= dataoutS1;
	dataoutS(23 DOWNTO 16) <= dataoutS2;
	dataoutS(31 DOWNTO 24) <= dataoutS3;

	-- Proteção na rotação também
	rotdelay0 <= safe_to_integer(Address(1 DOWNTO 0));

	datainS <= datain((31 - rotdelay0 * 8) DOWNTO 0) & datain(31 DOWNTO 31 - (rotdelay0 * 8 - 1));
	dataout <= dataoutS((rotdelay2 * 8 - 1) DOWNTO 0) & dataoutS(31 DOWNTO (rotdelay2 * 8));

	PROCESS (clockS)
	BEGIN
		IF rising_edge(clockS) THEN
			rotdelay1 <= rotdelay0;
			rotdelay2 <= rotdelay1;
		END IF;
	END PROCESS;

	MEM : lpm_ram_dq
	GENERIC MAP(lpm_widthad => ADDR_WIDTH, lpm_width => DATA_WIDTH, lpm_file => INIT_FILE, intended_device_family => "Cyclone V")
	PORT MAP(data => datainS0, Address => addrN0, we => wrS, inclock => clockS, outclock => clockS, q => dataoutS0);

	MEM_plus_One : lpm_ram_dq
	GENERIC MAP(lpm_widthad => ADDR_WIDTH, lpm_width => DATA_WIDTH, lpm_file => INIT_FILE, intended_device_family => "Cyclone V")
	PORT MAP(data => datainS1, Address => addrN1, we => wrS, inclock => clockS, outclock => clockS, q => dataoutS1);

	MEM_plus_Two : lpm_ram_dq
	GENERIC MAP(lpm_widthad => ADDR_WIDTH, lpm_width => DATA_WIDTH, lpm_file => INIT_FILE, intended_device_family => "Cyclone V")
	PORT MAP(data => datainS2, Address => addrN2, we => wrS, inclock => clockS, outclock => clockS, q => dataoutS2);

	MEM_plus_Three : lpm_ram_dq
	GENERIC MAP(lpm_widthad => ADDR_WIDTH, lpm_width => DATA_WIDTH, lpm_file => INIT_FILE, intended_device_family => "Cyclone V")
	PORT MAP(data => datainS3, Address => addrN3, we => wrS, inclock => clockS, outclock => clockS, q => dataoutS3);

END behavioral_arch;