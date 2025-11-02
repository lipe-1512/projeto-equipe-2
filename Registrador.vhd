LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY Registrador IS
	PORT(
		Clk		: IN  STD_LOGIC;
		Reset	: IN  STD_LOGIC;
		Load	: IN  STD_LOGIC;
		Entrada : IN  STD_LOGIC_VECTOR (31 downto 0);
		Saida	: OUT STD_LOGIC_VECTOR (31 downto 0)
	);
END Registrador;

ARCHITECTURE behavioral_arch OF Registrador IS
	signal Cluster : STD_LOGIC_VECTOR (31 downto 0);
begin
	Saida <= Cluster;
	process (Clk, Reset)
	begin
		if(Reset = '1') then
			Cluster <= (others => '0');
		elsif (rising_edge(Clk)) then
			if (Load = '1') then
				Cluster <= Entrada;
			end if;
		end if;
	 end process;
END behavioral_arch;
