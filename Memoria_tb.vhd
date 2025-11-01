LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY Memoria_tb IS
END Memoria_tb;

ARCHITECTURE behavioral OF Memoria_tb IS

    COMPONENT Memoria IS
        PORT (
            Address : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
            Clock : IN STD_LOGIC;
            Wr : IN STD_LOGIC;
            Datain : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
            Dataout : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
        );
    END COMPONENT;

    SIGNAL Address : STD_LOGIC_VECTOR(31 DOWNTO 0) := (others => '0');
    SIGNAL Clock : STD_LOGIC := '0';
    SIGNAL Wr : STD_LOGIC := '0';
    SIGNAL Datain : STD_LOGIC_VECTOR(31 DOWNTO 0) := (others => '0');
    SIGNAL Dataout : STD_LOGIC_VECTOR(31 DOWNTO 0);

BEGIN

    UUT: Memoria PORT MAP (
        Address => Address,
        Clock => Clock,
        Wr => Wr,
        Datain => Datain,
        Dataout => Dataout
    );

    Clock <= NOT Clock AFTER 10 ns; -- 50 MHz clock

    PROCESS
    BEGIN
        -- Reset/initial wait
        WAIT FOR 20 ns;

        -- Test write to address 0
        Address <= X"00000000";
        Datain <= X"12345678";
        Wr <= '1';
        WAIT FOR 20 ns;
        Wr <= '0';

        -- Test read from address 0
        WAIT FOR 20 ns;

        -- Test write to address 4 (next word)
        Address <= X"00000004";
        Datain <= X"ABCDEF00";
        Wr <= '1';
        WAIT FOR 20 ns;
        Wr <= '0';

        -- Test read from address 4
        WAIT FOR 20 ns;

        -- Test byte access (address 1, should affect byte 1)
        Address <= X"00000001";
        Datain <= X"000000FF";
        Wr <= '1';
        WAIT FOR 20 ns;
        Wr <= '0';

        -- Read back
        WAIT FOR 20 ns;

        -- Test edge case: address out of range (high bits set)
        Address <= X"10000000";
        Datain <= X"DEADBEEF";
        Wr <= '1';
        WAIT FOR 20 ns;
        Wr <= '0';

        -- Read back
        WAIT FOR 20 ns;

        WAIT;
    END PROCESS;

END behavioral;
