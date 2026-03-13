-- qbert.vhd
-- Q*bert Votrax sound subsystem (simulation wrapper)
--
-- Direct SC01 interface:
--   phoneme    : 6-bit phoneme index
--   inflection : 2-bit inflection (00 or 10)
--   stb        : phoneme strobe
--   ar         : acknowledge (active-high, from SC01)
--   clk_dac    : 8-bit clock DAC value → DDS frequency
--                sc_hz = 950000 + (clk_dac - 0xA0) * 5500
--
-- Signal chain:
--   VotraxSound (SC01 core + DDS + resampler to 48 kHz)
--     -> audio_out (16-bit signed, 48 kHz)

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity qbert is
    generic (
        CLK_HZ : integer := 50_000_000
    );
    port (
        clk         : in  std_logic;
        reset_n     : in  std_logic;

        phoneme     : in  std_logic_vector(5 downto 0);
        inflection  : in  std_logic_vector(1 downto 0);
        stb         : in  std_logic;
        ar          : out std_logic;
        clk_dac     : in  std_logic_vector(7 downto 0);

        audio_out   : out signed(15 downto 0);
        audio_valid : out std_logic
    );
end entity qbert;

architecture rtl of qbert is
begin

    u_votrax : entity work.VotraxSound
        generic map (
            CLK_HZ => CLK_HZ
        )
        port map (
            clk         => clk,
            reset_n     => reset_n,
            phoneme     => phoneme,
            inflection  => inflection,
            stb         => stb,
            ar          => ar,
            clk_dac     => clk_dac,
            audio_out   => audio_out,
            audio_valid => audio_valid
        );

end architecture rtl;
