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
--   VotraxSound (SC01 core + DDS, variable rate)
--     -> sc01a_resamp (variable rate → fixed 48 kHz, simulation only)
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

    signal votrax_audio   : signed(15 downto 0);
    signal votrax_valid   : std_logic;
    signal phase_inc_resamp : unsigned(15 downto 0) := to_unsigned(29792, 16);

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
            audio_out   => votrax_audio,
            audio_valid => votrax_valid
        );

    -- Compute resampler phase_inc from clk_dac: 32768 * 48000 * 18 / sc01_hz
    -- (real arithmetic is fine here — simulation wrapper only)
    process (clk)
        variable dac_i : integer;
        variable sc_hz : integer;
    begin
        if rising_edge(clk) then
            dac_i := to_integer(unsigned(clk_dac));
            if dac_i < 16#40# then dac_i := 16#40#; end if;
            sc_hz := 950000 + (dac_i - 16#A0#) * 5500;
            phase_inc_resamp <= to_unsigned(
                integer(32768.0 * 864000.0 / real(sc_hz)), 16);
        end if;
    end process;

    u_resamp : entity work.sc01a_resamp
        generic map (
            CLK_HZ      => CLK_HZ,
            SAMPLE_BITS => 16
        )
        port map (
            clk          => clk,
            reset_n      => reset_n,
            s_in         => votrax_audio,
            s_valid      => votrax_valid,
            phase_inc_in => phase_inc_resamp,
            s_out        => audio_out,
            s_out_valid  => audio_valid
        );

end architecture rtl;
