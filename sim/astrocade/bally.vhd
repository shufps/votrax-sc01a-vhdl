-- bally.vhd
-- Bally Astrocade Votrax sound subsystem
--
-- I_VOTRAX_DATA byte layout (Bally hardware encoding):
--   bit 7 : infl_sel  (0 = inflection 2, 1 = inflection 0)
--   bit 6 : clk_sel   (0 = 720 kHz, 1 = 780 kHz)
--   bits 5:0 : phoneme
--
-- Signal chain:
--   VotraxSound (SC01-A core + DDS + resampler to 48 kHz)
--     -> bally_rc_filter (4x cascaded RC lowpass, fc ~2584 Hz, on-board)
--     -> audio_out_l / audio_out_r

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity bally is
    generic (
        CLK_HZ : integer := 28_000_000
    );
    port (
        CLK           : in  std_logic;
        I_RESET_L     : in  std_logic;

        I_VOTRAX_DATA : in  std_logic_vector(7 downto 0);
        I_VOTRAX_STB  : in  std_logic;
        O_VOTRAX_AR   : out std_logic;

        s_enable      : in  std_logic;
        audio_out_l   : out std_logic_vector(15 downto 0);
        audio_out_r   : out std_logic_vector(15 downto 0);
        audio_valid   : out std_logic
    );
end entity bally;

architecture rtl of bally is

    signal votrax_audio   : std_logic_vector(15 downto 0);
    signal votrax_valid   : std_logic;

    signal rc_out         : signed(15 downto 0);
    signal rc_valid       : std_logic;

begin

    -- ================================================================
    -- SC01-A synthesizer + DDS + resampler (48 kHz output)
    -- ================================================================
    u_votrax : entity work.VotraxSound
        generic map (
            CLK_HZ => CLK_HZ
        )
        port map (
            CLK           => CLK,
            I_RESET_L     => I_RESET_L,
            I_VOTRAX_DATA => I_VOTRAX_DATA,
            I_VOTRAX_STB  => I_VOTRAX_STB,
            O_VOTRAX_AR   => O_VOTRAX_AR,
            s_enable      => s_enable,
            audio_out_l   => votrax_audio,
            audio_out_r   => open,
            audio_valid   => votrax_valid
        );

    -- ================================================================
    -- Bally on-board RC lowpass filter
    -- 4x (R=110kOhm, C=560pF) -> fc ~2584 Hz, fs=48 kHz
    -- ================================================================
    u_rc : entity work.bally_rc_filter
        port map (
            clk         => CLK,
            reset_n     => I_RESET_L,
            s_in        => signed(votrax_audio),
            s_valid     => votrax_valid,
            s_out       => rc_out,
            s_out_valid => rc_valid
        );

    -- ================================================================
    -- Outputs
    -- ================================================================
    audio_out_l <= std_logic_vector(rc_out);
    audio_out_r <= std_logic_vector(rc_out);
    audio_valid <= rc_valid;

end architecture rtl;
